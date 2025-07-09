#include "lidar_fusion_node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>

namespace recursive_patchwork {

LidarFusionNode::LidarFusionNode() 
    : Node("lidar_fusion_node") {
    
    RCLCPP_INFO(this->get_logger(), "Initializing LiDAR Fusion Node");
    
    // Load parameters
    loadParameters();
    
    // Initialize TF components
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize fusion engine
    fusion_engine_ = std::make_unique<LidarFusion>();
    
    // Setup ROS2 components
    setupPublisher();
    setupSubscribers();
    
    RCLCPP_INFO(this->get_logger(), "LiDAR Fusion Node initialized with %zu LiDAR topics", lidar_configs_.size());
}

LidarFusionNode::~LidarFusionNode() {
}

void LidarFusionNode::loadParameters() {
    // Declare parameters with defaults
    this->declare_parameter("output_frame_id", "base_link");
    this->declare_parameter("fused_topic_name", "/fused_pointcloud");
    this->declare_parameter("use_tf_static", true);
    this->declare_parameter("transform_timeout", 0.1);
    this->declare_parameter("min_clouds_to_fuse", 1);
    this->declare_parameter("remove_ego_vehicle", true);
    
    // Load parameters
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();
    fused_topic_name_ = this->get_parameter("fused_topic_name").as_string();
    use_tf_static_ = this->get_parameter("use_tf_static").as_bool();
    transform_timeout_ = this->get_parameter("transform_timeout").as_double();
    min_clouds_to_fuse_ = this->get_parameter("min_clouds_to_fuse").as_int();
    remove_ego_vehicle_ = this->get_parameter("remove_ego_vehicle").as_bool();
    
    // Load LiDAR configurations
    this->declare_parameter("lidar_topics", std::vector<std::string>{});
    auto topic_names = this->get_parameter("lidar_topics").as_string_array();
    
    for (size_t i = 0; i < topic_names.size(); ++i) {
        LidarTopicConfig config;
        config.topic_name = topic_names[i];
        
        // Load frame parameters for each LiDAR
        std::string frame_param = "lidar_" + std::to_string(i) + "_frame_id";
        std::string target_param = "lidar_" + std::to_string(i) + "_target_frame";
        std::string radius_param = "lidar_" + std::to_string(i) + "_ego_radius";
        
        this->declare_parameter(frame_param, config.topic_name + "_link");
        this->declare_parameter(target_param, output_frame_id_);
        this->declare_parameter(radius_param, 2.5);
        
        config.frame_id = this->get_parameter(frame_param).as_string();
        config.target_frame = this->get_parameter(target_param).as_string();
        config.ego_radius = this->get_parameter(radius_param).as_double();
        
        lidar_configs_.push_back(config);
        
        RCLCPP_INFO(this->get_logger(), "LiDAR %zu: %s -> %s (frame: %s, radius: %.2f)", 
                   i, config.topic_name.c_str(), config.target_frame.c_str(), 
                   config.frame_id.c_str(), config.ego_radius);
    }
    
    // If no topics specified, use default 3-LiDAR setup
    if (lidar_configs_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No LiDAR topics specified, using default 3-LiDAR configuration");
        
        std::vector<std::string> default_topics = {"/lidar_front", "/lidar_left", "/lidar_right"};
        std::vector<std::string> default_frames = {"lidar_front_link", "lidar_left_link", "lidar_right_link"};
        
        for (size_t i = 0; i < default_topics.size(); ++i) {
            LidarTopicConfig config;
            config.topic_name = default_topics[i];
            config.frame_id = default_frames[i];
            config.target_frame = output_frame_id_;
            config.ego_radius = 2.5f;
            lidar_configs_.push_back(config);
        }
    }
}

void LidarFusionNode::setupPublisher() {
    fused_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        fused_topic_name_, rclcpp::QoS(10));
    
    RCLCPP_INFO(this->get_logger(), "Publishing fused point cloud on: %s", fused_topic_name_.c_str());
}

void LidarFusionNode::setupSubscribers() {
    lidar_subs_.reserve(lidar_configs_.size());
    
    for (const auto& config : lidar_configs_) {
        if (!config.enabled) continue;
        
        auto subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            config.topic_name,
            rclcpp::QoS(10),
            [this, config](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->lidarCallback(msg, config.topic_name);
            }
        );
        
        lidar_subs_.push_back(subscription);
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", config.topic_name.c_str());
    }
}

void LidarFusionNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
                                   const std::string& topic_name) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Store the latest cloud
    latest_clouds_[topic_name] = msg;
    
    // Process fusion if we have enough clouds
    if (shouldFuse()) {
        processFusion();
    }
}

bool LidarFusionNode::shouldFuse() {
    return latest_clouds_.size() >= static_cast<size_t>(min_clouds_to_fuse_);
}

void LidarFusionNode::processFusion() {
    RCLCPP_DEBUG(this->get_logger(), "Processing fusion with %zu clouds", latest_clouds_.size());
    
    // Update cached transforms
    updateCachedTransforms();
    
    // Convert and transform point clouds
    std::vector<std::vector<Point3D>> transformed_clouds;
    transformed_clouds.reserve(latest_clouds_.size());
    
    for (const auto& config : lidar_configs_) {
        auto it = latest_clouds_.find(config.topic_name);
        if (it == latest_clouds_.end()) continue;
        
        // Convert to internal format
        auto points = convertPointCloud2ToPoints(*it->second);
        if (points.empty()) continue;
        
        // Apply transform if needed
        if (config.frame_id != output_frame_id_) {
            auto transform_it = cached_transforms_.find(config.frame_id + "_to_" + output_frame_id_);
            if (transform_it != cached_transforms_.end()) {
                points = fusion_engine_->applyTransform(points, transform_it->second);
            } else {
                RCLCPP_WARN(this->get_logger(), "No transform available for %s -> %s", 
                           config.frame_id.c_str(), output_frame_id_.c_str());
                continue;
            }
        }
        
        // Remove ego vehicle if enabled
        if (remove_ego_vehicle_) {
            points = fusion_engine_->removeEgoVehicle(points, config.ego_radius);
        }
        
        transformed_clouds.push_back(points);
    }
    
    if (transformed_clouds.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid point clouds to fuse");
        return;
    }
    
    // Fuse point clouds
    auto fused_points = fusion_engine_->fuseLidarPointClouds(transformed_clouds);
    
    // Publish result
    publishFusedCloud(fused_points);
}

std::vector<Point3D> LidarFusionNode::convertPointCloud2ToPoints(const sensor_msgs::msg::PointCloud2& cloud) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    
    std::vector<Point3D> points;
    points.reserve(pcl_cloud.size());
    
    for (const auto& pcl_point : pcl_cloud) {
        if (std::isfinite(pcl_point.x) && std::isfinite(pcl_point.y) && std::isfinite(pcl_point.z)) {
            points.emplace_back(pcl_point.x, pcl_point.y, pcl_point.z);
        }
    }
    
    return points;
}

sensor_msgs::msg::PointCloud2 LidarFusionNode::convertPointsToPointCloud2(const std::vector<Point3D>& points, 
                                                                          const std::string& frame_id) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.reserve(points.size());
    
    for (const auto& point : points) {
        pcl_cloud.emplace_back(point.x, point.y, point.z);
    }
    
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = frame_id;
    ros_cloud.header.stamp = this->now();
    
    return ros_cloud;
}

bool LidarFusionNode::getTransform(const std::string& from_frame, const std::string& to_frame, 
                                  Eigen::Matrix4f& transform) {
    try {
        geometry_msgs::msg::TransformStamped tf_transform;
        
        if (use_tf_static_) {
            tf_transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
        } else {
            tf_transform = tf_buffer_->lookupTransform(to_frame, from_frame, this->now(), 
                                                     rclcpp::Duration::from_seconds(transform_timeout_));
        }
        
        // Convert to Eigen matrix
        tf2::Transform tf2_transform;
        tf2::fromMsg(tf_transform.transform, tf2_transform);
        
        transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = Eigen::Map<const Eigen::Matrix3d>(tf2_transform.getBasis().getData()).cast<float>();
        transform.block<3, 1>(0, 3) = tf2_transform.getOrigin().cast<float>();
        
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return false;
    }
}

void LidarFusionNode::updateCachedTransforms() {
    for (const auto& config : lidar_configs_) {
        if (config.frame_id == output_frame_id_) continue;
        
        std::string transform_key = config.frame_id + "_to_" + output_frame_id_;
        Eigen::Matrix4f transform;
        
        if (getTransform(config.frame_id, output_frame_id_, transform)) {
            cached_transforms_[transform_key] = transform;
        }
    }
}

void LidarFusionNode::publishFusedCloud(const std::vector<Point3D>& fused_points) {
    if (fused_points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points to publish");
        return;
    }
    
    auto ros_cloud = convertPointsToPointCloud2(fused_points, output_frame_id_);
    fused_cloud_pub_->publish(ros_cloud);
    
    RCLCPP_INFO(this->get_logger(), "Published fused point cloud with %zu points", fused_points.size());
}

} // namespace recursive_patchwork 