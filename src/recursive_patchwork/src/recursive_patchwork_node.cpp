#include "recursive_patchwork_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace recursive_patchwork {

RecursivePatchworkNode::RecursivePatchworkNode() 
    : Node("recursive_patchwork_node") {
    
    // Declare parameters
    this->declare_parameter("input_topic", "/lidar/points");
    this->declare_parameter("ground_topic", "/patchwork/ground");
    this->declare_parameter("obstacles_topic", "/patchwork/obstacles");
    this->declare_parameter("visualization_topic", "/patchwork/visualization");
    this->declare_parameter("min_points", 100);
    this->declare_parameter("max_iterations", 50);
    this->declare_parameter("distance_threshold", 0.1);
    this->declare_parameter("angle_threshold", 0.1);
    
    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    ground_topic_ = this->get_parameter("ground_topic").as_string();
    obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();
    visualization_topic_ = this->get_parameter("visualization_topic").as_string();
    min_points_ = this->get_parameter("min_points").as_int();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    angle_threshold_ = this->get_parameter("angle_threshold").as_double();
    
    // Initialize the patchwork processor
    processor_ = std::make_unique<PointCloudProcessor>(
        min_points_, max_iterations_, distance_threshold_, angle_threshold_);
    
    // Create subscribers
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, 10,
        std::bind(&RecursivePatchworkNode::pointCloudCallback, this, std::placeholders::_1));
    
    // Create publishers
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_topic_, 10);
    obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        obstacles_topic_, 10);
    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        visualization_topic_, 10);
    
    RCLCPP_INFO(this->get_logger(), "Recursive Patchwork Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing ground to: %s", ground_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing obstacles to: %s", obstacles_topic_.c_str());
}

void RecursivePatchworkNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    
    auto start_time = this->now();
    
    try {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Processing point cloud with %zu points", cloud->size());
        
        // Convert PCL to our internal format
        std::vector<Point3D> points;
        points.reserve(cloud->size());
        
        for (const auto& pcl_point : cloud->points) {
            Point3D point;
            point.x = pcl_point.x;
            point.y = pcl_point.y;
            point.z = pcl_point.z;
            points.push_back(point);
        }
        
        // Process with patchwork algorithm
        auto result = processor_->processPointCloud(points);
        
        // Convert results back to ROS messages
        publishResults(result, msg->header);
        
        auto end_time = this->now();
        auto duration = end_time - start_time;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Processed in %ld ms: %zu ground, %zu obstacles", 
            duration.nanoseconds() / 1000000,
            result.ground_points.size(),
            result.obstacle_points.size());
            
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

void RecursivePatchworkNode::publishResults(
    const ProcessingResult& result, 
    const std_msgs::msg::Header& header) {
    
    // Publish ground points
    if (!result.ground_points.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ground_cloud->reserve(result.ground_points.size());
        
        for (const auto& point : result.ground_points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            ground_cloud->push_back(pcl_point);
        }
        
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_cloud, ground_msg);
        ground_msg.header = header;
        ground_msg.header.frame_id = header.frame_id;
        ground_pub_->publish(ground_msg);
    }
    
    // Publish obstacle points
    if (!result.obstacle_points.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        obstacle_cloud->reserve(result.obstacle_points.size());
        
        for (const auto& point : result.obstacle_points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            obstacle_cloud->push_back(pcl_point);
        }
        
        sensor_msgs::msg::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
        obstacle_msg.header = header;
        obstacle_msg.header.frame_id = header.frame_id;
        obstacles_pub_->publish(obstacle_msg);
    }
    
    // Publish visualization markers
    publishVisualization(result, header);
}

void RecursivePatchworkNode::publishVisualization(
    const ProcessingResult& result, 
    const std_msgs::msg::Header& header) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create ground plane markers
    for (size_t i = 0; i < result.ground_planes.size(); ++i) {
        const auto& plane = result.ground_planes[i];
        
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "ground_planes";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at plane center
        marker.pose.position.x = plane.center.x;
        marker.pose.position.y = plane.center.y;
        marker.pose.position.z = plane.center.z;
        
        // Orientation based on normal
        // This is a simplified approach - you might want more sophisticated orientation calculation
        marker.pose.orientation.w = 1.0;
        
        // Scale based on plane size
        marker.scale.x = plane.size_x;
        marker.scale.y = plane.size_y;
        marker.scale.z = 0.1; // Thin plane
        
        // Color: green for ground
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        
        marker_array.markers.push_back(marker);
    }
    
    // Create obstacle markers
    for (size_t i = 0; i < result.obstacle_clusters.size(); ++i) {
        const auto& cluster = result.obstacle_clusters[i];
        
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "obstacles";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at cluster center
        marker.pose.position.x = cluster.center.x;
        marker.pose.position.y = cluster.center.y;
        marker.pose.position.z = cluster.center.z;
        marker.pose.orientation.w = 1.0;
        
        // Scale based on cluster size
        marker.scale.x = cluster.radius * 2;
        marker.scale.y = cluster.radius * 2;
        marker.scale.z = cluster.radius * 2;
        
        // Color: red for obstacles
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.7;
        
        marker_array.markers.push_back(marker);
    }
    
    visualization_pub_->publish(marker_array);
}

} // namespace recursive_patchwork

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<recursive_patchwork::RecursivePatchworkNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Recursive Patchwork Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 