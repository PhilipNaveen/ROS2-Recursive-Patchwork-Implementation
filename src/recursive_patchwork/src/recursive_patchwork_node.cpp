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
    
    // Initialize the patchwork processor with configuration
    PatchworkConfig config;
    config.max_iter = max_iterations_;
    config.th_dist = distance_threshold_;
    config.th_seeds = angle_threshold_;
    processor_ = std::make_unique<RecursivePatchwork>(config);
    
    // Create subscribers
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
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
        auto [ground_points, obstacle_points] = processor_->filterGroundPoints(points);
        
        // Convert results back to ROS messages
        publishResults(ground_points, obstacle_points, msg->header);
        
        auto end_time = this->now();
        auto duration = end_time - start_time;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Processed in %ld ms: %zu ground, %zu obstacles", 
            duration.nanoseconds() / 1000000,
            ground_points.size(),
            obstacle_points.size());
            
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

void RecursivePatchworkNode::publishResults(
    const std::vector<Point3D>& ground_points,
    const std::vector<Point3D>& obstacle_points,
    const std_msgs::msg::Header& header) {
    
    // Publish ground points
    if (!ground_points.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ground_cloud->reserve(ground_points.size());
        
        for (const auto& point : ground_points) {
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
    if (!obstacle_points.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        obstacle_cloud->reserve(obstacle_points.size());
        
        for (const auto& point : obstacle_points) {
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
    publishVisualization(ground_points, obstacle_points, header);
}

void RecursivePatchworkNode::publishVisualization(
    const std::vector<Point3D>& ground_points,
    const std::vector<Point3D>& obstacle_points,
    const std_msgs::msg::Header& header) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create ground points markers (simplified - just show point count)
    if (!ground_points.empty()) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "ground_summary";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at origin
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 2;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.z = 0.5; // Text size
        
        // Color: green for ground
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.text = "Ground: " + std::to_string(ground_points.size()) + " points";
        marker_array.markers.push_back(marker);
    }
    
    // Create obstacle points markers
    if (!obstacle_points.empty()) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "obstacles_summary";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at origin
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 1.5;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.z = 0.5; // Text size
        
        // Color: red for obstacles
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.text = "Obstacles: " + std::to_string(obstacle_points.size()) + " points";
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