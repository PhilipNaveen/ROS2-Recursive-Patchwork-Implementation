#pragma once

#include "recursive_patchwork.hpp"
#include "point_cloud_processor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>

namespace recursive_patchwork {

class RecursivePatchworkNode : public rclcpp::Node {
public:
    RecursivePatchworkNode();

private:
    // Callback for incoming point clouds
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // Publish processing results
    void publishResults(const ProcessingResult& result, const std_msgs::msg::Header& header);
    
    // Publish visualization markers
    void publishVisualization(const ProcessingResult& result, const std_msgs::msg::Header& header);
    
    // Parameters
    std::string input_topic_;
    std::string ground_topic_;
    std::string obstacles_topic_;
    std::string visualization_topic_;
    int min_points_;
    int max_iterations_;
    double distance_threshold_;
    double angle_threshold_;
    
    // Core processor
    std::unique_ptr<PointCloudProcessor> processor_;
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
};

} // namespace recursive_patchwork 