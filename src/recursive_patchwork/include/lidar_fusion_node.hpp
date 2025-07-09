#pragma once

#include "lidar_fusion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace recursive_patchwork {

struct LidarTopicConfig {
    std::string topic_name;
    std::string frame_id;
    std::string target_frame;
    float ego_radius = 2.5f;
    bool enabled = true;
};

class LidarFusionNode : public rclcpp::Node {
public:
    LidarFusionNode();
    ~LidarFusionNode();

private:
    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_subs_;
    
    // TF components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Configuration
    std::vector<LidarTopicConfig> lidar_configs_;
    std::string output_frame_id_;
    std::string fused_topic_name_;
    
    // Data storage
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> latest_clouds_;
    std::unordered_map<std::string, Eigen::Matrix4f> cached_transforms_;
    std::mutex data_mutex_;
    
    // Fusion engine
    std::unique_ptr<LidarFusion> fusion_engine_;
    
    // Parameters
    bool use_tf_static_;
    double transform_timeout_;
    int min_clouds_to_fuse_;
    bool remove_ego_vehicle_;
    
    // Methods
    void loadParameters();
    void setupSubscribers();
    void setupPublisher();
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& topic_name);
    void processFusion();
    std::vector<Point3D> convertPointCloud2ToPoints(const sensor_msgs::msg::PointCloud2& cloud);
    sensor_msgs::msg::PointCloud2 convertPointsToPointCloud2(const std::vector<Point3D>& points, const std::string& frame_id);
    bool getTransform(const std::string& from_frame, const std::string& to_frame, Eigen::Matrix4f& transform);
    void updateCachedTransforms();
    bool shouldFuse();
    void publishFusedCloud(const std::vector<Point3D>& fused_points);
};

} // namespace recursive_patchwork 