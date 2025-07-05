#pragma once

#include "recursive_patchwork.hpp"
#include <string>
#include <vector>
#include <memory>

// ROS2 includes - only if ROS2 is available
#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#endif

namespace recursive_patchwork {

class RosbagLoader {
public:
    RosbagLoader();
    ~RosbagLoader();

    // Bag file operations
    bool openBag(const std::string& bag_path);
    void closeBag();
    bool isBagOpen() const;

    // Topic operations
    std::vector<std::string> getTopicNames() const;
    std::vector<std::string> getPointCloudTopics() const;
    size_t getMessageCount(const std::string& topic_name) const;

    // Point cloud loading
    std::vector<Point3D> loadPointCloud(const std::string& topic_name, size_t frame_number);
    std::vector<std::vector<Point3D>> loadMultiplePointClouds(
        const std::vector<std::string>& topic_names, 
        size_t frame_number);

    // Bag format detection
    static bool isMCAPFormat(const std::string& bag_path);
    static bool isDB3Format(const std::string& bag_path);

    // Error handling
    std::string getLastError() const { return last_error_; }
    void clearError() { last_error_.clear(); }

private:
#ifdef USE_ROS2
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
#endif
    std::string bag_path_;
    mutable std::string last_error_;
    bool bag_open_;

    // Helper functions
    bool initializeReader();
    std::vector<Point3D> convertPointCloud2ToPoints(const void* msg_data);
    bool seekToMessage(const std::string& topic_name, size_t frame_number);
    
    // MCAP specific functions
    bool loadMCAPPointCloud(const std::string& topic_name, size_t frame_number, std::vector<Point3D>& points);
    
    // DB3 specific functions  
    bool loadDB3PointCloud(const std::string& topic_name, size_t frame_number, std::vector<Point3D>& points);
};

} // namespace recursive_patchwork 