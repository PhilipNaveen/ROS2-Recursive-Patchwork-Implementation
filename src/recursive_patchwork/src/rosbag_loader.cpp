#include "rosbag_loader.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>

namespace recursive_patchwork {

RosbagLoader::RosbagLoader() : bag_open_(false) {
}

RosbagLoader::~RosbagLoader() {
    closeBag();
}

bool RosbagLoader::openBag(const std::string& bag_path) {
#ifdef USE_ROS2
    try {
        bag_path_ = bag_path;
        
        if (!std::filesystem::exists(bag_path)) {
            last_error_ = "Bag file does not exist: " + bag_path;
            return false;
        }
        
        if (!initializeReader()) {
            return false;
        }
        
        bag_open_ = true;
        return true;
    } catch (const std::exception& e) {
        last_error_ = "Exception opening bag: " + std::string(e.what());
        return false;
    }
#else
    last_error_ = "ROS2 support not available";
    return false;
#endif
}

void RosbagLoader::closeBag() {
#ifdef USE_ROS2
    if (reader_) {
        reader_.reset();
    }
#endif
    bag_open_ = false;
    bag_path_.clear();
}

bool RosbagLoader::isBagOpen() const {
    return bag_open_;
}

std::vector<std::string> RosbagLoader::getTopicNames() const {
#ifdef USE_ROS2
    if (!bag_open_ || !reader_) {
        return {};
    }
    
    try {
        auto metadata = reader_->get_metadata();
        std::vector<std::string> topics;
        for (const auto& topic : metadata.topics_with_message_count) {
            topics.push_back(topic.topic_metadata.name);
        }
        return topics;
    } catch (const std::exception& e) {
        last_error_ = "Exception getting topics: " + std::string(e.what());
        return {};
    }
#else
    return {};
#endif
}

std::vector<std::string> RosbagLoader::getPointCloudTopics() const {
    auto all_topics = getTopicNames();
    std::vector<std::string> point_cloud_topics;
    
    for (const auto& topic : all_topics) {
        if (topic.find("point") != std::string::npos || 
            topic.find("cloud") != std::string::npos ||
            topic.find("lidar") != std::string::npos) {
            point_cloud_topics.push_back(topic);
        }
    }
    
    return point_cloud_topics;
}

size_t RosbagLoader::getMessageCount(const std::string& topic_name) const {
#ifdef USE_ROS2
    if (!bag_open_ || !reader_) {
        return 0;
    }
    
    try {
        auto metadata = reader_->get_metadata();
        for (const auto& topic : metadata.topics_with_message_count) {
            if (topic.topic_metadata.name == topic_name) {
                return topic.message_count;
            }
        }
    } catch (const std::exception& e) {
        last_error_ = "Exception getting message count: " + std::string(e.what());
    }
#endif
    return 0;
}

std::vector<Point3D> RosbagLoader::loadPointCloud(const std::string& topic_name, size_t frame_number) {
#ifdef USE_ROS2
    if (!bag_open_ || !reader_) {
        last_error_ = "Bag not open";
        return {};
    }
    
    try {
        std::vector<Point3D> points;
        
        // Determine bag format and load accordingly
        if (isMCAPFormat(bag_path_)) {
            if (!loadMCAPPointCloud(topic_name, frame_number, points)) {
                return {};
            }
        } else if (isDB3Format(bag_path_)) {
            if (!loadDB3PointCloud(topic_name, frame_number, points)) {
                return {};
            }
        } else {
            // Try generic approach
            if (!seekToMessage(topic_name, frame_number)) {
                return {};
            }
            
            auto message = reader_->read_next();
            if (!message) {
                last_error_ = "No message found at frame " + std::to_string(frame_number);
                return {};
            }
            
            points = convertPointCloud2ToPoints(message->serialized_data.get());
        }
        
        return points;
    } catch (const std::exception& e) {
        last_error_ = "Exception loading point cloud: " + std::string(e.what());
        return {};
    }
#else
    last_error_ = "ROS2 support not available";
    return {};
#endif
}

std::vector<std::vector<Point3D>> RosbagLoader::loadMultiplePointClouds(
    const std::vector<std::string>& topic_names, size_t frame_number) {
    
    std::vector<std::vector<Point3D>> point_clouds;
    point_clouds.reserve(topic_names.size());
    
    for (const auto& topic : topic_names) {
        auto points = loadPointCloud(topic, frame_number);
        point_clouds.push_back(points);
    }
    
    return point_clouds;
}

bool RosbagLoader::isMCAPFormat(const std::string& bag_path) {
    std::ifstream file(bag_path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Check for MCAP magic number
    char magic[8];
    file.read(magic, 8);
    return (magic[0] == 'M' && magic[1] == 'C' && magic[2] == 'A' && magic[3] == 'P');
}

bool RosbagLoader::isDB3Format(const std::string& bag_path) {
    std::ifstream file(bag_path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Check for SQLite magic number
    char magic[16];
    file.read(magic, 16);
    return (magic[0] == 'S' && magic[1] == 'Q' && magic[2] == 'L' && magic[3] == 'i' &&
            magic[4] == 't' && magic[5] == 'e' && magic[6] == ' ' && magic[7] == 'f');
}

bool RosbagLoader::initializeReader() {
#ifdef USE_ROS2
    try {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path_;
        storage_options.storage_id = "sqlite3"; // Default for DB3 format
        
        // Try to detect format
        if (isMCAPFormat(bag_path_)) {
            storage_options.storage_id = "mcap";
        }
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
        reader_->open(storage_options, converter_options);
        
        return true;
    } catch (const std::exception& e) {
        last_error_ = "Exception initializing reader: " + std::string(e.what());
        return false;
    }
#else
    last_error_ = "ROS2 support not available";
    return false;
#endif
}

std::vector<Point3D> RosbagLoader::convertPointCloud2ToPoints(const void* msg_data) {
#ifdef USE_ROS2
    try {
        const auto* msg = static_cast<const sensor_msgs::msg::PointCloud2*>(msg_data);
        std::vector<Point3D> points;
        points.reserve(msg->data.size() / (msg->point_step));
        
        for (size_t i = 0; i < msg->data.size(); i += msg->point_step) {
            Point3D point;
            
            // Extract x, y, z coordinates
            // This is a simplified version - in practice you'd need to handle different field layouts
            const float* data = reinterpret_cast<const float*>(&msg->data[i]);
            point.x = data[0];
            point.y = data[1];
            point.z = data[2];
            
            points.push_back(point);
        }
        
        return points;
    } catch (const std::exception& e) {
        last_error_ = "Exception converting point cloud: " + std::string(e.what());
        return {};
    }
#else
    return {};
#endif
}

bool RosbagLoader::seekToMessage(const std::string& topic_name, size_t frame_number) {
#ifdef USE_ROS2
    if (!reader_) {
        return false;
    }
    
    try {
        size_t current_frame = 0;
        
        while (reader_->has_next()) {
            auto message = reader_->read_next();
            if (message && message->topic_name == topic_name) {
                if (current_frame == frame_number) {
                    // Rewind to this message
                    reader_->seek(message->time_stamp);
                    return true;
                }
                current_frame++;
            }
        }
        
        last_error_ = "Frame " + std::to_string(frame_number) + " not found for topic " + topic_name;
        return false;
    } catch (const std::exception& e) {
        last_error_ = "Exception seeking to message: " + std::string(e.what());
        return false;
    }
#else
    return false;
#endif
}

bool RosbagLoader::loadMCAPPointCloud(const std::string& topic_name, size_t frame_number, std::vector<Point3D>& points) {
    (void)topic_name;
    (void)frame_number;
    (void)points;
    // TODO: Implement MCAP-specific loading
    last_error_ = "MCAP loading not yet implemented";
    return false;
}

bool RosbagLoader::loadDB3PointCloud(const std::string& topic_name, size_t frame_number, std::vector<Point3D>& points) {
    (void)topic_name;
    (void)frame_number;
    (void)points;
    // TODO: Implement DB3-specific loading
    last_error_ = "DB3 loading not yet implemented";
    return false;
}

} // namespace recursive_patchwork 