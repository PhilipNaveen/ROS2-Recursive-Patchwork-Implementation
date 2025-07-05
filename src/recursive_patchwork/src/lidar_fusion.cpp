#include "lidar_fusion.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace recursive_patchwork {

LidarFusion::LidarFusion() {
    setDefaultLidarConfigs();
}

LidarFusion::~LidarFusion() {
}

void LidarFusion::addLidar(const LidarConfig& config) {
    lidar_configs_.push_back(config);
}

void LidarFusion::setDefaultLidarConfigs() {
    lidar_configs_.clear();
    
    // Default configuration for 3-LiDAR setup
    // Front LiDAR (1): faces forward (0 degrees)
    LidarConfig front_lidar{1, "/lidar_front", 0.0f, 2.5f};
    
    // Left LiDAR (2): angled 60 degrees behind the car (120 degrees from forward)
    LidarConfig left_lidar{2, "/lidar_left", 120.0f, 2.5f};
    
    // Right LiDAR (3): angled 60 degrees behind the car (-120 degrees from forward)
    LidarConfig right_lidar{3, "/lidar_right", -120.0f, 2.5f};
    
    lidar_configs_.push_back(front_lidar);
    lidar_configs_.push_back(left_lidar);
    lidar_configs_.push_back(right_lidar);
}

void LidarFusion::clearLidars() {
    lidar_configs_.clear();
}

std::vector<Point3D> LidarFusion::fuseLidarPointClouds(
    const std::vector<std::vector<Point3D>>& lidar_point_clouds) {
    
    if (lidar_point_clouds.empty()) {
        return {};
    }
    
    if (lidar_point_clouds.size() != lidar_configs_.size()) {
        std::cerr << "Warning: Number of point clouds (" << lidar_point_clouds.size() 
                  << ") doesn't match number of LiDAR configs (" << lidar_configs_.size() << ")" << std::endl;
    }
    
    std::vector<std::vector<Point3D>> processed_clouds;
    processed_clouds.reserve(lidar_point_clouds.size());
    
    // Process each LiDAR point cloud
    for (size_t i = 0; i < std::min(lidar_point_clouds.size(), lidar_configs_.size()); ++i) {
        const auto& points = lidar_point_clouds[i];
        const auto& config = lidar_configs_[i];
        
        std::cout << "Processing LiDAR " << config.lidar_id << ": " << config.topic_name << std::endl;
        
        auto processed_points = processSingleLidar(points, config);
        processed_clouds.push_back(processed_points);
        
        std::cout << "LiDAR " << config.lidar_id << " - Final point count: " << processed_points.size() << std::endl;
    }
    
    // Combine all point clouds
    size_t total_points = 0;
    for (const auto& cloud : processed_clouds) {
        total_points += cloud.size();
    }
    
    std::vector<Point3D> fused_points;
    fused_points.reserve(total_points);
    
    for (const auto& cloud : processed_clouds) {
        fused_points.insert(fused_points.end(), cloud.begin(), cloud.end());
    }
    
    std::cout << "Fused point cloud - Total points: " << fused_points.size() << std::endl;
    
    return fused_points;
}

std::vector<Point3D> LidarFusion::processSingleLidar(
    const std::vector<Point3D>& points, const LidarConfig& config) {
    
    if (points.empty()) {
        std::cout << "Warning: No points loaded for LiDAR " << config.lidar_id << std::endl;
        return {};
    }
    
    // Apply rotation if needed
    std::vector<Point3D> rotated_points = points;
    if (std::abs(config.rotation_angle) > 1e-6f) {
        rotated_points = applyRotation2D(points, config.rotation_angle);
        std::cout << "LiDAR " << config.lidar_id << ": Applied " << config.rotation_angle << "° rotation" << std::endl;
    }
    
    // Remove ego vehicle
    auto filtered_points = removeEgoVehicle(rotated_points, config.ego_radius);
    
    return filtered_points;
}

std::vector<Point3D> LidarFusion::applyRotation2D(const std::vector<Point3D>& points, float angle_degrees) {
    float angle_rad = angle_degrees * M_PI / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);
    
    std::vector<Point3D> rotated_points;
    rotated_points.reserve(points.size());
    
    for (const auto& point : points) {
        Point3D rotated;
        rotated.x = point.x * cos_a - point.y * sin_a;
        rotated.y = point.x * sin_a + point.y * cos_a;
        rotated.z = point.z;  // Z remains unchanged
        rotated_points.push_back(rotated);
    }
    
    return rotated_points;
}

std::vector<Point3D> LidarFusion::applyTransform(const std::vector<Point3D>& points,
                                                const Eigen::Matrix4f& transform_matrix) {
    std::vector<Point3D> transformed_points;
    transformed_points.reserve(points.size());
    
    for (const auto& point : points) {
        Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f transformed = transform_matrix * homogeneous_point;
        
        Point3D transformed_point;
        transformed_point.x = transformed(0) / transformed(3);
        transformed_point.y = transformed(1) / transformed(3);
        transformed_point.z = transformed(2) / transformed(3);
        
        transformed_points.push_back(transformed_point);
    }
    
    return transformed_points;
}

std::vector<Point3D> LidarFusion::removeEgoVehicle(const std::vector<Point3D>& points, float radius) {
    std::vector<Point3D> filtered_points;
    filtered_points.reserve(points.size());
    
    for (const auto& point : points) {
        if (!isPointInEgoRadius(point, radius)) {
            filtered_points.push_back(point);
        }
    }
    
    return filtered_points;
}

Eigen::Matrix4f LidarFusion::createRotationMatrix2D(float angle_degrees) {
    float angle_rad = angle_degrees * M_PI / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);
    
    Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
    rotation_matrix(0, 0) = cos_a;
    rotation_matrix(0, 1) = -sin_a;
    rotation_matrix(1, 0) = sin_a;
    rotation_matrix(1, 1) = cos_a;
    
    return rotation_matrix;
}

Eigen::Matrix4f LidarFusion::createTranslationMatrix(float x, float y, float z) {
    Eigen::Matrix4f translation_matrix = Eigen::Matrix4f::Identity();
    translation_matrix(0, 3) = x;
    translation_matrix(1, 3) = y;
    translation_matrix(2, 3) = z;
    
    return translation_matrix;
}

bool LidarFusion::isPointInEgoRadius(const Point3D& point, float radius) {
    float distance_2d = std::sqrt(point.x * point.x + point.y * point.y);
    return distance_2d <= radius;
}

} // namespace recursive_patchwork 