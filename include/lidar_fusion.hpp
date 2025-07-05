#pragma once

#include "recursive_patchwork.hpp"
#include <vector>
#include <string>
#include <memory>

namespace recursive_patchwork {

class LidarFusion {
public:
    LidarFusion();
    ~LidarFusion();

    // Configuration
    void addLidar(const LidarConfig& config);
    void setDefaultLidarConfigs();
    void clearLidars();

    // Main fusion function
    std::vector<Point3D> fuseLidarPointClouds(
        const std::vector<std::vector<Point3D>>& lidar_point_clouds);

    // Individual LiDAR processing
    std::vector<Point3D> processSingleLidar(
        const std::vector<Point3D>& points, 
        const LidarConfig& config);

    // Transform utilities
    static std::vector<Point3D> applyRotation2D(
        const std::vector<Point3D>& points, 
        float angle_degrees);
    
    static std::vector<Point3D> applyTransform(
        const std::vector<Point3D>& points,
        const Eigen::Matrix4f& transform_matrix);

    // Ego vehicle removal
    static std::vector<Point3D> removeEgoVehicle(
        const std::vector<Point3D>& points, 
        float radius = 2.5f);

    // Getter
    const std::vector<LidarConfig>& getLidarConfigs() const { return lidar_configs_; }

private:
    std::vector<LidarConfig> lidar_configs_;
    
    // Helper functions
    static Eigen::Matrix4f createRotationMatrix2D(float angle_degrees);
    static Eigen::Matrix4f createTranslationMatrix(float x, float y, float z);
    static bool isPointInEgoRadius(const Point3D& point, float radius);
};

} // namespace recursive_patchwork 