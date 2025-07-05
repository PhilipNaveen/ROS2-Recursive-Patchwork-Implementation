#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

namespace recursive_patchwork {

// Forward declarations
class PointCloudProcessor;
class LidarFusion;
class RosbagLoader;
class Visualization;

// Point cloud data structure
struct Point3D {
    float x, y, z;
    Point3D() : x(0), y(0), z(0) {}
    Point3D(float x, float y, float z) : x(x), y(y), z(z) {}
};

// Configuration structure
struct PatchworkConfig {
    float sensor_height = 1.2f;
    float max_range = 150.0f;
    int num_sectors = 10;
    int max_iter = 100;
    bool adaptive_seed_height = true;
    float th_seeds = 0.15f;
    float th_dist = 0.2f;
    float th_outlier = 0.08f;
    float filtering_radius = 150.0f;
    int max_split_depth = 1000;
};

// LiDAR configuration
struct LidarConfig {
    int lidar_id;
    std::string topic_name;
    float rotation_angle;  // degrees
    float ego_radius = 2.5f;
};

// Main Recursive Patchwork class
class RecursivePatchwork {
public:
    RecursivePatchwork(const PatchworkConfig& config = PatchworkConfig{});
    ~RecursivePatchwork();

    // Main processing functions
    std::pair<std::vector<Point3D>, std::vector<Point3D>> 
    filterGroundPoints(const std::vector<Point3D>& points);
    
    std::vector<Point3D> 
    sampleGroundAndObstacles(const std::vector<Point3D>& points, 
                            float target_height = 1.1f, 
                            float base_tol = 0.5f);

    // Utility functions
    std::vector<Point3D> cleanPoints(const std::vector<Point3D>& points);
    std::vector<Point3D> rotatePoints2D(const std::vector<Point3D>& points, float angle_degrees);
    
    // Configuration
    void setConfig(const PatchworkConfig& config) { config_ = config; }
    const PatchworkConfig& getConfig() const { return config_; }

private:
    PatchworkConfig config_;
    
    // Core algorithm components
    struct PlaneFitResult {
        Eigen::Vector3f centroid;
        Eigen::Vector3f normal;
        float residual;
    };
    
    PlaneFitResult fitPlanePCA(const std::vector<Point3D>& points);
    std::vector<bool> fitPlaneAndSplit(const std::vector<Point3D>& patch_points, 
                                      float mean_dist, int depth = 0);
    
    // Helper functions
    float computeDistance2D(const Point3D& p);
    float computeDistance2D(float x, float y);
    std::vector<Point3D> removeEgoVehicle(const std::vector<Point3D>& points, float radius = 2.5f);
};

// Performance timing utility
class Timer {
public:
    Timer() : start_(std::chrono::high_resolution_clock::now()) {}
    
    double elapsed() const {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double>(end - start_).count();
    }
    
    void reset() {
        start_ = std::chrono::high_resolution_clock::now();
    }

private:
    std::chrono::high_resolution_clock::time_point start_;
};

} // namespace recursive_patchwork 