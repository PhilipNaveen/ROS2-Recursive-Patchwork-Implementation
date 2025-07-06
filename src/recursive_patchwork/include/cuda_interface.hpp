#pragma once

#include "recursive_patchwork.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace recursive_patchwork {
namespace cuda {

// CUDA utilities and GPU-accelerated functions
class CudaManager {
public:
    // Check if CUDA is available
    static bool isAvailable();
    
    // Initialize CUDA context
    static bool initialize();
    
    // Cleanup CUDA resources
    static void cleanup();

    // GPU-accelerated point cloud operations
    static std::vector<Point3D> applyRotation2D(
        const std::vector<Point3D>& points, 
        float angle_degrees);
    
    static std::vector<Point3D> applyTransform(
        const std::vector<Point3D>& points,
        const Eigen::Matrix4f& transform_matrix);
    
    static std::vector<Point3D> removeEgoVehicle(
        const std::vector<Point3D>& points, 
        float radius = 2.5f);

    // Phase 1: GPU-accelerated patchwork operations
    static std::vector<float> computeDistances2D(
        const std::vector<Point3D>& points);
    
    static std::vector<Point3D> filterPointsByRadius(
        const std::vector<Point3D>& points,
        const std::vector<float>& distances,
        float radius);
    
    static std::vector<float> computeAngles(
        const std::vector<Point3D>& points);
    
    static std::vector<float> computePlaneDistances(
        const std::vector<Point3D>& points,
        const Eigen::Vector3f& centroid,
        const Eigen::Vector3f& normal);

private:
    static bool initialized_;
    static bool available_;
};

// Wrapper functions that automatically choose GPU or CPU implementation
namespace ops {
    std::vector<Point3D> applyRotation2D(
        const std::vector<Point3D>& points, 
        float angle_degrees);
    
    std::vector<Point3D> applyTransform(
        const std::vector<Point3D>& points,
        const Eigen::Matrix4f& transform_matrix);
    
    std::vector<Point3D> removeEgoVehicle(
        const std::vector<Point3D>& points, 
        float radius = 2.5f);

    // Phase 1: GPU-accelerated patchwork operations
    std::vector<float> computeDistances2D(
        const std::vector<Point3D>& points);
    
    std::vector<Point3D> filterPointsByRadius(
        const std::vector<Point3D>& points,
        const std::vector<float>& distances,
        float radius);
    
    std::vector<float> computeAngles(
        const std::vector<Point3D>& points);
    
    std::vector<float> computePlaneDistances(
        const std::vector<Point3D>& points,
        const Eigen::Vector3f& centroid,
        const Eigen::Vector3f& normal);
}

} // namespace cuda
} // namespace recursive_patchwork 