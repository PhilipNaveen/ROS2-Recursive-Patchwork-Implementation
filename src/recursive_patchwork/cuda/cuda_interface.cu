#include "cuda_interface.hpp"
#include "lidar_fusion.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

#ifdef USE_CUDA
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/remove.h>
#include <thrust/copy.h>
#include <thrust/functional.h>
#include <thrust/tuple.h>

// Forward declarations of CUDA wrapper functions
void cuda_rotate_points(float* d_x, float* d_y, float* d_z, 
                       float cos_a, float sin_a, int n);
void cuda_transform_points(float* d_x, float* d_y, float* d_z,
                          float* d_matrix, int n);

// Phase 1 forward declarations
void cuda_compute_distances_2d(float* d_x, float* d_y, float* d_distances, int n);
void cuda_filter_points_by_radius(float* d_x, float* d_y, float* d_z, float* d_distances,
                                 bool* d_mask, float radius, int n);
void cuda_compute_angles(float* d_x, float* d_y, float* d_angles, int n);
void cuda_classify_points_in_patch(float* d_distances, float* d_angles,
                                  bool* d_mask, int n,
                                  float r0, float r1, float a0, float a1);
void cuda_compute_plane_distances(float* d_x, float* d_y, float* d_z,
                                 float* d_distances, int n,
                                 float centroid_x, float centroid_y, float centroid_z,
                                 float normal_x, float normal_y, float normal_z);
void cuda_classify_ground_points(float* d_plane_distances, bool* d_ground_mask,
                                float threshold, int n);

#endif // USE_CUDA

namespace recursive_patchwork {
namespace cuda {

// Static member initialization
bool CudaManager::initialized_ = false;
bool CudaManager::available_ = false;

bool CudaManager::isAvailable() {
#ifdef USE_CUDA
    if (!initialized_) {
        initialize();
    }
    return available_;
#else
    return false;
#endif
}

bool CudaManager::initialize() {
#ifdef USE_CUDA
    if (initialized_) {
        return available_;
    }
    
    cudaError_t error = cudaSetDevice(0);
    if (error != cudaSuccess) {
        std::cerr << "❌ CUDA initialization failed: " << cudaGetErrorString(error) << std::endl;
        available_ = false;
    } else {
        available_ = true;
        std::cout << "🚀 CUDA acceleration initialized successfully" << std::endl;
    }
    
    initialized_ = true;
    return available_;
#else
    available_ = false;
    initialized_ = true;
    std::cout << "ℹ️  CUDA acceleration not available (USE_CUDA not defined)" << std::endl;
    return false;
#endif
}

void CudaManager::cleanup() {
#ifdef USE_CUDA
    if (initialized_ && available_) {
        cudaDeviceReset();
        available_ = false;
        initialized_ = false;
    }
#endif
}

std::vector<Point3D> CudaManager::applyRotation2D(
    const std::vector<Point3D>& points, float angle_degrees) {
    
#ifdef USE_CUDA
    if (!isAvailable() || points.empty()) {
        return LidarFusion::applyRotation2D(points, angle_degrees);
    }
    
    std::cout << "🔄 [CUDA] Applying 2D rotation to " << points.size() << " points..." << std::endl;
    
    int n = points.size();
    float angle_rad = angle_degrees * M_PI / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);
    
    // Allocate device memory
    float *d_x, *d_y, *d_z;
    cudaMalloc(&d_x, n * sizeof(float));
    cudaMalloc(&d_y, n * sizeof(float));
    cudaMalloc(&d_z, n * sizeof(float));
    
    // Copy data to device
    std::vector<float> x_vec, y_vec, z_vec;
    x_vec.reserve(n);
    y_vec.reserve(n);
    z_vec.reserve(n);
    
    for (const auto& point : points) {
        x_vec.push_back(point.x);
        y_vec.push_back(point.y);
        z_vec.push_back(point.z);
    }
    
    cudaMemcpy(d_x, x_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_z, z_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel using wrapper
    cuda_rotate_points(d_x, d_y, d_z, cos_a, sin_a, n);
    
    // Copy results back
    cudaMemcpy(x_vec.data(), d_x, n * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(y_vec.data(), d_y, n * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(z_vec.data(), d_z, n * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Free device memory
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_z);
    
    // Convert back to Point3D
    std::vector<Point3D> result;
    result.reserve(n);
    for (int i = 0; i < n; ++i) {
        result.emplace_back(x_vec[i], y_vec[i], z_vec[i]);
    }
    
    std::cout << "✅ [CUDA] 2D rotation completed successfully" << std::endl;
    return result;
#else
    return LidarFusion::applyRotation2D(points, angle_degrees);
#endif
}

std::vector<Point3D> CudaManager::applyTransform(
    const std::vector<Point3D>& points,
    const Eigen::Matrix4f& transform_matrix) {
    
#ifdef USE_CUDA
    if (!isAvailable() || points.empty()) {
        return LidarFusion::applyTransform(points, transform_matrix);
    }
    
    std::cout << "🔄 [CUDA] Applying 4x4 transformation to " << points.size() << " points..." << std::endl;
    
    int n = points.size();
    
    // Allocate device memory
    float *d_x, *d_y, *d_z, *d_matrix;
    cudaMalloc(&d_x, n * sizeof(float));
    cudaMalloc(&d_y, n * sizeof(float));
    cudaMalloc(&d_z, n * sizeof(float));
    cudaMalloc(&d_matrix, 16 * sizeof(float));
    
    // Copy data to device
    std::vector<float> x_vec, y_vec, z_vec;
    x_vec.reserve(n);
    y_vec.reserve(n);
    z_vec.reserve(n);
    
    for (const auto& point : points) {
        x_vec.push_back(point.x);
        y_vec.push_back(point.y);
        z_vec.push_back(point.z);
    }
    
    cudaMemcpy(d_x, x_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_z, z_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    
    // Copy transformation matrix
    std::vector<float> matrix_vec(16);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix_vec[i * 4 + j] = transform_matrix(i, j);
        }
    }
    cudaMemcpy(d_matrix, matrix_vec.data(), 16 * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel using wrapper
    cuda_transform_points(d_x, d_y, d_z, d_matrix, n);
    
    // Copy results back
    cudaMemcpy(x_vec.data(), d_x, n * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(y_vec.data(), d_y, n * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(z_vec.data(), d_z, n * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Free device memory
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_z);
    cudaFree(d_matrix);
    
    // Convert back to Point3D
    std::vector<Point3D> result;
    result.reserve(n);
    for (int i = 0; i < n; ++i) {
        result.emplace_back(x_vec[i], y_vec[i], z_vec[i]);
    }
    
    std::cout << "✅ [CUDA] 4x4 transformation completed successfully" << std::endl;
    return result;
#else
    return LidarFusion::applyTransform(points, transform_matrix);
#endif
}

std::vector<Point3D> CudaManager::removeEgoVehicle(
    const std::vector<Point3D>& points, float radius) {
    
#ifdef USE_CUDA
    if (!isAvailable() || points.empty()) {
        return LidarFusion::removeEgoVehicle(points, radius);
    }
    
    // For now, fall back to CPU implementation to avoid Thrust complexity
    // TODO: Implement proper GPU filtering when Thrust issues are resolved
    return LidarFusion::removeEgoVehicle(points, radius);
#else
    return LidarFusion::removeEgoVehicle(points, radius);
#endif
}

// Phase 1: GPU-accelerated distance calculations
std::vector<float> CudaManager::computeDistances2D(const std::vector<Point3D>& points) {
#ifdef USE_CUDA
    if (!isAvailable() || points.empty()) {
        // Fallback to CPU
        std::vector<float> distances;
        distances.reserve(points.size());
        for (const auto& point : points) {
            distances.push_back(std::sqrt(point.x * point.x + point.y * point.y));
        }
        return distances;
    }
    
    std::cout << "🔄 [CUDA] Computing 2D distances for " << points.size() << " points..." << std::endl;
    
    int n = points.size();
    
    // Allocate device memory
    float *d_x, *d_y, *d_distances;
    cudaMalloc(&d_x, n * sizeof(float));
    cudaMalloc(&d_y, n * sizeof(float));
    cudaMalloc(&d_distances, n * sizeof(float));
    
    // Copy data to device
    std::vector<float> x_vec, y_vec;
    x_vec.reserve(n);
    y_vec.reserve(n);
    
    for (const auto& point : points) {
        x_vec.push_back(point.x);
        y_vec.push_back(point.y);
    }
    
    cudaMemcpy(d_x, x_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel
    cuda_compute_distances_2d(d_x, d_y, d_distances, n);
    
    // Copy results back
    std::vector<float> distances(n);
    cudaMemcpy(distances.data(), d_distances, n * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Free device memory
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_distances);
    
    std::cout << "✅ [CUDA] 2D distance computation completed successfully" << std::endl;
    return distances;
#else
    // Fallback to CPU
    std::vector<float> distances;
    distances.reserve(points.size());
    for (const auto& point : points) {
        distances.push_back(std::sqrt(point.x * point.x + point.y * point.y));
    }
    return distances;
#endif
}

// Phase 1: GPU-accelerated point filtering by radius
std::vector<Point3D> CudaManager::filterPointsByRadius(const std::vector<Point3D>& points, 
                                                       const std::vector<float>& distances, 
                                                       float radius) {
#ifdef USE_CUDA
    if (!isAvailable() || points.empty()) {
        // Fallback to CPU
        std::vector<Point3D> filtered_points;
        filtered_points.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            if (distances[i] <= radius) {
                filtered_points.push_back(points[i]);
            }
        }
        return filtered_points;
    }
    
    std::cout << "🔄 [CUDA] Filtering " << points.size() << " points by radius " << radius << "..." << std::endl;
    
    int n = points.size();
    
    // Allocate device memory
    float *d_x, *d_y, *d_z, *d_distances;
    bool *d_mask;
    cudaMalloc(&d_x, n * sizeof(float));
    cudaMalloc(&d_y, n * sizeof(float));
    cudaMalloc(&d_z, n * sizeof(float));
    cudaMalloc(&d_distances, n * sizeof(float));
    cudaMalloc(&d_mask, n * sizeof(bool));
    
    // Copy data to device
    std::vector<float> x_vec, y_vec, z_vec;
    x_vec.reserve(n);
    y_vec.reserve(n);
    z_vec.reserve(n);
    
    for (const auto& point : points) {
        x_vec.push_back(point.x);
        y_vec.push_back(point.y);
        z_vec.push_back(point.z);
    }
    
    cudaMemcpy(d_x, x_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_z, z_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_distances, distances.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel
    cuda_filter_points_by_radius(d_x, d_y, d_z, d_distances, d_mask, radius, n);
    
    // Copy mask back
    std::vector<bool> mask(n);
    cudaMemcpy(mask.data(), d_mask, n * sizeof(bool), cudaMemcpyDeviceToHost);
    
    // Free device memory
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_z);
    cudaFree(d_distances);
    cudaFree(d_mask);
    
    // Apply mask to get filtered points
    std::vector<Point3D> filtered_points;
    filtered_points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (mask[i]) {
            filtered_points.push_back(points[i]);
        }
    }
    
    std::cout << "✅ [CUDA] Point filtering completed successfully. Filtered: " 
              << filtered_points.size() << "/" << points.size() << " points" << std::endl;
    return filtered_points;
#else
    // Fallback to CPU
    std::vector<Point3D> filtered_points;
    filtered_points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (distances[i] <= radius) {
            filtered_points.push_back(points[i]);
        }
    }
    return filtered_points;
#endif
}

// Phase 1: GPU-accelerated angle calculations
std::vector<float> CudaManager::computeAngles(const std::vector<Point3D>& points) {
#ifdef USE_CUDA
    if (!isAvailable() || points.empty()) {
        // Fallback to CPU
        std::vector<float> angles;
        angles.reserve(points.size());
        for (const auto& point : points) {
            float angle = std::atan2(point.y, point.x);
            if (angle < 0) angle += 2.0f * M_PI;
            angles.push_back(angle);
        }
        return angles;
    }
    
    std::cout << "🔄 [CUDA] Computing angles for " << points.size() << " points..." << std::endl;
    
    int n = points.size();
    
    // Allocate device memory
    float *d_x, *d_y, *d_angles;
    cudaMalloc(&d_x, n * sizeof(float));
    cudaMalloc(&d_y, n * sizeof(float));
    cudaMalloc(&d_angles, n * sizeof(float));
    
    // Copy data to device
    std::vector<float> x_vec, y_vec;
    x_vec.reserve(n);
    y_vec.reserve(n);
    
    for (const auto& point : points) {
        x_vec.push_back(point.x);
        y_vec.push_back(point.y);
    }
    
    cudaMemcpy(d_x, x_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel
    cuda_compute_angles(d_x, d_y, d_angles, n);
    
    // Copy results back
    std::vector<float> angles(n);
    cudaMemcpy(angles.data(), d_angles, n * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Free device memory
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_angles);
    
    std::cout << "✅ [CUDA] Angle computation completed successfully" << std::endl;
    return angles;
#else
    // Fallback to CPU
    std::vector<float> angles;
    angles.reserve(points.size());
    for (const auto& point : points) {
        float angle = std::atan2(point.y, point.x);
        if (angle < 0) angle += 2.0f * M_PI;
        angles.push_back(angle);
    }
    return angles;
#endif
}

// Phase 1: GPU-accelerated plane distance computations
std::vector<float> CudaManager::computePlaneDistances(const std::vector<Point3D>& points,
                                                      const Eigen::Vector3f& centroid,
                                                      const Eigen::Vector3f& normal) {
#ifdef USE_CUDA
    if (!isAvailable() || points.empty()) {
        // Fallback to CPU
        std::vector<float> distances;
        distances.reserve(points.size());
        for (const auto& point : points) {
            Eigen::Vector3f p_vec(point.x, point.y, point.z);
            float dist = std::abs((p_vec - centroid).dot(normal));
            distances.push_back(dist);
        }
        return distances;
    }
    
    std::cout << "🔄 [CUDA] Computing plane distances for " << points.size() << " points..." << std::endl;
    
    int n = points.size();
    
    // Allocate device memory
    float *d_x, *d_y, *d_z, *d_distances;
    cudaMalloc(&d_x, n * sizeof(float));
    cudaMalloc(&d_y, n * sizeof(float));
    cudaMalloc(&d_z, n * sizeof(float));
    cudaMalloc(&d_distances, n * sizeof(float));
    
    // Copy data to device
    std::vector<float> x_vec, y_vec, z_vec;
    x_vec.reserve(n);
    y_vec.reserve(n);
    z_vec.reserve(n);
    
    for (const auto& point : points) {
        x_vec.push_back(point.x);
        y_vec.push_back(point.y);
        z_vec.push_back(point.z);
    }
    
    cudaMemcpy(d_x, x_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_z, z_vec.data(), n * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel
    cuda_compute_plane_distances(d_x, d_y, d_z, d_distances, n,
                                centroid(0), centroid(1), centroid(2),
                                normal(0), normal(1), normal(2));
    
    // Copy results back
    std::vector<float> distances(n);
    cudaMemcpy(distances.data(), d_distances, n * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Free device memory
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_z);
    cudaFree(d_distances);
    
    std::cout << "✅ [CUDA] Plane distance computation completed successfully" << std::endl;
    return distances;
#else
    // Fallback to CPU
    std::vector<float> distances;
    distances.reserve(points.size());
    for (const auto& point : points) {
        Eigen::Vector3f p_vec(point.x, point.y, point.z);
        float dist = std::abs((p_vec - centroid).dot(normal));
        distances.push_back(dist);
    }
    return distances;
#endif
}

// Wrapper functions that automatically choose GPU or CPU implementation
namespace ops {

std::vector<Point3D> applyRotation2D(
    const std::vector<Point3D>& points, float angle_degrees) {
    
    if (CudaManager::isAvailable()) {
        return CudaManager::applyRotation2D(points, angle_degrees);
    } else {
        std::cout << "🔄 [CPU] Applying 2D rotation to " << points.size() << " points..." << std::endl;
        auto result = LidarFusion::applyRotation2D(points, angle_degrees);
        std::cout << "✅ [CPU] 2D rotation completed successfully" << std::endl;
        return result;
    }
}

std::vector<Point3D> applyTransform(
    const std::vector<Point3D>& points,
    const Eigen::Matrix4f& transform_matrix) {
    
    if (CudaManager::isAvailable()) {
        return CudaManager::applyTransform(points, transform_matrix);
    } else {
        std::cout << "🔄 [CPU] Applying 4x4 transformation to " << points.size() << " points..." << std::endl;
        auto result = LidarFusion::applyTransform(points, transform_matrix);
        std::cout << "✅ [CPU] 4x4 transformation completed successfully" << std::endl;
        return result;
    }
}

std::vector<Point3D> removeEgoVehicle(
    const std::vector<Point3D>& points, float radius) {
    
    if (CudaManager::isAvailable()) {
        std::cout << "🔄 [CPU] Removing ego vehicle from " << points.size() << " points (GPU fallback)..." << std::endl;
        auto result = LidarFusion::removeEgoVehicle(points, radius);
        std::cout << "✅ [CPU] Ego vehicle removal completed successfully" << std::endl;
        return result;
    } else {
        std::cout << "🔄 [CPU] Removing ego vehicle from " << points.size() << " points..." << std::endl;
        auto result = LidarFusion::removeEgoVehicle(points, radius);
        std::cout << "✅ [CPU] Ego vehicle removal completed successfully" << std::endl;
        return result;
    }
}

std::vector<float> computeDistances2D(const std::vector<Point3D>& points) {
    if (CudaManager::isAvailable()) {
        return CudaManager::computeDistances2D(points);
    } else {
        std::cout << "🔄 [CPU] Computing 2D distances for " << points.size() << " points..." << std::endl;
        std::vector<float> distances;
        distances.reserve(points.size());
        for (const auto& point : points) {
            distances.push_back(std::sqrt(point.x * point.x + point.y * point.y));
        }
        std::cout << "✅ [CPU] 2D distance computation completed successfully" << std::endl;
        return distances;
    }
}

std::vector<Point3D> filterPointsByRadius(const std::vector<Point3D>& points,
                                               const std::vector<float>& distances,
                                               float radius) {
    if (CudaManager::isAvailable()) {
        return CudaManager::filterPointsByRadius(points, distances, radius);
    } else {
        std::cout << "🔄 [CPU] Filtering " << points.size() << " points by radius " << radius << "..." << std::endl;
        std::vector<Point3D> filtered_points;
        filtered_points.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            if (distances[i] <= radius) {
                filtered_points.push_back(points[i]);
            }
        }
        std::cout << "✅ [CPU] Point filtering completed successfully. Filtered: " 
                  << filtered_points.size() << "/" << points.size() << " points" << std::endl;
        return filtered_points;
    }
}

std::vector<float> computeAngles(const std::vector<Point3D>& points) {
    if (CudaManager::isAvailable()) {
        return CudaManager::computeAngles(points);
    } else {
        std::cout << "🔄 [CPU] Computing angles for " << points.size() << " points..." << std::endl;
        std::vector<float> angles;
        angles.reserve(points.size());
        for (const auto& point : points) {
            float angle = std::atan2(point.y, point.x);
            if (angle < 0) angle += 2.0f * M_PI;
            angles.push_back(angle);
        }
        std::cout << "✅ [CPU] Angle computation completed successfully" << std::endl;
        return angles;
    }
}

std::vector<float> computePlaneDistances(const std::vector<Point3D>& points,
                                              const Eigen::Vector3f& centroid,
                                              const Eigen::Vector3f& normal) {
    if (CudaManager::isAvailable()) {
        return CudaManager::computePlaneDistances(points, centroid, normal);
    } else {
        std::cout << "🔄 [CPU] Computing plane distances for " << points.size() << " points..." << std::endl;
        std::vector<float> distances;
        distances.reserve(points.size());
        for (const auto& point : points) {
            Eigen::Vector3f p_vec(point.x, point.y, point.z);
            float dist = std::abs((p_vec - centroid).dot(normal));
            distances.push_back(dist);
        }
        std::cout << "✅ [CPU] Plane distance computation completed successfully" << std::endl;
        return distances;
    }
}

} // namespace ops

} // namespace cuda
} // namespace recursive_patchwork 