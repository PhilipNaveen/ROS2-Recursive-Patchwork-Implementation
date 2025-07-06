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
        std::cerr << "CUDA initialization failed: " << cudaGetErrorString(error) << std::endl;
        available_ = false;
    } else {
        available_ = true;
        std::cout << "CUDA initialized successfully" << std::endl;
    }
    
    initialized_ = true;
    return available_;
#else
    available_ = false;
    initialized_ = true;
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

// Wrapper functions that automatically choose GPU or CPU implementation
namespace ops {

std::vector<Point3D> applyRotation2D(
    const std::vector<Point3D>& points, float angle_degrees) {
    
    if (CudaManager::isAvailable()) {
        return CudaManager::applyRotation2D(points, angle_degrees);
    } else {
        return LidarFusion::applyRotation2D(points, angle_degrees);
    }
}

std::vector<Point3D> applyTransform(
    const std::vector<Point3D>& points,
    const Eigen::Matrix4f& transform_matrix) {
    
    if (CudaManager::isAvailable()) {
        return CudaManager::applyTransform(points, transform_matrix);
    } else {
        return LidarFusion::applyTransform(points, transform_matrix);
    }
}

std::vector<Point3D> removeEgoVehicle(
    const std::vector<Point3D>& points, float radius) {
    
    if (CudaManager::isAvailable()) {
        return CudaManager::removeEgoVehicle(points, radius);
    } else {
        return LidarFusion::removeEgoVehicle(points, radius);
    }
}

} // namespace ops

} // namespace cuda
} // namespace recursive_patchwork 