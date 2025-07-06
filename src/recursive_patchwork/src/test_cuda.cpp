#include "cuda_interface.hpp"
#include "lidar_fusion.hpp"
#include <iostream>
#include <random>
#include <chrono>

using namespace recursive_patchwork;

// Generate random test points
std::vector<Point3D> generateTestPoints(int num_points) {
    std::vector<Point3D> points;
    points.reserve(num_points);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
    
    for (int i = 0; i < num_points; ++i) {
        points.emplace_back(dist(gen), dist(gen), dist(gen));
    }
    
    return points;
}

// Time a function execution
template<typename Func>
double timeFunction(Func func, const std::string& name) {
    auto start = std::chrono::high_resolution_clock::now();
    auto result = func();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double>(end - start).count();
    std::cout << name << " took " << duration * 1000.0 << " ms" << std::endl;
    
    return duration;
}

// Compare two point clouds
bool comparePointClouds(const std::vector<Point3D>& a, const std::vector<Point3D>& b, float tolerance = 1e-6f) {
    if (a.size() != b.size()) {
        std::cout << "Size mismatch: " << a.size() << " vs " << b.size() << std::endl;
        return false;
    }
    
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i].x - b[i].x) > tolerance ||
            std::abs(a[i].y - b[i].y) > tolerance ||
            std::abs(a[i].z - b[i].z) > tolerance) {
            std::cout << "Point " << i << " mismatch: (" 
                      << a[i].x << ", " << a[i].y << ", " << a[i].z << ") vs ("
                      << b[i].x << ", " << b[i].y << ", " << b[i].z << ")" << std::endl;
            return false;
        }
    }
    return true;
}

int main() {
    std::cout << "=== CUDA Point Cloud Operations Test ===" << std::endl;
    
    // Check CUDA availability
    bool cuda_available = cuda::CudaManager::isAvailable();
    std::cout << "CUDA available: " << (cuda_available ? "YES" : "NO") << std::endl;
    
    // Generate test data
    const int num_points = 100000;
    std::cout << "Generating " << num_points << " test points..." << std::endl;
    auto test_points = generateTestPoints(num_points);
    
    // Test 1: Rotation
    std::cout << "\n--- Testing 2D Rotation ---" << std::endl;
    float rotation_angle = 45.0f;
    
    // CPU version
    auto cpu_rotated = timeFunction([&]() {
        return LidarFusion::applyRotation2D(test_points, rotation_angle);
    }, "CPU rotation");
    
    // GPU version (or CPU fallback)
    auto gpu_rotated = timeFunction([&]() {
        return cuda::ops::applyRotation2D(test_points, rotation_angle);
    }, "GPU rotation");
    
    // Compare results
    bool rotation_match = comparePointClouds(cpu_rotated, gpu_rotated);
    std::cout << "Rotation results match: " << (rotation_match ? "YES" : "NO") << std::endl;
    
    // Test 2: Ego vehicle removal
    std::cout << "\n--- Testing Ego Vehicle Removal ---" << std::endl;
    float ego_radius = 2.5f;
    
    // CPU version
    auto cpu_filtered = timeFunction([&]() {
        return LidarFusion::removeEgoVehicle(test_points, ego_radius);
    }, "CPU filtering");
    
    // GPU version (or CPU fallback)
    auto gpu_filtered = timeFunction([&]() {
        return cuda::ops::removeEgoVehicle(test_points, ego_radius);
    }, "GPU filtering");
    
    // Compare results
    bool filtering_match = comparePointClouds(cpu_filtered, gpu_filtered);
    std::cout << "Filtering results match: " << (filtering_match ? "YES" : "NO") << std::endl;
    
    // Test 3: 4x4 Transformation
    std::cout << "\n--- Testing 4x4 Transformation ---" << std::endl;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = 1.0f;  // Translate in X
    transform(1, 3) = 2.0f;  // Translate in Y
    transform(2, 3) = 0.5f;  // Translate in Z
    
    // CPU version
    auto cpu_transformed = timeFunction([&]() {
        return LidarFusion::applyTransform(test_points, transform);
    }, "CPU transformation");
    
    // GPU version (or CPU fallback)
    auto gpu_transformed = timeFunction([&]() {
        return cuda::ops::applyTransform(test_points, transform);
    }, "GPU transformation");
    
    // Compare results
    bool transform_match = comparePointClouds(cpu_transformed, gpu_transformed);
    std::cout << "Transformation results match: " << (transform_match ? "YES" : "NO") << std::endl;
    
    // Summary
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "CUDA available: " << (cuda_available ? "YES" : "NO") << std::endl;
    std::cout << "All tests passed: " << (rotation_match && filtering_match && transform_match ? "YES" : "NO") << std::endl;
    
    if (cuda_available) {
        std::cout << "GPU acceleration is working!" << std::endl;
    } else {
        std::cout << "Running in CPU-only mode (CUDA not available)" << std::endl;
    }
    
    return 0;
} 