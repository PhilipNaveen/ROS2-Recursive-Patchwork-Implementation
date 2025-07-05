#include "recursive_patchwork.hpp"
#include "point_cloud_processor.hpp"
#include "lidar_fusion.hpp"
#include "visualization.hpp"
#include <iostream>
#include <random>
#include <cassert>

using namespace recursive_patchwork;

// Generate synthetic point cloud data
std::vector<Point3D> generateSyntheticPointCloud(size_t num_points = 10000) {
    std::vector<Point3D> points;
    points.reserve(num_points);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Ground points (flat surface with some noise)
    std::normal_distribution<float> ground_z(0.0f, 0.05f);
    std::uniform_real_distribution<float> ground_xy(-50.0f, 50.0f);
    
    // Obstacle points (vertical structures)
    std::uniform_real_distribution<float> obstacle_xy(-30.0f, 30.0f);
    std::uniform_real_distribution<float> obstacle_z(0.5f, 3.0f);
    
    // Generate 70% ground points, 30% obstacle points
    size_t ground_count = static_cast<size_t>(num_points * 0.7);
    size_t obstacle_count = num_points - ground_count;
    
    // Ground points
    for (size_t i = 0; i < ground_count; ++i) {
        Point3D point;
        point.x = ground_xy(gen);
        point.y = ground_xy(gen);
        point.z = ground_z(gen);
        points.push_back(point);
    }
    
    // Obstacle points (cars, pedestrians, etc.)
    for (size_t i = 0; i < obstacle_count; ++i) {
        Point3D point;
        point.x = obstacle_xy(gen);
        point.y = obstacle_xy(gen);
        point.z = obstacle_z(gen);
        points.push_back(point);
    }
    
    return points;
}

void testBasicFunctionality() {
    std::cout << "=== Testing Basic Functionality ===" << std::endl;
    
    // Generate synthetic data
    auto points = generateSyntheticPointCloud(5000);
    std::cout << "Generated " << points.size() << " synthetic points" << std::endl;
    
    // Initialize Recursive Patchwork
    PatchworkConfig config;
    config.sensor_height = 1.2f;
    config.filtering_radius = 50.0f;
    config.num_sectors = 8;
    config.max_iter = 50;
    
    RecursivePatchwork patchwork(config);
    
    // Test ground segmentation
    auto [ground_points, non_ground_points] = patchwork.filterGroundPoints(points);
    
    std::cout << "Ground points: " << ground_points.size() << std::endl;
    std::cout << "Non-ground points: " << non_ground_points.size() << std::endl;
    
    // Basic assertions
    assert(ground_points.size() + non_ground_points.size() <= points.size());
    assert(ground_points.size() > 0);
    assert(non_ground_points.size() > 0);
    
    std::cout << "✓ Basic functionality test passed" << std::endl;
}

void testEnhancedFiltering() {
    std::cout << "\n=== Testing Enhanced Filtering ===" << std::endl;
    
    auto points = generateSyntheticPointCloud(3000);
    
    RecursivePatchwork patchwork;
    
    // Test enhanced filtering
    auto filtered_points = patchwork.sampleGroundAndObstacles(points, 1.1f, 0.5f);
    
    std::cout << "Original points: " << points.size() << std::endl;
    std::cout << "Filtered points: " << filtered_points.size() << std::endl;
    
    assert(filtered_points.size() <= points.size());
    assert(filtered_points.size() > 0);
    
    std::cout << "✓ Enhanced filtering test passed" << std::endl;
}

void testPointCloudProcessor() {
    std::cout << "\n=== Testing Point Cloud Processor ===" << std::endl;
    
    auto points = generateSyntheticPointCloud(1000);
    
    // Test point cleaning
    auto cleaned_points = PointCloudProcessor::removeNaNPoints(points);
    assert(cleaned_points.size() == points.size());
    
    // Test centroid computation
    auto centroid = PointCloudProcessor::computeCentroid(points);
    assert(std::isfinite(centroid(0)) && std::isfinite(centroid(1)) && std::isfinite(centroid(2)));
    
    // Test PCA
    auto [pca_centroid, pca_eigenvectors] = PointCloudProcessor::computePCA(points);
    assert(std::isfinite(pca_centroid(0)) && std::isfinite(pca_centroid(1)) && std::isfinite(pca_centroid(2)));
    
    std::cout << "✓ Point cloud processor test passed" << std::endl;
}

void testLidarFusion() {
    std::cout << "\n=== Testing LiDAR Fusion ===" << std::endl;
    
    auto points1 = generateSyntheticPointCloud(1000);
    auto points2 = generateSyntheticPointCloud(1000);
    
    LidarFusion fusion;
    
    // Test rotation
    auto rotated_points = LidarFusion::applyRotation2D(points1, 45.0f);
    assert(rotated_points.size() == points1.size());
    
    // Test ego vehicle removal
    auto filtered_points = LidarFusion::removeEgoVehicle(points1, 2.5f);
    assert(filtered_points.size() <= points1.size());
    
    // Test fusion
    std::vector<std::vector<Point3D>> lidar_clouds = {points1, points2};
    auto fused_points = fusion.fuseLidarPointClouds(lidar_clouds);
    assert(fused_points.size() > 0);
    
    std::cout << "✓ LiDAR fusion test passed" << std::endl;
}

void testPerformance() {
    std::cout << "\n=== Testing Performance ===" << std::endl;
    
    auto points = generateSyntheticPointCloud(10000);
    
    RecursivePatchwork patchwork;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto [ground_points, non_ground_points] = patchwork.filterGroundPoints(points);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Processed " << points.size() << " points in " << duration.count() << " ms" << std::endl;
    std::cout << "Processing rate: " << (points.size() / (duration.count() / 1000.0)) << " points/second" << std::endl;
    
    std::cout << "✓ Performance test completed" << std::endl;
}

int main() {
    std::cout << "Recursive Patchwork C++ Test Suite" << std::endl;
    std::cout << "==================================" << std::endl;
    
    try {
        testBasicFunctionality();
        testEnhancedFiltering();
        testPointCloudProcessor();
        testLidarFusion();
        testPerformance();
        
        std::cout << "\n🎉 All tests passed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
} 