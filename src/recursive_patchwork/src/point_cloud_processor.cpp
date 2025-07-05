#include "point_cloud_processor.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <iostream>

namespace recursive_patchwork {

PointCloudProcessor::PointCloudProcessor() {
}

PointCloudProcessor::~PointCloudProcessor() {
}

std::vector<Point3D> PointCloudProcessor::removeNaNPoints(const std::vector<Point3D>& points) {
    std::vector<Point3D> valid_points;
    valid_points.reserve(points.size());
    
    for (const auto& point : points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            valid_points.push_back(point);
        }
    }
    
    return valid_points;
}

std::vector<Point3D> PointCloudProcessor::filterByDistance(const std::vector<Point3D>& points, 
                                                          float min_dist, float max_dist) {
    std::vector<Point3D> filtered_points;
    filtered_points.reserve(points.size());
    
    for (const auto& point : points) {
        float dist = std::sqrt(point.x * point.x + point.y * point.y);
        if (dist >= min_dist && dist <= max_dist) {
            filtered_points.push_back(point);
        }
    }
    
    return filtered_points;
}

std::vector<Point3D> PointCloudProcessor::filterByHeight(const std::vector<Point3D>& points, 
                                                        float min_height, float max_height) {
    std::vector<Point3D> filtered_points;
    filtered_points.reserve(points.size());
    
    for (const auto& point : points) {
        if (point.z >= min_height && point.z <= max_height) {
            filtered_points.push_back(point);
        }
    }
    
    return filtered_points;
}

Eigen::Vector3f PointCloudProcessor::computeCentroid(const std::vector<Point3D>& points) {
    if (points.empty()) {
        return Eigen::Vector3f::Zero();
    }
    
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& point : points) {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= points.size();
    
    return centroid;
}

Eigen::Matrix3f PointCloudProcessor::computeCovariance(const std::vector<Point3D>& points, 
                                                      const Eigen::Vector3f& centroid) {
    if (points.size() < 2) {
        return Eigen::Matrix3f::Zero();
    }
    
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& point : points) {
        Eigen::Vector3f diff(point.x - centroid(0), point.y - centroid(1), point.z - centroid(2));
        cov += diff * diff.transpose();
    }
    cov /= (points.size() - 1);
    
    return cov;
}

std::pair<Eigen::Vector3f, Eigen::Matrix3f> PointCloudProcessor::computePCA(const std::vector<Point3D>& points) {
    if (points.size() < 3) {
        return {Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity()};
    }
    
    Eigen::Vector3f centroid = computeCentroid(points);
    Eigen::Matrix3f cov = computeCovariance(points, centroid);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();
    
    return {centroid, eigenvectors};
}

float PointCloudProcessor::computePointToPlaneDistance(const Point3D& point, 
                                                     const Eigen::Vector3f& plane_point, 
                                                     const Eigen::Vector3f& plane_normal) {
    Eigen::Vector3f p_vec(point.x, point.y, point.z);
    return std::abs((p_vec - plane_point).dot(plane_normal));
}

std::vector<float> PointCloudProcessor::computeDistancesToPlane(const std::vector<Point3D>& points,
                                                               const Eigen::Vector3f& plane_point,
                                                               const Eigen::Vector3f& plane_normal) {
    std::vector<float> distances;
    distances.reserve(points.size());
    
    for (const auto& point : points) {
        distances.push_back(computePointToPlaneDistance(point, plane_point, plane_normal));
    }
    
    return distances;
}

std::vector<Point3D> PointCloudProcessor::randomSubsample(const std::vector<Point3D>& points, 
                                                         size_t target_size) {
    if (points.size() <= target_size) {
        return points;
    }
    
    std::vector<Point3D> subsampled;
    subsampled.reserve(target_size);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, points.size() - 1);
    
    std::vector<bool> selected(points.size(), false);
    size_t selected_count = 0;
    
    while (selected_count < target_size) {
        size_t idx = dis(gen);
        if (!selected[idx]) {
            selected[idx] = true;
            subsampled.push_back(points[idx]);
            selected_count++;
        }
    }
    
    return subsampled;
}

std::vector<Point3D> PointCloudProcessor::voxelGridFilter(const std::vector<Point3D>& points, 
                                                         float voxel_size) {
    if (points.empty() || voxel_size <= 0) {
        return points;
    }
    
    std::unordered_map<std::string, std::vector<size_t>> voxel_map;
    
    // Group points by voxel
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        int voxel_x = static_cast<int>(std::floor(point.x / voxel_size));
        int voxel_y = static_cast<int>(std::floor(point.y / voxel_size));
        int voxel_z = static_cast<int>(std::floor(point.z / voxel_size));
        
        std::string voxel_key = std::to_string(voxel_x) + "," + 
                               std::to_string(voxel_y) + "," + 
                               std::to_string(voxel_z);
        
        voxel_map[voxel_key].push_back(i);
    }
    
    // Select representative point from each voxel (centroid)
    std::vector<Point3D> filtered_points;
    filtered_points.reserve(voxel_map.size());
    
    for (const auto& [voxel_key, indices] : voxel_map) {
        if (indices.empty()) continue;
        
        // Compute centroid of points in this voxel
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (size_t idx : indices) {
            sum_x += points[idx].x;
            sum_y += points[idx].y;
            sum_z += points[idx].z;
        }
        
        Point3D centroid;
        centroid.x = sum_x / indices.size();
        centroid.y = sum_y / indices.size();
        centroid.z = sum_z / indices.size();
        
        filtered_points.push_back(centroid);
    }
    
    return filtered_points;
}

Eigen::MatrixXf PointCloudProcessor::pointsToEigenMatrix(const std::vector<Point3D>& points) {
    if (points.empty()) {
        return Eigen::MatrixXf(0, 3);
    }
    
    Eigen::MatrixXf matrix(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        matrix(i, 0) = points[i].x;
        matrix(i, 1) = points[i].y;
        matrix(i, 2) = points[i].z;
    }
    
    return matrix;
}

std::vector<Point3D> PointCloudProcessor::eigenMatrixToPoints(const Eigen::MatrixXf& matrix) {
    std::vector<Point3D> points;
    points.reserve(matrix.rows());
    
    for (int i = 0; i < matrix.rows(); ++i) {
        Point3D point;
        point.x = matrix(i, 0);
        point.y = matrix(i, 1);
        point.z = matrix(i, 2);
        points.push_back(point);
    }
    
    return points;
}

bool PointCloudProcessor::isValidPoint(const Point3D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool PointCloudProcessor::hasValidPoints(const std::vector<Point3D>& points) {
    for (const auto& point : points) {
        if (!isValidPoint(point)) {
            return false;
        }
    }
    return true;
}

} // namespace recursive_patchwork 