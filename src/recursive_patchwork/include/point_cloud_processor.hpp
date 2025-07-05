#pragma once

#include "recursive_patchwork.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace recursive_patchwork {

class PointCloudProcessor {
public:
    PointCloudProcessor();
    ~PointCloudProcessor();

    // Point cloud operations
    static std::vector<Point3D> removeNaNPoints(const std::vector<Point3D>& points);
    static std::vector<Point3D> filterByDistance(const std::vector<Point3D>& points, 
                                                float min_dist, float max_dist);
    static std::vector<Point3D> filterByHeight(const std::vector<Point3D>& points, 
                                              float min_height, float max_height);
    
    // Statistical operations
    static Eigen::Vector3f computeCentroid(const std::vector<Point3D>& points);
    static Eigen::Matrix3f computeCovariance(const std::vector<Point3D>& points, 
                                            const Eigen::Vector3f& centroid);
    static std::pair<Eigen::Vector3f, Eigen::Matrix3f> computePCA(const std::vector<Point3D>& points);
    
    // Geometric operations
    static float computePointToPlaneDistance(const Point3D& point, 
                                           const Eigen::Vector3f& plane_point, 
                                           const Eigen::Vector3f& plane_normal);
    static std::vector<float> computeDistancesToPlane(const std::vector<Point3D>& points,
                                                     const Eigen::Vector3f& plane_point,
                                                     const Eigen::Vector3f& plane_normal);
    
    // Subsampling
    static std::vector<Point3D> randomSubsample(const std::vector<Point3D>& points, 
                                               size_t target_size);
    static std::vector<Point3D> voxelGridFilter(const std::vector<Point3D>& points, 
                                               float voxel_size);
    
    // Conversion utilities
    static Eigen::MatrixXf pointsToEigenMatrix(const std::vector<Point3D>& points);
    static std::vector<Point3D> eigenMatrixToPoints(const Eigen::MatrixXf& matrix);
    
    // Validation
    static bool isValidPoint(const Point3D& point);
    static bool hasValidPoints(const std::vector<Point3D>& points);
};

} // namespace recursive_patchwork 