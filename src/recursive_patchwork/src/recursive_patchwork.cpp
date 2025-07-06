#include "recursive_patchwork.hpp"
#include "point_cloud_processor.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <numeric>

namespace recursive_patchwork {

RecursivePatchwork::RecursivePatchwork(const PatchworkConfig& config) 
    : config_(config) {
} // O(1)

RecursivePatchwork::~RecursivePatchwork() {
} // O(1)

std::vector<Point3D> RecursivePatchwork::cleanPoints(const std::vector<Point3D>& points) {
    std::vector<Point3D> cleaned_points;
    cleaned_points.reserve(points.size()); // Allocation doesnt cost time
    
    for (const auto& point : points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            cleaned_points.push_back(point);
        }
    } // O(n)
    
    if (cleaned_points.size() < points.size()) {
        std::cout << "Removed " << (points.size() - cleaned_points.size()) 
                  << " points with NaN/inf values" << std::endl;
    } // O(1)
    
    return cleaned_points;
}

std::vector<Point3D> RecursivePatchwork::rotatePoints2D(const std::vector<Point3D>& points, float angle_degrees) {
    float angle_rad = angle_degrees * M_PI / 180.0f;
    float cos_a = std::cos(angle_rad); // O(nlogn*n^{\frac{1}{2}}) = O(n^{\frac{3}{2}}logn)
    float sin_a = std::sin(angle_rad); // same as above
    
    std::vector<Point3D> rotated_points;
    rotated_points.reserve(points.size()); // Memory allocation O(1)
    
    for (const auto& point : points) {
        Point3D rotated;
        rotated.x = point.x * cos_a - point.y * sin_a; // Harvey 2019 => O(nlogn)+1 [Addition=O(1)] BUT if small O(1)
        rotated.y = point.x * sin_a + point.y * cos_a; // Harvey 2019 => O(nlogn)+1
        rotated.z = point.z;  // Z remains unchanged
        rotated_points.push_back(rotated);
    } // This will be O(n*nlogn)=O(n^{2}logn)
    
    return rotated_points; // Final: O(n^{2}logn)+O(n^{\frac{3}{2}}logn) = O(n^{2}logn) But if we assume that it is small and not a bigint it would be O(n^{\frac{3}{2}}logn) 
}

float RecursivePatchwork::computeDistance2D(const Point3D& p) {
    return std::sqrt(p.x * p.x + p.y * p.y); // Harvey 2019 => O(nlogn) and sqrt uses the same complexity or if small O(1)
}

float RecursivePatchwork::computeDistance2D(float x, float y) {
    return std::sqrt(x * x + y * y); // Harvey 2019 => O(nlogn) and sqrt uses the same complexity  or if small O(1)
}

std::vector<Point3D> RecursivePatchwork::removeEgoVehicle(const std::vector<Point3D>& points, float radius) {
    std::vector<Point3D> filtered_points;
    filtered_points.reserve(points.size()); // Memory allocation does not cost time
    
    for (const auto& point : points) { // repeats n times
        if (computeDistance2D(point) > radius) { // computeDistance2D is O(nlogn) // if small assume O(1) ask
            filtered_points.push_back(point);
        }
    } // O(n^{2}logn) or O(n)
    
    return filtered_points;
}

RecursivePatchwork::PlaneFitResult RecursivePatchwork::fitPlanePCA(const std::vector<Point3D>& points) {
    if (points.size() < 3) {
        return {{0, 0, 0}, {0, 0, 1}, std::numeric_limits<float>::max()};
    }
    
    // Compute centroid
    Eigen::Vector3f centroid = PointCloudProcessor::computeCentroid(points); //O(n) (add all up divide by tot)
    
    // Compute covariance matrix
    Eigen::Matrix3f cov = PointCloudProcessor::computeCovariance(points, centroid); // O(n) <= check due to dealing with matrix
    
    // Eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov); //O(1)
    Eigen::Vector3f normal = solver.eigenvectors().col(0); // O(1)
    
    // Ensure normal points upward (positive Z)
    if (normal(2) < 0) { //O(1)
        normal = -normal;
    }
    
    // Compute residual
    float residual = 0.0f;
    for (const auto& point : points) { // O(n) Iterations
        Eigen::Vector3f p_vec(point.x, point.y, point.z); // O(1) Cost for claculating
        float dist = std::abs((p_vec - centroid).dot(normal)); // O(1)
        residual += dist;
    }
    residual /= points.size();
    
    return {centroid, normal, residual}; // Total O(3n+3)
}

std::vector<bool> RecursivePatchwork::fitPlaneAndSplit(const std::vector<Point3D>& patch_points, 
                                                      float mean_dist, int depth) {
    if (patch_points.size() < 3 || depth > config_.max_split_depth) {
        return std::vector<bool>(patch_points.size(), false);
    }
    
    // Enforce minimum spatial area to stop splitting
    float x_min = std::numeric_limits<float>::max(), x_max = -std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max(), y_max = -std::numeric_limits<float>::max();
    
    for (const auto& point : patch_points) {
        x_min = std::min(x_min, point.x);
        x_max = std::max(x_max, point.x);
        y_min = std::min(y_min, point.y);
        y_max = std::max(y_max, point.y);
    }
    
    float area = (x_max - x_min) * (y_max - y_min);
    if (area < 25.0f && depth > 0) {
        return std::vector<bool>(patch_points.size(), true);
    }
    
    // Check if points are already flat
    float z_min = std::numeric_limits<float>::max(), z_max = -std::numeric_limits<float>::max();
    for (const auto& point : patch_points) {
        z_min = std::min(z_min, point.z);
        z_max = std::max(z_max, point.z);
    }
    
    if ((z_max - z_min) < 0.05f && patch_points.size() > 10) {
        return std::vector<bool>(patch_points.size(), true);
    }
    
    // Extract Z values for seed selection
    std::vector<float> z_values;
    z_values.reserve(patch_points.size());
    for (const auto& point : patch_points) {
        z_values.push_back(point.z);
    }
    
    float rel_dist = mean_dist / config_.filtering_radius;
    float z_th;
    
    if (config_.adaptive_seed_height) {
        z_th = config_.sensor_height + 0.2f * rel_dist;
    } else {
        // Use 10th percentile
        std::vector<float> sorted_z = z_values;
        std::sort(sorted_z.begin(), sorted_z.end());
        size_t idx = static_cast<size_t>(0.1f * sorted_z.size());
        z_th = sorted_z[idx] + config_.th_seeds;
    }
    
    // Create seed mask
    std::vector<bool> seed_mask(patch_points.size(), false);
    for (size_t i = 0; i < patch_points.size(); ++i) {
        if (z_values[i] < z_th) {
            seed_mask[i] = true;
        }
    }
    
    // If not enough seeds, use lowest points
    size_t seed_count = std::count(seed_mask.begin(), seed_mask.end(), true);
    if (seed_count < 3) {
        std::vector<size_t> indices(patch_points.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::partial_sort(indices.begin(), indices.begin() + 3, indices.end(),
                         [&z_values](size_t a, size_t b) { return z_values[a] < z_values[b]; });
        
        seed_mask.assign(patch_points.size(), false);
        for (size_t i = 0; i < 3; ++i) {
            seed_mask[indices[i]] = true;
        }
    }
    
    // Iterative plane fitting
    std::vector<bool> ground_mask = seed_mask;
    for (int iter = 0; iter < config_.max_iter; ++iter) {
        // Extract ground points
        std::vector<Point3D> ground_points;
        ground_points.reserve(patch_points.size());
        for (size_t i = 0; i < patch_points.size(); ++i) {
            if (ground_mask[i]) {
                ground_points.push_back(patch_points[i]);
            }
        }
        
        if (ground_points.size() < 3) break;
        
        // Fit plane
        auto plane_result = fitPlanePCA(ground_points);
        
        // Compute distances to plane
        std::vector<bool> new_mask(patch_points.size(), false);
        float threshold = config_.th_dist * (1.0f + 0.2f * rel_dist);
        
        for (size_t i = 0; i < patch_points.size(); ++i) {
            const auto& point = patch_points[i];
            Eigen::Vector3f p_vec(point.x, point.y, point.z);
            float dist = std::abs((p_vec - plane_result.centroid).dot(plane_result.normal));
            if (dist < threshold) {
                new_mask[i] = true;
            }
        }
        
        // Check convergence
        if (new_mask == ground_mask) break;
        ground_mask = new_mask;
    }
    
    // Final plane fit for residual computation
    std::vector<Point3D> final_ground_points;
    final_ground_points.reserve(patch_points.size());
    for (size_t i = 0; i < patch_points.size(); ++i) {
        if (ground_mask[i]) {
            final_ground_points.push_back(patch_points[i]);
        }
    }
    
    auto final_plane = fitPlanePCA(final_ground_points);
    
    // Decide whether to split
    float split_threshold = config_.th_dist * (1.0f + 1.5f * depth);
    size_t min_patch_size = 50 + 10 * depth;
    
    if (final_plane.residual > split_threshold && depth < config_.max_split_depth && 
        patch_points.size() >= min_patch_size) {
        
        // Split along axis with higher variance
        float var_x = 0.0f, var_y = 0.0f;
        Eigen::Vector3f centroid = PointCloudProcessor::computeCentroid(patch_points);
        
        for (const auto& point : patch_points) {
            float dx = point.x - centroid(0);
            float dy = point.y - centroid(1);
            var_x += dx * dx;
            var_y += dy * dy;
        }
        var_x /= patch_points.size();
        var_y /= patch_points.size();
        
        int split_axis = (var_x > var_y) ? 0 : 1;
        float median_val;
        
        if (split_axis == 0) {
            std::vector<float> x_vals;
            x_vals.reserve(patch_points.size());
            for (const auto& point : patch_points) {
                x_vals.push_back(point.x);
            }
            std::sort(x_vals.begin(), x_vals.end());
            median_val = x_vals[x_vals.size() / 2];
        } else {
            std::vector<float> y_vals;
            y_vals.reserve(patch_points.size());
            for (const auto& point : patch_points) {
                y_vals.push_back(point.y);
            }
            std::sort(y_vals.begin(), y_vals.end());
            median_val = y_vals[y_vals.size() / 2];
        }
        
        // Split into left and right patches
        std::vector<Point3D> left_patch, right_patch;
        left_patch.reserve(patch_points.size() / 2);
        right_patch.reserve(patch_points.size() / 2);
        
        for (const auto& point : patch_points) {
            float val = (split_axis == 0) ? point.x : point.y;
            if (val <= median_val) {
                left_patch.push_back(point);
            } else {
                right_patch.push_back(point);
            }
        }
        
        // Recursive calls
        auto left_result = fitPlaneAndSplit(left_patch, mean_dist, depth + 1);
        auto right_result = fitPlaneAndSplit(right_patch, mean_dist, depth + 1);
        
        // Combine results
        std::vector<bool> result(patch_points.size(), false);
        size_t left_idx = 0, right_idx = 0;
        
        for (const auto& point : patch_points) {
            float val = (split_axis == 0) ? point.x : point.y;
            if (val <= median_val) {
                result[left_idx] = left_result[left_idx];
                left_idx++;
            } else {
                result[right_idx + left_patch.size()] = right_result[right_idx];
                right_idx++;
            }
        }
        
        return result;
    }
    
    return ground_mask;
}

std::pair<std::vector<Point3D>, std::vector<Point3D>> 
RecursivePatchwork::filterGroundPoints(const std::vector<Point3D>& points) {
    Timer timer;
    
    // Clean points
    auto cleaned_points = cleanPoints(points);
    if (cleaned_points.empty()) {
        return {{}, {}};
    }
    
    // Compute distances and filter by radius
    std::vector<float> distances;
    distances.reserve(cleaned_points.size());
    for (const auto& point : cleaned_points) {
        distances.push_back(computeDistance2D(point));
    }
    
    std::vector<Point3D> points_zone;
    std::vector<float> d_zone;
    points_zone.reserve(cleaned_points.size());
    d_zone.reserve(cleaned_points.size());
    
    for (size_t i = 0; i < cleaned_points.size(); ++i) {
        if (distances[i] <= config_.filtering_radius) {
            points_zone.push_back(cleaned_points[i]);
            d_zone.push_back(distances[i]);
        }
    }
    
    if (points_zone.size() < 3) {
        return {{}, cleaned_points};
    }
    
    // Create ring and sector structure
    float r_min = 1.0f, r_max = config_.filtering_radius;
    int num_rings = 8;
    std::vector<float> ring_edges(num_rings + 1);
    
    for (int i = 0; i <= num_rings; ++i) {
        ring_edges[i] = r_min * std::pow(r_max / r_min, static_cast<float>(i) / num_rings);
    }
    
    float sector_angle = 2.0f * M_PI / config_.num_sectors;
    
    // Compute angles
    std::vector<float> angles;
    angles.reserve(points_zone.size());
    for (const auto& point : points_zone) {
        float angle = std::atan2(point.y, point.x);
        if (angle < 0) angle += 2.0f * M_PI;
        angles.push_back(angle);
    }
    
    std::vector<bool> is_ground_zone(points_zone.size(), false);
    
    // Process each ring-sector patch
    for (int ring_idx = 0; ring_idx < num_rings; ++ring_idx) {
        float r0 = ring_edges[ring_idx], r1 = ring_edges[ring_idx + 1];
        
        for (int sector_idx = 0; sector_idx < config_.num_sectors; ++sector_idx) {
            float a0 = sector_idx * sector_angle, a1 = (sector_idx + 1) * sector_angle;
            
            // Find points in this patch
            std::vector<Point3D> patch_points;
            std::vector<size_t> patch_indices;
            patch_points.reserve(points_zone.size() / (num_rings * config_.num_sectors));
            patch_indices.reserve(points_zone.size() / (num_rings * config_.num_sectors));
            
            for (size_t i = 0; i < points_zone.size(); ++i) {
                if (d_zone[i] >= r0 && d_zone[i] < r1 && 
                    angles[i] >= a0 && angles[i] < a1) {
                    patch_points.push_back(points_zone[i]);
                    patch_indices.push_back(i);
                }
            }
            
            if (patch_points.empty()) continue;
            
            // Compute mean distance for this patch
            float mean_dist = 0.0f;
            for (size_t idx : patch_indices) {
                mean_dist += d_zone[idx];
            }
            mean_dist /= patch_indices.size();
            
            // Apply recursive plane fitting
            auto ground_mask = fitPlaneAndSplit(patch_points, mean_dist);
            
            // Mark ground points
            for (size_t i = 0; i < patch_indices.size(); ++i) {
                if (ground_mask[i]) {
                    is_ground_zone[patch_indices[i]] = true;
                }
            }
        }
    }
    
    // Separate ground and non-ground points
    std::vector<Point3D> ground_points, non_ground_points;
    ground_points.reserve(points_zone.size());
    non_ground_points.reserve(points_zone.size());
    
    for (size_t i = 0; i < points_zone.size(); ++i) {
        if (is_ground_zone[i]) {
            ground_points.push_back(points_zone[i]);
        } else {
            non_ground_points.push_back(points_zone[i]);
        }
    }
    
    // Add points outside filtering radius to non-ground
    for (size_t i = 0; i < cleaned_points.size(); ++i) {
        if (distances[i] > config_.filtering_radius) {
            non_ground_points.push_back(cleaned_points[i]);
        }
    }
    
    std::cout << "Ground filtering completed in " << timer.elapsed() << " seconds" << std::endl;
    std::cout << "Ground points: " << ground_points.size() 
              << ", Non-ground points: " << non_ground_points.size() << std::endl;
    
    return {ground_points, non_ground_points};
}

std::vector<Point3D> RecursivePatchwork::sampleGroundAndObstacles(
    const std::vector<Point3D>& points, float target_height, float base_tol) {
    
    // First, separate ground and non-ground points
    auto [ground_points, non_ground_points] = filterGroundPoints(points);
    
    if (non_ground_points.empty()) {
        return ground_points;
    }
    
    // Remove ego vehicle from non-ground points
    non_ground_points = removeEgoVehicle(non_ground_points, 2.5f);
    
    // Filter by target height
    std::vector<Point3D> obstacle_points;
    obstacle_points.reserve(non_ground_points.size());
    
    for (const auto& point : non_ground_points) {
        if (std::abs(point.z - target_height) <= base_tol) {
            obstacle_points.push_back(point);
        }
    }
    
    // Subsample ground points for context
    std::vector<Point3D> ground_sample;
    if (!ground_points.empty()) {
        size_t sample_size = std::min(static_cast<size_t>(2000), ground_points.size());
        ground_sample = PointCloudProcessor::randomSubsample(ground_points, sample_size);
    }
    
    // Combine ground sample and obstacle points
    std::vector<Point3D> result;
    result.reserve(ground_sample.size() + obstacle_points.size());
    result.insert(result.end(), ground_sample.begin(), ground_sample.end());
    result.insert(result.end(), obstacle_points.begin(), obstacle_points.end());
    
    return result;
}

} // namespace recursive_patchwork 