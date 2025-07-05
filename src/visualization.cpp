#include "visualization.hpp"
#include <algorithm>
#include <iostream>

namespace recursive_patchwork {

Visualization::Visualization() 
    : ground_color_(0, 255, 0)      // Green for ground
    , non_ground_color_(128, 128, 128)  // Gray for non-ground
    , filtered_color_(255, 0, 0)    // Red for filtered obstacles
    , background_color_(0, 0, 0)    // Black background
{
}

Visualization::~Visualization() {
}

cv::Mat Visualization::createBEVImage(const std::vector<Point3D>& points,
                                     int width, int height,
                                     float x_min, float y_min,
                                     float x_max, float y_max) {
    
    cv::Mat image(height, width, CV_8UC3, background_color_);
    
    if (points.empty()) {
        return image;
    }
    
    // Draw all points in gray
    drawPoints(image, points, non_ground_color_, 1.0f);
    
    return image;
}

cv::Mat Visualization::createGroundNonGroundImage(const std::vector<Point3D>& ground_points,
                                                 const std::vector<Point3D>& non_ground_points,
                                                 int width, int height,
                                                 float x_min, float y_min,
                                                 float x_max, float y_max) {
    
    cv::Mat image(height, width, CV_8UC3, background_color_);
    
    // Draw ground points in green
    if (!ground_points.empty()) {
        drawPoints(image, ground_points, ground_color_, 1.0f);
    }
    
    // Draw non-ground points in gray
    if (!non_ground_points.empty()) {
        drawPoints(image, non_ground_points, non_ground_color_, 1.0f);
    }
    
    return image;
}

cv::Mat Visualization::createEnhancedFilteredImage(const std::vector<Point3D>& filtered_points,
                                                  int width, int height,
                                                  float x_min, float y_min,
                                                  float x_max, float y_max) {
    
    cv::Mat image(height, width, CV_8UC3, background_color_);
    
    if (!filtered_points.empty()) {
        drawPoints(image, filtered_points, filtered_color_, 1.0f);
    }
    
    return image;
}

bool Visualization::saveBEVImage(const std::vector<Point3D>& points,
                                const std::string& filename,
                                int width, int height,
                                float x_min, float y_min,
                                float x_max, float y_max) {
    
    cv::Mat image = createBEVImage(points, width, height, x_min, y_min, x_max, y_max);
    return cv::imwrite(filename, image);
}

bool Visualization::saveGroundNonGroundImage(const std::vector<Point3D>& ground_points,
                                            const std::vector<Point3D>& non_ground_points,
                                            const std::string& filename,
                                            int width, int height,
                                            float x_min, float y_min,
                                            float x_max, float y_max) {
    
    cv::Mat image = createGroundNonGroundImage(ground_points, non_ground_points, 
                                              width, height, x_min, y_min, x_max, y_max);
    return cv::imwrite(filename, image);
}

void Visualization::showImage(const cv::Mat& image, const std::string& window_name) {
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::imshow(window_name, image);
}

void Visualization::waitForKey(int delay_ms) {
    cv::waitKey(delay_ms);
}

cv::Point2i Visualization::worldToPixel(const Point3D& point, 
                                       int width, int height,
                                       float x_min, float y_min,
                                       float x_max, float y_max) {
    
    // Convert world coordinates to pixel coordinates
    float x_ratio = (point.x - x_min) / (x_max - x_min);
    float y_ratio = (point.y - y_min) / (y_max - y_min);
    
    // Flip Y axis (image coordinates are top-down)
    y_ratio = 1.0f - y_ratio;
    
    int pixel_x = static_cast<int>(x_ratio * width);
    int pixel_y = static_cast<int>(y_ratio * height);
    
    // Clamp to image bounds
    pixel_x = std::max(0, std::min(width - 1, pixel_x));
    pixel_y = std::max(0, std::min(height - 1, pixel_y));
    
    return cv::Point2i(pixel_x, pixel_y);
}

bool Visualization::isPointInBounds(const Point3D& point,
                                   float x_min, float y_min,
                                   float x_max, float y_max) {
    return point.x >= x_min && point.x <= x_max && 
           point.y >= y_min && point.y <= y_max;
}

void Visualization::drawPoints(cv::Mat& image, const std::vector<Point3D>& points,
                              const cv::Scalar& color, float point_size) {
    
    if (points.empty()) {
        return;
    }
    
    int width = image.cols;
    int height = image.rows;
    
    // Determine bounds from points if not specified
    float x_min = std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max();
    float x_max = -std::numeric_limits<float>::max();
    float y_max = -std::numeric_limits<float>::max();
    
    for (const auto& point : points) {
        x_min = std::min(x_min, point.x);
        y_min = std::min(y_min, point.y);
        x_max = std::max(x_max, point.x);
        y_max = std::max(y_max, point.y);
    }
    
    // Add some padding
    float padding = 5.0f;
    x_min -= padding;
    y_min -= padding;
    x_max += padding;
    y_max += padding;
    
    // Draw each point
    for (const auto& point : points) {
        if (isPointInBounds(point, x_min, y_min, x_max, y_max)) {
            cv::Point2i pixel = worldToPixel(point, width, height, x_min, y_min, x_max, y_max);
            
            if (point_size <= 1.0f) {
                image.at<cv::Vec3b>(pixel.y, pixel.x) = cv::Vec3b(color[0], color[1], color[2]);
            } else {
                int radius = static_cast<int>(point_size);
                cv::circle(image, pixel, radius, color, -1);
            }
        }
    }
}

} // namespace recursive_patchwork 