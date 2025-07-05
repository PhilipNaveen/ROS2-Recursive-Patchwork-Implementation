#pragma once

#include "recursive_patchwork.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace recursive_patchwork {

class Visualization {
public:
    Visualization();
    ~Visualization();

    // BEV (Bird's Eye View) visualization
    cv::Mat createBEVImage(const std::vector<Point3D>& points,
                          int width = 300, int height = 150,
                          float x_min = -150.0f, float y_min = -75.0f,
                          float x_max = 150.0f, float y_max = 75.0f);

    // Ground vs non-ground visualization
    cv::Mat createGroundNonGroundImage(const std::vector<Point3D>& ground_points,
                                      const std::vector<Point3D>& non_ground_points,
                                      int width = 300, int height = 150,
                                      float x_min = -150.0f, float y_min = -75.0f,
                                      float x_max = 150.0f, float y_max = 75.0f);

    // Enhanced filtering visualization
    cv::Mat createEnhancedFilteredImage(const std::vector<Point3D>& filtered_points,
                                       int width = 300, int height = 150,
                                       float x_min = -150.0f, float y_min = -75.0f,
                                       float x_max = 150.0f, float y_max = 75.0f);

    // Save functions
    bool saveBEVImage(const std::vector<Point3D>& points,
                     const std::string& filename,
                     int width = 300, int height = 150,
                     float x_min = -150.0f, float y_min = -75.0f,
                     float x_max = 150.0f, float y_max = 75.0f);

    bool saveGroundNonGroundImage(const std::vector<Point3D>& ground_points,
                                 const std::vector<Point3D>& non_ground_points,
                                 const std::string& filename,
                                 int width = 300, int height = 150,
                                 float x_min = -150.0f, float y_min = -75.0f,
                                 float x_max = 150.0f, float y_max = 75.0f);

    // Display functions
    void showImage(const cv::Mat& image, const std::string& window_name = "Recursive Patchwork");
    void waitForKey(int delay_ms = 0);

    // Color configuration
    void setGroundColor(const cv::Scalar& color) { ground_color_ = color; }
    void setNonGroundColor(const cv::Scalar& color) { non_ground_color_ = color; }
    void setFilteredColor(const cv::Scalar& color) { filtered_color_ = color; }
    void setBackgroundColor(const cv::Scalar& color) { background_color_ = color; }

private:
    cv::Scalar ground_color_;
    cv::Scalar non_ground_color_;
    cv::Scalar filtered_color_;
    cv::Scalar background_color_;

    // Helper functions
    cv::Point2i worldToPixel(const Point3D& point, 
                            int width, int height,
                            float x_min, float y_min,
                            float x_max, float y_max);
    
    bool isPointInBounds(const Point3D& point,
                        float x_min, float y_min,
                        float x_max, float y_max);

    void drawPoints(cv::Mat& image, const std::vector<Point3D>& points,
                   const cv::Scalar& color, float point_size = 1.0f);
};

} // namespace recursive_patchwork 