#include "recursive_patchwork.hpp"
#include "point_cloud_processor.hpp"
#include "lidar_fusion.hpp"
#include "visualization.hpp"

#ifdef USE_ROS2
#include "rosbag_loader.hpp"
#endif

#include <iostream>
#include <string>
#include <vector>
#include <chrono>

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
              << "Options:\n"
#ifdef USE_ROS2
              << "  <bag_path>                    Path to ROS2 bag file\n"
              << "  --topics <topic1> [topic2] ... Topic names (default: /lidar_points)\n"
              << "  --frame <number>              Frame number (default: 0)\n"
#else
              << "  --demo                         Run demo with synthetic data\n"
#endif
              << "  --bev-width <pixels>          BEV image width (default: 300)\n"
              << "  --bev-height <pixels>         BEV image height (default: 150)\n"
              << "  --x-min <meters>              Minimum X coordinate (default: -150)\n"
              << "  --y-min <meters>              Minimum Y coordinate (default: -75)\n"
              << "  --use-patchwork               Use Recursive Patchwork filtering\n"
              << "  --target-height <meters>      Target height for obstacles (default: 1.1)\n"
              << "  --height-tolerance <meters>   Height tolerance (default: 0.5)\n"
              << "  --separate-display            Display ground and non-ground separately\n"
              << "  --help                        Show this help message\n"
              << "\n"
              << "Examples:\n"
#ifdef USE_ROS2
              << "  " << program_name << " data.bag --frame 10\n"
              << "  " << program_name << " data.mcap --topics /lidar_front /lidar_left /lidar_right --use-patchwork\n"
              << "  " << program_name << " data.db3 --bev-width 600 --bev-height 300 --separate-display\n";
#else
              << "  " << program_name << " --demo --use-patchwork\n"
              << "  " << program_name << " --demo --bev-width 600 --bev-height 300 --separate-display\n";
#endif
}

// Generate synthetic point cloud for demo
std::vector<recursive_patchwork::Point3D> generateDemoPointCloud(size_t num_points = 10000) {
    std::vector<recursive_patchwork::Point3D> points;
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
        recursive_patchwork::Point3D point;
        point.x = ground_xy(gen);
        point.y = ground_xy(gen);
        point.z = ground_z(gen);
        points.push_back(point);
    }
    
    // Obstacle points (cars, pedestrians, etc.)
    for (size_t i = 0; i < obstacle_count; ++i) {
        recursive_patchwork::Point3D point;
        point.x = obstacle_xy(gen);
        point.y = obstacle_xy(gen);
        point.z = obstacle_z(gen);
        points.push_back(point);
    }
    
    return points;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    // Default parameters
    std::string bag_path;
    std::vector<std::string> topic_names = {"/lidar_points"};
    size_t frame_number = 0;
    int bev_width = 300;
    int bev_height = 150;
    float x_min = -150.0f;
    float y_min = -75.0f;
    bool use_patchwork = false;
    float target_height = 1.1f;
    float height_tolerance = 0.5f;
    bool separate_display = false;
    bool demo_mode = false;
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--demo") {
            demo_mode = true;
        } else if (arg == "--topics") {
            topic_names.clear();
            while (++i < argc && argv[i][0] != '-') {
                topic_names.push_back(argv[i]);
            }
            --i; // Adjust for the loop increment
        } else if (arg == "--frame" && i + 1 < argc) {
            frame_number = std::stoul(argv[++i]);
        } else if (arg == "--bev-width" && i + 1 < argc) {
            bev_width = std::stoi(argv[++i]);
        } else if (arg == "--bev-height" && i + 1 < argc) {
            bev_height = std::stoi(argv[++i]);
        } else if (arg == "--x-min" && i + 1 < argc) {
            x_min = std::stof(argv[++i]);
        } else if (arg == "--y-min" && i + 1 < argc) {
            y_min = std::stof(argv[++i]);
        } else if (arg == "--use-patchwork") {
            use_patchwork = true;
        } else if (arg == "--target-height" && i + 1 < argc) {
            target_height = std::stof(argv[++i]);
        } else if (arg == "--height-tolerance" && i + 1 < argc) {
            height_tolerance = std::stof(argv[++i]);
        } else if (arg == "--separate-display") {
            separate_display = true;
        } else if (arg[0] != '-') {
            // First non-flag argument is the bag path
            if (bag_path.empty()) {
                bag_path = arg;
            }
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    
    // Validate parameters
    if (bev_width <= 0 || bev_height <= 0) {
        std::cerr << "Error: BEV width and height must be positive" << std::endl;
        return 1;
    }
    
    // Check if we have a bag path or demo mode
#ifdef USE_ROS2
    if (bag_path.empty() && !demo_mode) {
        std::cerr << "Error: Must provide a bag file path or use --demo mode" << std::endl;
        printUsage(argv[0]);
        return 1;
    }
#else
    if (!demo_mode) {
        std::cout << "ROS2 support not available. Running in demo mode with synthetic data." << std::endl;
        demo_mode = true;
    }
#endif
    
    std::cout << "=== Recursive Patchwork C++ Implementation ===" << std::endl;
    if (demo_mode) {
        std::cout << "Mode: Demo (synthetic data)" << std::endl;
    } else {
        std::cout << "Bag file: " << bag_path << std::endl;
        std::cout << "Topics: ";
        for (const auto& topic : topic_names) {
            std::cout << topic << " ";
        }
        std::cout << std::endl;
        std::cout << "Frame: " << frame_number << std::endl;
    }
    std::cout << "BEV size: " << bev_width << "x" << bev_height << std::endl;
    std::cout << "Use Patchwork: " << (use_patchwork ? "Yes" : "No") << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Initialize components
        recursive_patchwork::LidarFusion lidar_fusion;
        recursive_patchwork::RecursivePatchwork patchwork;
        recursive_patchwork::Visualization viz;
        
        std::vector<recursive_patchwork::Point3D> fused_points;
        
        if (demo_mode) {
            // Generate synthetic data for demo
            std::cout << "Generating synthetic point cloud..." << std::endl;
            fused_points = generateDemoPointCloud(10000);
            std::cout << "Generated " << fused_points.size() << " synthetic points" << std::endl;
        } else {
#ifdef USE_ROS2
            // Load from ROS2 bag file
            recursive_patchwork::RosbagLoader bag_loader;
            
            // Open bag file
            if (!bag_loader.openBag(bag_path)) {
                std::cerr << "Error opening bag file: " << bag_loader.getLastError() << std::endl;
                return 1;
            }
            
            // Load point clouds
            std::vector<std::vector<recursive_patchwork::Point3D>> lidar_point_clouds;
            
            if (topic_names.size() == 1) {
                // Single LiDAR mode
                auto points = bag_loader.loadPointCloud(topic_names[0], frame_number);
                if (points.empty()) {
                    std::cerr << "Error: No points loaded from topic " << topic_names[0] << std::endl;
                    return 1;
                }
                lidar_point_clouds.push_back(points);
            } else {
                // Multiple LiDAR fusion mode
                lidar_point_clouds = bag_loader.loadMultiplePointClouds(topic_names, frame_number);
                
                // Check if any point clouds were loaded
                bool has_points = false;
                for (const auto& cloud : lidar_point_clouds) {
                    if (!cloud.empty()) {
                        has_points = true;
                        break;
                    }
                }
                
                if (!has_points) {
                    std::cerr << "Error: No points loaded from any topic" << std::endl;
                    return 1;
                }
            }
            
            // Fuse LiDAR point clouds
            if (lidar_point_clouds.size() == 1) {
                fused_points = lidar_point_clouds[0];
            } else {
                fused_points = lidar_fusion.fuseLidarPointClouds(lidar_point_clouds);
            }
#else
            std::cerr << "ROS2 support not available. Use --demo mode for testing." << std::endl;
            return 1;
#endif
        }
        
        if (fused_points.empty()) {
            std::cerr << "Error: No points available for processing" << std::endl;
            return 1;
        }
        
        std::cout << "Total points: " << fused_points.size() << std::endl;
        
        // Process with Recursive Patchwork if requested
        if (use_patchwork) {
            std::cout << "\nApplying Recursive Patchwork filtering..." << std::endl;
            
            // Separate ground and non-ground points
            auto [ground_points, non_ground_points] = patchwork.filterGroundPoints(fused_points);
            
            std::cout << "Ground points: " << ground_points.size() << std::endl;
            std::cout << "Non-ground points: " << non_ground_points.size() << std::endl;
            
            // Create visualizations
            std::string base_filename = demo_mode ? "demo_frame" : "lidar_bev_frame_" + std::to_string(frame_number);
            
            if (separate_display) {
                // Save ground vs non-ground visualization
                std::string ground_non_ground_filename = base_filename + "_patchwork.png";
                viz.saveGroundNonGroundImage(ground_points, non_ground_points, 
                                           ground_non_ground_filename,
                                           bev_width, bev_height, x_min, y_min, 
                                           x_min + bev_width, y_min + bev_height);
                std::cout << "Saved: " << ground_non_ground_filename << std::endl;
                
                // Enhanced filtering
                auto filtered_points = patchwork.sampleGroundAndObstacles(fused_points, 
                                                                        target_height, height_tolerance);
                
                std::string enhanced_filename = base_filename + "_enhanced.png";
                viz.saveBEVImage(filtered_points, enhanced_filename,
                                bev_width, bev_height, x_min, y_min,
                                x_min + bev_width, y_min + bev_height);
                std::cout << "Saved: " << enhanced_filename << std::endl;
            } else {
                // Save only non-ground points
                std::string patchwork_filename = base_filename + "_patchwork.png";
                viz.saveBEVImage(non_ground_points, patchwork_filename,
                                bev_width, bev_height, x_min, y_min,
                                x_min + bev_width, y_min + bev_height);
                std::cout << "Saved: " << patchwork_filename << std::endl;
            }
        } else {
            // Original visualization without filtering
            std::string original_filename = demo_mode ? "demo_original.png" : 
                                          "lidar_bev_frame_" + std::to_string(frame_number) + ".png";
            viz.saveBEVImage(fused_points, original_filename,
                            bev_width, bev_height, x_min, y_min,
                            x_min + bev_width, y_min + bev_height);
            std::cout << "Saved: " << original_filename << std::endl;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        std::cout << "\nProcessing completed in " << duration.count() << " ms" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 