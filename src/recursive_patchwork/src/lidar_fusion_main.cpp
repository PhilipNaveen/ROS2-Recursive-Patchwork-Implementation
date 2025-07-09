#include "lidar_fusion_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<recursive_patchwork::LidarFusionNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting LiDAR Fusion Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 