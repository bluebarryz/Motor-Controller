#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arcade_control/MotorDriver.hpp"
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<composition::MotorDriver>(rclcpp::NodeOptions()));
    
    rclcpp::shutdown();
    return 0;
}