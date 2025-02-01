#ifndef MOTOR_SPEED_CONTROLLER_HPP_
#define MOTOR_SPEED_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "motor_controller/msg/arcade_speed.hpp"
#include "motor_controller/msg/motor_speeds.hpp"
#include <lifecycle_msgs/msg/state.hpp>


namespace composition {

class MotorSpeedController : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MotorSpeedController(const rclcpp::NodeOptions &options);
private:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &);

    // subscriber to arcade speed topic (/cmd_vel)
    rclcpp::Subscription<motor_controller::msg::ArcadeSpeed>::SharedPtr arcade_sub;
    
    // publisher(s) to set motor speeds
    rclcpp_lifecycle::LifecyclePublisher<motor_controller::msg::MotorSpeeds>::SharedPtr motor_speeds_pub;

    // function(s) to compute motor speeds
    void arcade_callback(const motor_controller::msg::ArcadeSpeed arcade_msg);
    motor_controller::msg::MotorSpeeds compute_motor_speeds(const float arcade_l, const float arcade_r);
};

} // namespace composition

#endif
