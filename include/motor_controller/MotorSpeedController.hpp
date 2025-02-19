#ifndef MOTOR_SPEED_CONTROLLER_HPP_
#define MOTOR_SPEED_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "motor_controller/msg/arcade_speed.hpp"
#include "motor_controller/msg/motor_speeds.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>


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
    
    // publishers to set motor speeds
    rclcpp_lifecycle::LifecyclePublisher<motor_controller::msg::MotorSpeeds>::SharedPtr motor_speeds_pub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr odrive_pub;

    // functions to compute motor speeds
    void arcade_callback(const motor_controller::msg::ArcadeSpeed arcade_msg);
    motor_controller::msg::MotorSpeeds compute_motor_speeds(const float arcade_l, const float arcade_r);
    
    void publish_speeds_odrive(motor_controller::msg::MotorSpeeds speeds);
    const nlohmann::json odrive_speed_req_json = {
        {"Stage", "run"},
        {"Type", "request"},
        {"Target", "Drivetrain"},
        {"Command", "Set_Input_Vel"}
    };
};

} // namespace composition

#endif
