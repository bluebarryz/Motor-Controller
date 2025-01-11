#include "motor_controller/MotorSpeedController.hpp"

using namespace std::placeholders;

namespace composition {

MotorSpeedController::MotorSpeedController(const rclcpp::NodeOptions &options)
    : LifecycleNode("motor_speed_controller", options) {
    RCLCPP_INFO(get_logger(), "Initializing MotorSpeedController lifecycle");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorSpeedController::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Configuring MotorSpeedController");
    
    arcade_sub = create_subscription<motor_controller::msg::ArcadeSpeed>(
        "/cmd_vel", 10,
        std::bind(&MotorSpeedController::arcade_callback, this, _1));

    motor_speeds_pub = create_publisher<motor_controller::msg::MotorSpeeds>(
        "/cmd_vel_out", 10);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorSpeedController::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating MotorSpeedController");
    
    // Activate the lifecycle publisher
    motor_speeds_pub->on_activate();
    RCLCPP_INFO(get_logger(), "MotorSpeedController activated");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorSpeedController::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating MotorSpeedController");
    
    // Deactivate the lifecycle publisher
    motor_speeds_pub->on_deactivate();
    RCLCPP_INFO(get_logger(), "MotorSpeedController deactivated");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorSpeedController::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Cleaning up MotorSpeedController");
    
    arcade_sub.reset();
    motor_speeds_pub.reset();
    RCLCPP_INFO(get_logger(), "MotorSpeedController cleanup done");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorSpeedController::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Shutting down MotorSpeedController");
    
    arcade_sub.reset();
    motor_speeds_pub.reset();
    RCLCPP_INFO(get_logger(), "MotorSpeedController shutdown done");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MotorSpeedController::arcade_callback(const motor_controller::msg::ArcadeSpeed arcade_msg) {
    // Ignore arcade msg if component is inactive. This component shouldn't be inactive
    //  when ArcadeDriver is publishing ArcadeSpeed messages though.
	if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
		RCLCPP_WARN(get_logger(), "Received arcade message while not active, ignoring...");
		return;
	}
    float arcade_left = arcade_msg.l;
    float arcade_right = arcade_msg.r;

    motor_controller::msg::MotorSpeeds motor_speeds_msg = MotorSpeedController::compute_motor_speeds(arcade_left, arcade_right);
    motor_speeds_pub->publish(std::move(motor_speeds_msg));
}

motor_controller::msg::MotorSpeeds MotorSpeedController::compute_motor_speeds(const float arcade_l, const float arcade_r) {
    auto motor_speeds_msg = motor_controller::msg::MotorSpeeds();
    motor_speeds_msg.m1 = arcade_l * 0.8;
    motor_speeds_msg.m2 = arcade_l;
    motor_speeds_msg.m3 = arcade_l;
    motor_speeds_msg.m4 = arcade_r * 0.8;
    motor_speeds_msg.m5 = arcade_r;
    motor_speeds_msg.m6 = arcade_r;
    return motor_speeds_msg;
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::MotorSpeedController)