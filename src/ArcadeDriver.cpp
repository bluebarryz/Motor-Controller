#include "motor_controller/ArcadeDriver.hpp"
#include <cmath>

using namespace std::placeholders;

namespace composition {

ArcadeDriver::ArcadeDriver(const rclcpp::NodeOptions &options)
    : LifecycleNode("arcade_driver", options) {
    RCLCPP_INFO(get_logger(), "Initializing ArcadeDriver lifecycle");
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ArcadeDriver::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Configuring ArcadeDriver");
    
    joystick_sub = create_subscription<geometry_msgs::msg::Twist>(
        "/joystick_input", 10,
        std::bind(&ArcadeDriver::joystick_callback, this, _1));
        
    arcade_pub = create_publisher<motor_controller::msg::ArcadeSpeed>(
        "/cmd_vel", 10);
        
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ArcadeDriver::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating ArcadeDriver");
    
    // Activate the lifecycle publisher
    arcade_pub->on_activate();
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ArcadeDriver::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating ArcadeDriver");
    
    // Deactivate the lifecycle publisher
    arcade_pub->on_deactivate();
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ArcadeDriver::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Cleaning up ArcadeDriver");
    
    joystick_sub.reset();
    arcade_pub.reset();
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ArcadeDriver::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Shutting down ArcadeDriver");
    
    joystick_sub.reset();
    arcade_pub.reset();
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// joystick input is "negligible" if it is very close to (0, 0), basically
bool ArcadeDriver::is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive) {
	return (fabs(new_joystick_rotate) + fabs(new_joystick_drive) < THRESHOLD);
}

void ArcadeDriver::joystick_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
	// Ignore Twist msg if component is inactive
	if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
		RCLCPP_WARN(get_logger(), "Received twist message while not active, ignoring...");
		return;
	}

	RCLCPP_INFO(get_logger(), "Received Twist message - linear.x: %.2f, angular.z: %.2f",
				msg->linear.x, msg->angular.z);

	float joystick_drive = msg->linear.x;
	float joystick_rotate = msg->angular.z;

	if (is_negligible_joystick_change(joystick_rotate, joystick_drive)) {
		RCLCPP_INFO(rclcpp::get_logger("ArcadeDriver"), "Negligibe joystick change");
		return;
	}

	motor_controller::msg::ArcadeSpeed arcade_msg = ArcadeDriver::joystick_to_speed_mapper(joystick_rotate, joystick_drive);
	RCLCPP_INFO(get_logger(), "Publishing ArcadeSpeed - left: %.2f, right: %.2f",
				arcade_msg.l, arcade_msg.r);
	arcade_pub->publish(std::move(arcade_msg));
}

motor_controller::msg::ArcadeSpeed ArcadeDriver::joystick_to_speed_mapper(const float joystick_rotate, const float joystick_drive) {
	const float MAX = fmax(fabs(joystick_drive), fabs(joystick_rotate));
	const float DIFF = joystick_drive - joystick_rotate;
	const float TOTAL = joystick_drive + joystick_rotate;

	/*
	maximum = max(abs(drive), abs(rotate))
	total, difference = drive + rotate, drive - rotate
		# set speed according to the quadrant that the values are in
	if drive >= 0:
		if rotate >= 0:  # I quadrant
			left_motor(maximum)
			right_motor(difference)
		else:            # II quadrant
			left_motor(total)
			right_motor(maximum)
	else:
		if rotate >= 0:  # IV quadrant
			left_motor(total)
			right_motor(-maximum)
		else:            # III quadrant
			left_motor(-maximum)
			right_motor(difference)
	*/

	float left_motor;
	float right_motor;

	if (joystick_drive >= 0) {
		if (joystick_rotate >= 0) {
			left_motor = MAX;
			right_motor = DIFF;
		} else {
			left_motor = TOTAL;
			right_motor = MAX;
		}
	} else {
		if (joystick_rotate >= 0) {
			left_motor = TOTAL;
			right_motor = -1 * MAX;
		} else {
			left_motor = -1 * MAX;
			right_motor = DIFF;
		}
	}

	RCLCPP_INFO(rclcpp::get_logger("ArcadeDriver"), "joystick: x=%.2f, y=%.2f, speed: l=%.2f, r=%.2f", joystick_rotate, joystick_drive, left_motor, right_motor);

	auto arcade_msg = motor_controller::msg::ArcadeSpeed();
	arcade_msg.l = left_motor;
	arcade_msg.r = right_motor;
	return arcade_msg;
}


} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::ArcadeDriver)
