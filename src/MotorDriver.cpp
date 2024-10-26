#include "arcade_control/MotorDriver.hpp"

using namespace std::placeholders;

namespace composition {

MotorDriver::MotorDriver(const rclcpp::NodeOptions &options) : Node("motor_driver", options) {
    motor_server = this->create_service<arcade_control::srv::JoystickInput>("/joystick_input",
		std::bind(&MotorDriver::motor_server_callback, this, _1, _2));

    speed_pub = this->create_publisher<arcade_control::msg::Speed>(
        "/cmd_vel", 10);
}

void MotorDriver::motor_server_callback(const std::shared_ptr<arcade_control::srv::JoystickInput::Request> request,
    std::shared_ptr<arcade_control::srv::JoystickInput::Response> response) {
    
	Speed motor_speed = MotorDriver::joystick_to_speed_mapper(request->x, request->y);
	auto msg = arcade_control::msg::Speed();
	msg.r = motor_speed.r;
	msg.l = motor_speed.l;

	speed_pub->publish(std::move(msg));
	response->success = true;
}

Speed MotorDriver::joystick_to_speed_mapper(const int joystick_x, const int joystick_r) {
	return Speed{5, 5};
}


} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::MotorDriver)
