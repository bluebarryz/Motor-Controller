#include "arcade_control/MotorDriver.hpp"
#include <cmath>

using namespace std::placeholders;

namespace composition {

MotorDriver::MotorDriver(const rclcpp::NodeOptions &options) : Node("motor_driver", options), prev_joystick_rotate(-2), prev_joystick_drive(-2) {
    motor_server = this->create_service<arcade_control::srv::JoystickInput>("/joystick_input",
		std::bind(&MotorDriver::motor_server_callback, this, _1, _2));
	std::cout << "created service" <<std::endl;

    speed_pub = this->create_publisher<arcade_control::msg::Speed>(
        "/cmd_vel", 10);
}

bool MotorDriver::is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive) {
	return (fabs(new_joystick_rotate - prev_joystick_rotate) < THRESHOLD || 
		fabs(new_joystick_drive - prev_joystick_drive) < THRESHOLD);
}

void MotorDriver::motor_server_callback(const std::shared_ptr<arcade_control::srv::JoystickInput::Request> request,
    std::shared_ptr<arcade_control::srv::JoystickInput::Response> response) {
    
	if (is_negligible_joystick_change(request->x, request->y)) {
		std::cout << "negligible!" << std::endl;
		response->success = true;
		return;
	}

	Speed motor_speed = MotorDriver::joystick_to_speed_mapper(request->x, request->y);
	auto msg = arcade_control::msg::Speed();
	msg.r = motor_speed.r;
	msg.l = motor_speed.l;

	speed_pub->publish(std::move(msg));
	response->success = true;
	prev_joystick_rotate = request->x;
	prev_joystick_drive = request->y;
}

Speed MotorDriver::joystick_to_speed_mapper(const float joystick_rotate, const float joystick_drive) {
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

	RCLCPP_INFO(rclcpp::get_logger("MotorDriver"), "joystick: x=%.2f, y=%.2f, speed: l=%.2f, r=%.2f", joystick_rotate, joystick_drive, left_motor, right_motor);


	return Speed{left_motor, right_motor};
}


} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::MotorDriver)
