#include "arcade_control/MotorSpeedController.hpp"

using namespace std::placeholders;

namespace composition {

MotorSpeedController::MotorSpeedController(const rclcpp::NodeOptions &options) : Node("motor_speed_controller", options) {
    arcade_sub = this->create_subscription<arcade_control::msg::ArcadeSpeed>(
        "/cmd_vel", 10,
        std::bind(&MotorSpeedController::arcade_callback, this, _1));

    motor_speeds_pub = this->create_publisher<arcade_control::msg::MotorSpeeds>(
        "/cmd_vel_out", 10);
}

void MotorSpeedController::arcade_callback(const arcade_control::msg::ArcadeSpeed arcade_msg) {
    float arcade_left = arcade_msg.l;
    float arcade_right = arcade_msg.r;

    MotorSpeeds motor_speed_values = MotorSpeedController::compute_motor_speeds(arcade_left, arcade_right);
    auto motor_speeds_msg = arcade_control::msg::MotorSpeeds();
    motor_speeds_msg.m1 = motor_speed_values.m1;
    motor_speeds_msg.m2 = motor_speed_values.m2;
    motor_speeds_msg.m3 = motor_speed_values.m3;
    motor_speeds_msg.m4 = motor_speed_values.m4;
    motor_speeds_msg.m5 = motor_speed_values.m5;
    motor_speeds_msg.m6 = motor_speed_values.m6;

    motor_speeds_pub->publish(std::move(motor_speeds_msg));
}

MotorSpeeds MotorSpeedController::compute_motor_speeds(const float arcade_l, const float arcade_r) {
    float m1 = arcade_l * 0.8;
    float m2 = arcade_l;
    float m3 = arcade_l;
    float m4 = arcade_r * 0.8;
    float m5 = arcade_r;
    float m6 = arcade_r;
    return MotorSpeeds{m1, m2, m3, m4, m5, m6};
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::MotorSpeedController)