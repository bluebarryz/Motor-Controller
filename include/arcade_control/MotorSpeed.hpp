#include <rclcpp/rclcpp.hpp>
#include "arcade_control/ArcadeSpeed.hpp"
#include "arcade_control/srv/joystick_input.hpp"
#include "arcade_control/msg/arcade_speed.hpp"


namespace composition {

class MotorSpeed : public rclcpp::Node {
public:
    explicit MotorSpeed(const rclcpp::NodeOptions &options);
private:
    rclcpp::Service<arcade_control::srv::JoystickInput>::SharedPtr motor_server;

    rclcpp::Subscription<arcade_control::msg::ArcadeSpeed>::SharedPtr arcade_driver_sub;
    rclcpp::Publisher<arcade_control::msg::MotorSpeed>::SharedPtr motor_speed_pub;

    ArcadeSpeed joystick_to_speed_mapper(const float joystick_rotate, const float joystick_drive);
    void motor_server_callback(const std::shared_ptr<arcade_control::srv::JoystickInput::Request> request,
        std::shared_ptr<arcade_control::srv::JoystickInput::Response> response);
    bool is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive);
    float prev_joystick_rotate;
    float prev_joystick_drive;
    const float THRESHOLD = 0.05;
};

} // namespace composition