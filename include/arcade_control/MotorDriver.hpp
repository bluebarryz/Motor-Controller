#include <rclcpp/rclcpp.hpp>
#include "arcade_control/Speed.hpp"
#include "arcade_control/srv/joystick_input.hpp"
#include "arcade_control/msg/speed.hpp"


namespace composition {

class MotorDriver : public rclcpp::Node {
public:
    explicit MotorDriver(const rclcpp::NodeOptions &options);
private:
    rclcpp::Service<arcade_control::srv::JoystickInput>::SharedPtr motor_server;
    rclcpp::Publisher<arcade_control::msg::Speed>::SharedPtr speed_pub;

    Speed joystick_to_speed_mapper(const float joystick_rotate, const float joystick_drive);
    void motor_server_callback(const std::shared_ptr<arcade_control::srv::JoystickInput::Request> request,
        std::shared_ptr<arcade_control::srv::JoystickInput::Response> response);
    bool is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive);
    float prev_joystick_rotate;
    float prev_joystick_drive;
    const float THRESHOLD = 0.05;
};

} // namespace composition