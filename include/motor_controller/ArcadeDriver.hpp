#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "motor_controller/msg/arcade_speed.hpp"


namespace composition {

class ArcadeDriver : public rclcpp::Node {
public:
    explicit ArcadeDriver(const rclcpp::NodeOptions &options);
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joystick_sub;
    rclcpp::Publisher<motor_controller::msg::ArcadeSpeed>::SharedPtr arcade_pub;

    motor_controller::msg::ArcadeSpeed joystick_to_speed_mapper(const float joystick_rotate, const float joystick_drive);
    void joystick_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive);
    const float THRESHOLD = 0.05;
};

} // namespace composition