#include <rclcpp/rclcpp.hpp>
#include "motor_controller/msg/arcade_speed.hpp"
#include "motor_controller/msg/motor_speeds.hpp"


namespace composition {

class MotorSpeedController : public rclcpp::Node {
public:
    explicit MotorSpeedController(const rclcpp::NodeOptions &options);
private:
    // subscriber to arcade speed topic (/cmd_vel)
    rclcpp::Subscription<motor_controller::msg::ArcadeSpeed>::SharedPtr arcade_sub;
    
    // publisher(s) to set motor speeds
    rclcpp::Publisher<motor_controller::msg::MotorSpeeds>::SharedPtr motor_speeds_pub;

    // function(s) to compute motor speeds
    void arcade_callback(const motor_controller::msg::ArcadeSpeed arcade_msg);
    motor_controller::msg::MotorSpeeds compute_motor_speeds(const float arcade_l, const float arcade_r);
};

} // namespace composition