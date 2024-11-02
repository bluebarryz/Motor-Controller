#include <rclcpp/rclcpp.hpp>
#include "arcade_control/MotorSpeeds.hpp"
#include "arcade_control/msg/arcade_speed.hpp"
#include "arcade_control/msg/motor_speeds.hpp"


namespace composition {

class MotorSpeedController : public rclcpp::Node {
public:
    explicit MotorSpeedController(const rclcpp::NodeOptions &options);
private:
    // subscriber to arcade speed topic (/cmd_vel)
    rclcpp::Subscription<arcade_control::msg::ArcadeSpeed>::SharedPtr arcade_sub;
    
    // publisher(s) to set motor speeds
    rclcpp::Publisher<arcade_control::msg::MotorSpeeds>::SharedPtr motor_speeds_pub;

    // function(s) to compute motor speeds
    void arcade_callback(const arcade_control::msg::ArcadeSpeed arcade_msg);
    MotorSpeeds compute_motor_speeds(const float arcade_l, const float arcade_r);
};

} // namespace composition