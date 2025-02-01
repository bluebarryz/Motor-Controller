#ifndef JOY_TO_TWIST_HPP_
#define JOY_TO_TWIST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace motor_controller {

class JoyToTwist : public rclcpp::Node {
public:
    explicit JoyToTwist(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    
    // Params
    int axis_linear_;
    int axis_angular_;
    double scale_linear_;
    double scale_angular_;
};

}

#endif