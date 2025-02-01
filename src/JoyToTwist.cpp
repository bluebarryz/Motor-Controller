#include "motor_controller/JoyToTwist.hpp"

using namespace std::placeholders;

namespace motor_controller {

JoyToTwist::JoyToTwist(const rclcpp::NodeOptions & options)
: Node("joy_to_twist", options) {
    // Declare parameters
    this->declare_parameter("axis_linear", 1);    // Left stick vertical
    this->declare_parameter("axis_angular", 0);   // Left stick horizontal
    this->declare_parameter("scale_linear", 1.0);
    this->declare_parameter("scale_angular", 1.0);
    
    // Get parameters
    axis_linear_ = this->get_parameter("axis_linear").as_int();
    axis_angular_ = this->get_parameter("axis_angular").as_int();
    scale_linear_ = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();
    
    // Publisher for twist msgs
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/joystick_input", 10);
    
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, 
        std::bind(&JoyToTwist::joy_callback, this, _1));
        
    RCLCPP_INFO(get_logger(), "JoyToTwist node initialized");
}

void JoyToTwist::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    auto twist_msg = geometry_msgs::msg::Twist();
    
    // Check if the message contains enough axes
    if (joy_msg->axes.size() > static_cast<size_t>(std::max(axis_linear_, axis_angular_))) {
        twist_msg.linear.x = scale_linear_ * joy_msg->axes[axis_linear_];
        twist_msg.angular.z = scale_angular_ * joy_msg->axes[axis_angular_];
        
        RCLCPP_DEBUG(get_logger(), "Publishing Twist - linear.x: %.2f, angular.z: %.2f",
                     twist_msg.linear.x, twist_msg.angular.z);

        if (twist_msg.linear.x > 0 || twist_msg.angular.z > 0) {  
            twist_pub_->publish(twist_msg);
        }
    } else {
        RCLCPP_WARN(get_logger(), "Joy message doesn't contain enough axes");
    }
}

} // namespace motor_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motor_controller::JoyToTwist)