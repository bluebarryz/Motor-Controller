#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arcade_control/ArcadeDriver.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "arcade_control/msg/speed.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TestArcadeDriver : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        test_node = std::make_shared<rclcpp::Node>("test_node");
        
        arcade_driver = std::make_shared<composition::ArcadeDriver>(rclcpp::NodeOptions());

        last_speed_msg = std::make_shared<arcade_control::msg::ArcadeSpeed>();
        last_speed_msg->l = last_speed_msg->r = 0;
        speed_sub = test_node->create_subscription<arcade_control::msg::ArcadeSpeed>(
            "/cmd_vel",
            10,
            [this](const arcade_control::msg::ArcadeSpeed::SharedPtr msg) {
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Message: Left speed %.2f, Right speed %.2f", msg->l, msg->r);
                last_speed_msg = msg;
            });

        joystick_pub = test_node->create_publisher<geometry_msgs::msg::Twist>(
            "/joystick_input",
            10);
        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Setup completed");
    }

    void TearDown() override {
        arcade_driver.reset();
        test_node.reset();
        rclcpp::shutdown();
        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Teardown completed");
    }

    std::shared_ptr<rclcpp::Node> test_node;
    std::shared_ptr<composition::ArcadeDriver> arcade_driver;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joystick_pub;
    rclcpp::Subscription<arcade_control::msg::ArcadeSpeed>::SharedPtr speed_sub;
    arcade_control::msg::ArcadeSpeed::SharedPtr last_speed_msg;
    const float TOL = 0.0001;

    void publish_joystick_input(float joystick_rotate, float joystick_drive) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = joystick_drive;
        twist_msg.angular.z = joystick_rotate;
        
        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), 
                    "Publishing twist: linear.x = %.2f, angular.z = %.2f", 
                    twist_msg.linear.x, twist_msg.angular.z);
        
        joystick_pub->publish(twist_msg);
    }
    
    // Wait for the /cmd_vel subscriber to receive the expected speed values
    bool wait_for_speed_message(float expected_l, float expected_r, const std::chrono::seconds timeout = std::chrono::seconds(3)) {
        auto start_time = std::chrono::steady_clock::now();
        
        while (true) {
            rclcpp::spin_some(test_node);
            rclcpp::spin_some(arcade_driver);
            
            if (last_speed_msg != nullptr) {
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Received speed message: l=%.2f, r=%.2f", last_speed_msg->l, last_speed_msg->r);
                
                if (std::abs(last_speed_msg->r - expected_r) < TOL && std::abs(last_speed_msg->l - expected_l) < TOL) {
                    return true;
                }
            }
            
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                RCLCPP_ERROR(rclcpp::get_logger("TestArcadeDriver"), "Timed out waiting for expected speed message");
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Expected speed message: l=%.2f, r=%.2f", expected_l, expected_r);
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Received speed message: l=%.2f, r=%.2f", last_speed_msg->l, last_speed_msg->r);
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void assert_motor_speed(float joystick_rotate, float joystick_drive, float expected_l, float expected_r) {
        publish_joystick_input(joystick_rotate, joystick_drive);
        ASSERT_TRUE(wait_for_speed_message(expected_l, expected_r));
        ASSERT_NE(last_speed_msg, nullptr);
        
        EXPECT_NEAR(last_speed_msg->l, expected_l, TOL);
        EXPECT_NEAR(last_speed_msg->r, expected_r, TOL);
    }
};

// Joystick movement away from rest state is beneath the threshold
TEST_F(TestArcadeDriver, Test0) {
    float joystick_rotate = 0.01;
    float joystick_drive = -0.02;
    float expected_l = 0;
    float expected_r = 0;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test1) {    
    float joystick_rotate = 0.0;
    float joystick_drive = 1.0;
    float expected_l = 1.0;
    float expected_r = 1.0;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test2) {
    float joystick_rotate = 0.0;
    float joystick_drive = 0 - 1.0;
    float expected_l = 0 - 1.0;
    float expected_r = 0 - 1.0;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test3) {
    float joystick_rotate_0 = 0.5;
    float joystick_drive_0 = 0.5;
    float expected_l_0 = 0.5;
    float expected_r_0 = 0;
    assert_motor_speed(joystick_rotate_0, joystick_drive_0, expected_l_0, expected_r_0);

    float joystick_rotate = 0.51;
    float joystick_drive = 0.49;
    float expected_l_1 = 0.51;
    float expected_r_1 = -0.02;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l_1, expected_r_1);
}

TEST_F(TestArcadeDriver, Test4) {
    float joystick_rotate = 0.77;
    float joystick_drive = 0.35;
    float expected_l = 0.77;
    float expected_r = -0.42;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test5) {
    float joystick_rotate = -0.67;
    float joystick_drive = 0.2;
    float expected_l = -0.47;
    float expected_r = 0.67;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test6) {
    float joystick_rotate = -0.8;
    float joystick_drive = -0.77;
    float expected_l = -0.8;
    float expected_r = 0.03;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test7) {
    float joystick_rotate = 0.12;
    float joystick_drive = -0.68;
    float expected_l = -0.56;
    float expected_r = -0.68;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l, expected_r);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}