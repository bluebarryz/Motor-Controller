#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "motor_controller/ArcadeDriver.hpp"
#include "motor_controller/MotorSpeedController.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_controller/msg/arcade_speed.hpp"
#include "motor_controller/msg/motor_speeds.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TestArcadeDriver : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
        test_node = std::make_shared<rclcpp::Node>("test_node");
        
        arcade_driver = std::make_shared<composition::ArcadeDriver>(rclcpp::NodeOptions());
        motor_speed_controller = std::make_shared<composition::MotorSpeedController>(rclcpp::NodeOptions());

        last_arcade_speed_msg = std::make_shared<motor_controller::msg::ArcadeSpeed>();
        last_arcade_speed_msg->l = last_arcade_speed_msg->r = 0;
        arcade_speed_sub = test_node->create_subscription<motor_controller::msg::ArcadeSpeed>(
            "/cmd_vel",
            10,
            [](const motor_controller::msg::ArcadeSpeed::SharedPtr msg) {
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Message: Left speed %.2f, Right speed %.2f", msg->l, msg->r);
                last_arcade_speed_msg = msg;
            });

        joystick_pub = test_node->create_publisher<geometry_msgs::msg::Twist>(
            "/joystick_input",
            10);

        last_motor_speed_msg = std::make_shared<motor_controller::msg::MotorSpeeds>();
        last_motor_speed_msg->m1 = last_motor_speed_msg->m2 = last_motor_speed_msg->m3 = last_motor_speed_msg->m4 = last_motor_speed_msg->m5 = last_motor_speed_msg->m6 = 0;
        motor_speed_sub = test_node->create_subscription<motor_controller::msg::MotorSpeeds>(
            "/cmd_vel_out",
            10,
            [](const motor_controller::msg::MotorSpeeds::SharedPtr msg) {
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Message: m1=%.2f, m2=%.2f, m3=%.2f, m4=%.2f, m5=%.2f, m6=%.2f", 
                    msg->m1, msg->m2, msg->m3, msg->m4, msg->m5, msg->m6);
                last_motor_speed_msg = msg;
            });
        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Setup completed");
    }

    static void TearDownTestSuite() {
        arcade_speed_sub.reset();
        motor_speed_sub.reset();
        joystick_pub.reset();

        last_arcade_speed_msg.reset();
        last_motor_speed_msg.reset();

        arcade_driver.reset();
        motor_speed_controller.reset();
        test_node.reset();

        rclcpp::shutdown();

        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Teardown completed");
    }

    static std::shared_ptr<rclcpp::Node> test_node;
    static std::shared_ptr<composition::ArcadeDriver> arcade_driver;
    static std::shared_ptr<composition::MotorSpeedController> motor_speed_controller;
    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joystick_pub;
    static rclcpp::Subscription<motor_controller::msg::ArcadeSpeed>::SharedPtr arcade_speed_sub;
    static motor_controller::msg::ArcadeSpeed::SharedPtr last_arcade_speed_msg;
    static rclcpp::Subscription<motor_controller::msg::MotorSpeeds>::SharedPtr motor_speed_sub;
    static motor_controller::msg::MotorSpeeds::SharedPtr last_motor_speed_msg;
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
    
    // Wait for the /cmd_vel and /cmd_vel_out subscribers to receive the expected speed values
    bool wait_for_speed_messages(float expected_l, float expected_r, const std::chrono::seconds timeout = std::chrono::seconds(3)) {
        auto start_time = std::chrono::steady_clock::now();
        
        while (true) {
            rclcpp::spin_some(test_node);
            rclcpp::spin_some(arcade_driver);
            rclcpp::spin_some(motor_speed_controller);
            
            if (last_arcade_speed_msg != nullptr && last_motor_speed_msg != nullptr) {
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "last_arcade_speed_msg: l=%.2f, r=%.2f", last_arcade_speed_msg->l, last_arcade_speed_msg->r);
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "last_motor_speed_msg: m1=%.2f, m2=%.2f, m3=%.2f, m4=%.2f, m5=%.2f, m6=%.2f", 
                    last_motor_speed_msg->m1, last_motor_speed_msg->m2, last_motor_speed_msg->m3, last_motor_speed_msg->m4, last_motor_speed_msg->m5, last_motor_speed_msg->m6);
                
                if (std::abs(last_arcade_speed_msg->r - expected_r) < TOL && std::abs(last_arcade_speed_msg->l - expected_l) < TOL 
                    && std::abs(last_motor_speed_msg->m1 - 0.8 * expected_l) < TOL && std::abs(last_motor_speed_msg->m2 - expected_l) < TOL 
                    && std::abs(last_motor_speed_msg->m3 - expected_l) < TOL && std::abs(last_motor_speed_msg->m4 - 0.8 * expected_r) < TOL 
                    && std::abs(last_motor_speed_msg->m5 - expected_r) < TOL && std::abs(last_motor_speed_msg->m6 - expected_r) < TOL ) {
                    return true;
                }
            }
            
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                RCLCPP_ERROR(rclcpp::get_logger("TestArcadeDriver"), "Timed out waiting for expected speed messages");
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Expected arcade speed message: l=%.2f, r=%.2f", expected_l, expected_r);
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Received arcade speed message: l=%.2f, r=%.2f", last_arcade_speed_msg->l, last_arcade_speed_msg->r);
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Expected motor speed message: m1=%.2f, m2=%.2f, m3=%.2f, m4=%.2f, m5=%.2f, m6=%.2f", 
                    0.8 * expected_l, expected_l, expected_l, 0.8 * expected_r, expected_r, expected_r);
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Received motor speed message: m1=%.2f, m2=%.2f, m3=%.2f, m4=%.2f, m5=%.2f, m6=%.2f", 
                    last_motor_speed_msg->m1, last_motor_speed_msg->m2, last_motor_speed_msg->m3, last_motor_speed_msg->m4, last_motor_speed_msg->m5, last_motor_speed_msg->m6);
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void assert_speeds(float joystick_rotate, float joystick_drive, float expected_l, float expected_r) {
        publish_joystick_input(joystick_rotate, joystick_drive);
        ASSERT_TRUE(wait_for_speed_messages(expected_l, expected_r));
        ASSERT_NE(last_arcade_speed_msg, nullptr);
        
        EXPECT_NEAR(last_arcade_speed_msg->l, expected_l, TOL);
        EXPECT_NEAR(last_arcade_speed_msg->r, expected_r, TOL);
        EXPECT_NEAR(last_motor_speed_msg->m1, 0.8 * expected_l, TOL);
        EXPECT_NEAR(last_motor_speed_msg->m2, expected_l, TOL);
        EXPECT_NEAR(last_motor_speed_msg->m3, expected_l, TOL);
        EXPECT_NEAR(last_motor_speed_msg->m4, 0.8 * expected_r, TOL);
        EXPECT_NEAR(last_motor_speed_msg->m5, expected_r, TOL);
        EXPECT_NEAR(last_motor_speed_msg->m6, expected_r, TOL);
    }
};

std::shared_ptr<rclcpp::Node> TestArcadeDriver::test_node;
std::shared_ptr<composition::ArcadeDriver> TestArcadeDriver::arcade_driver;
std::shared_ptr<composition::MotorSpeedController> TestArcadeDriver::motor_speed_controller;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr TestArcadeDriver::joystick_pub;
rclcpp::Subscription<motor_controller::msg::ArcadeSpeed>::SharedPtr TestArcadeDriver::arcade_speed_sub;
motor_controller::msg::ArcadeSpeed::SharedPtr TestArcadeDriver::last_arcade_speed_msg;
rclcpp::Subscription<motor_controller::msg::MotorSpeeds>::SharedPtr TestArcadeDriver::motor_speed_sub;
motor_controller::msg::MotorSpeeds::SharedPtr TestArcadeDriver::last_motor_speed_msg;

// Joystick movement away from rest state is beneath the threshold
TEST_F(TestArcadeDriver, Test0) {
    float joystick_rotate = 0.01;
    float joystick_drive = -0.02;
    float expected_l = 0; // arcade speed should still be 0 because joystick did not exceed threshold
    float expected_r = 0;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test1) {    
    float joystick_rotate = 0.0;
    float joystick_drive = 1.0;
    float expected_l = 1.0;
    float expected_r = 1.0;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test2) {
    float joystick_rotate = 0.0;
    float joystick_drive = 0 - 1.0;
    float expected_l = 0 - 1.0;
    float expected_r = 0 - 1.0;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test3) {
    float joystick_rotate_0 = 0.5;
    float joystick_drive_0 = 0.5;
    float expected_l_0 = 0.5;
    float expected_r_0 = 0;
    assert_speeds(joystick_rotate_0, joystick_drive_0, expected_l_0, expected_r_0);

    float joystick_rotate = 0.51;
    float joystick_drive = 0.49;
    float expected_l_1 = 0.51;
    float expected_r_1 = -0.02;
    assert_speeds(joystick_rotate, joystick_drive, expected_l_1, expected_r_1);
}

TEST_F(TestArcadeDriver, Test4) {
    float joystick_rotate = 0.77;
    float joystick_drive = 0.35;
    float expected_l = 0.77;
    float expected_r = -0.42;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test5) {
    float joystick_rotate = -0.67;
    float joystick_drive = 0.2;
    float expected_l = -0.47;
    float expected_r = 0.67;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test6) {
    float joystick_rotate = -0.8;
    float joystick_drive = -0.77;
    float expected_l = -0.8;
    float expected_r = 0.03;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test7) {
    float joystick_rotate = 0.12;
    float joystick_drive = -0.68;
    float expected_l = -0.56;
    float expected_r = -0.68;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test8) {
    float joystick_rotate = 0.8;
    float joystick_drive = 0;
    float expected_l = 0.8;
    float expected_r = -0.8;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test9) {
    float joystick_rotate = -0.12;
    float joystick_drive = 0;
    float expected_l = -0.12;
    float expected_r = 0.12;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

TEST_F(TestArcadeDriver, Test10) {
    float joystick_rotate = -0.18;
    float joystick_drive = 0.1;
    float expected_l = -0.08;
    float expected_r = 0.18;
    assert_speeds(joystick_rotate, joystick_drive, expected_l, expected_r);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}