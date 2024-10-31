#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arcade_control/ArcadeDriver.hpp"
#include "arcade_control/srv/joystick_input.hpp"
#include "arcade_control/msg/speed.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TestArcadeDriver : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        test_node = std::make_shared<rclcpp::Node>("test_node");
        
        arcade_driverr = std::make_shared<composition::ArcadeDriver>(rclcpp::NodeOptions());

        last_speed_msg = std::make_shared<arcade_control::msg::ArcadeSpeed>();
        last_speed_msg->l = last_speed_msg->r = 0;
        speed_sub = test_node->create_subscription<arcade_control::msg::ArcadeSpeed>(
            "/cmd_vel",
            10,
            [this](const arcade_control::msg::ArcadeSpeed::SharedPtr msg) {
                RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Message: Left speed %.2f, Right speed %.2f", msg->l, msg->r);
                last_speed_msg = msg;
            });

        client = test_node->create_client<arcade_control::srv::JoystickInput>("/joystick_input");
        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Setup completed");
    }

    void TearDown() override {
        arcade_driverr.reset();
        test_node.reset();
        rclcpp::shutdown();
        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Teardown completed");
    }

    std::shared_ptr<rclcpp::Node> test_node;
    std::shared_ptr<composition::ArcadeDriver> arcade_driverr;
    rclcpp::Client<arcade_control::srv::JoystickInput>::SharedPtr client;
    rclcpp::Subscription<arcade_control::msg::ArcadeSpeed>::SharedPtr speed_sub;
    arcade_control::msg::ArcadeSpeed::SharedPtr last_speed_msg;
    const float TOL = 0.0001;

    bool send_joystick_input(float rotate, float drive) {
        auto request = std::make_shared<arcade_control::srv::JoystickInput::Request>();
        request->x = rotate;
        request->y = drive;

        if (!client->wait_for_service(1s)) {
            RCLCPP_ERROR(rclcpp::get_logger("TestArcadeDriver"), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
            return false;
        }

        auto result = client->async_send_request(request);
        RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "Sent request: x = %.2f, y = %.2f", request->x, request->y);

        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(test_node);
        executor->add_node(arcade_driverr);

        // Wait for result
        if (executor->spin_until_future_complete(result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("TestArcadeDriver"), "successful request!");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("TestArcadeDriver"), "Failed to call service");
            rclcpp::shutdown();
            return false;
        }

        return result.get()->success;
    }
    
    // Wait for the /cmd_vel subscriber to receive the expected speed values
    bool wait_for_speed_message(float expected_l, float expected_r, const std::chrono::seconds timeout = std::chrono::seconds(3)) {
        auto start_time = std::chrono::steady_clock::now();
        
        while (true) {
            rclcpp::spin_some(test_node);
            rclcpp::spin_some(arcade_driverr);
            
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
        ASSERT_TRUE(send_joystick_input(joystick_rotate, joystick_drive));
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

    // ArcadeSpeed should remain the same if joystick deltas are less than threshold
    float joystick_rotate = 0.51;
    float joystick_drive = 0.49;
    assert_motor_speed(joystick_rotate, joystick_drive, expected_l_0, expected_r_0);
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