#ifndef STATE_MANAGER_HPP_
#define STATE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "motor_controller/msg/state.hpp"
#include "motor_controller/msg/transition.hpp"
#include "motor_controller/srv/change_state.hpp"
#include "motor_controller/srv/get_state.hpp"
#include <mutex>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include "rclcpp/wait_for_message.hpp"

using Transition = motor_controller::msg::Transition;
using State = motor_controller::msg::State;

class StateManager : public rclcpp::Node {
public:
    explicit StateManager(const rclcpp::NodeOptions &options);
    enum class TransitionCallbackReturn : uint8_t {
        SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS,
        FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE,
        ERROR = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR
    };
private:
    rclcpp::Service<motor_controller::srv::ChangeState>::SharedPtr change_state_server;
    rclcpp::Service<motor_controller::srv::GetState>::SharedPtr get_state_server;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr arcade_driver_lifecycle_client;
    void handle_change_state(const std::shared_ptr<motor_controller::srv::ChangeState::Request> request,
        std::shared_ptr<motor_controller::srv::ChangeState::Response> response);
    void handle_get_state(const std::shared_ptr<motor_controller::srv::GetState::Request> request,
        std::shared_ptr<motor_controller::srv::GetState::Response> response);


    // transition_map_ maps a pair containing {a state and a transition from that state} to the next state.
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> transition_map_;
    void init_transition_map();

    // callback_map_ maps a transition id to a callback to execute for this transition
    std::unordered_map<uint8_t, std::function<TransitionCallbackReturn(const uint8_t t_id)>> callback_map_;
    void init_callback_map();

    // predicate_map_ maps a transition id to a predicate function
    std::unordered_map<uint8_t, std::function<bool(const uint8_t t_id)>> predicate_map_;
    void init_predicate_map();

    // transition_type_map_ maps each transition id to a Transition Type
    std::unordered_map<uint8_t, uint8_t> transition_type_map_;
    void init_transition_type_map();

    uint8_t current_state_;
    mutable std::recursive_mutex state_mutex_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr odrive_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odrive_pub;
    nlohmann::json json_msg_;
    void odrive_sub_callback(const std_msgs::msg::String::SharedPtr odrive_response);


    TransitionCallbackReturn change_arcade_driver_state(const uint8_t arcade_lifecycle_transition);
    TransitionCallbackReturn pre_calibration(const uint8_t transition_id);
    TransitionCallbackReturn shutdown(const uint8_t transition_id);

    void publish_odrive_request(const nlohmann::json& req_json);
    const nlohmann::json odrive_req1_json = {
            {"Stage", "Calibration"},
            {"Type", "request"},
            {"Target", "Drivetrain"},
            {"Command", "Set_Axis_State"},
            {"Payload", {
                {"1", "FULL_CALIBRATION_SEQUENCE"},
                {"2", "FULL_CALIBRATION_SEQUENCE"},
                {"3", "FULL_CALIBRATION_SEQUENCE"},
                {"4", "FULL_CALIBRATION_SEQUENCE"},
                {"5", "FULL_CALIBRATION_SEQUENCE"},
                {"6", "FULL_CALIBRATION_SEQUENCE"}
            }}
    };

    const nlohmann::json odrive_req2_json = {
        {"Stage", "Calibration"},
        {"Type", "request"},
        {"Target", "Drivetrain"},
        {"Command", "Set_Axis_State"},
        {"Payload", {
            {"1", "CLOSED_LOOP_CONTROL"},
            {"2", "CLOSED_LOOP_CONTROL"},
            {"3", "CLOSED_LOOP_CONTROL"},
            {"4", "CLOSED_LOOP_CONTROL"},
            {"5", "CLOSED_LOOP_CONTROL"},
            {"6", "CLOSED_LOOP_CONTROL"}
        }}
    };

    const nlohmann::json odrive_req3_json = {
        {"Stage", "Calibration"},
        {"Type", "request"},
        {"Target", "Drivetrain"},
        {"Command", "Set_Controller_Mode"},
        {"Payload", {
            {"1", {{"control_mode", "VELOCITY_CONTROL"}, {"input_mode", "VEL_RAMP"}}},
            {"2", {{"control_mode", "VELOCITY_CONTROL"}, {"input_mode", "VEL_RAMP"}}},
            {"3", {{"control_mode", "VELOCITY_CONTROL"}, {"input_mode", "VEL_RAMP"}}},
            {"4", {{"control_mode", "VELOCITY_CONTROL"}, {"input_mode", "VEL_RAMP"}}},
            {"5", {{"control_mode", "VELOCITY_CONTROL"}, {"input_mode", "VEL_RAMP"}}},
            {"6", {{"control_mode", "VELOCITY_CONTROL"}, {"input_mode", "VEL_RAMP"}}}
        }}
    };
};

#endif