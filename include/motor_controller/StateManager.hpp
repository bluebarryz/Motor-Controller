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



using Transition = motor_controller::msg::Transition;
using State = motor_controller::msg::State;


class StateManager : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit StateManager(const rclcpp::NodeOptions &options);
    enum class TransitionCallbackReturn : uint8_t {
        SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS,
        FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE,
        ERROR = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR
    };
private:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &);

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

    TransitionCallbackReturn change_arcade_driver_state(const uint8_t arcade_lifecycle_transition);
    TransitionCallbackReturn pre_calibration(const uint8_t transition_id);
    TransitionCallbackReturn shutdown(const uint8_t transition_id);
};

#endif