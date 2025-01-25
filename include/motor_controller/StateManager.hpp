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


namespace composition {

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

    bool is_legal_transition(const Transition& transition_id);

    // transition_map maps a pair containing {a state and a transition from that state} to the next state.
    static const std::unordered_map<std::pair<State, Transition>, State>& get_transition_map() {
        static const auto transition_map = init_transition_map();
        return transition_map;
    }
    static std::unordered_map<std::pair<State, Transition>, State> init_transition_map();

    // callback_map maps a transition id to a callback to execute for this transition
    static const std::unordered_map<Transition, std::function<TransitionCallbackReturn(const Transition&)>>& get_callback_map() {
        static const auto callback_map = init_callback_map();
        return callback_map;
    }
    static std::unordered_map<Transition, std::function<TransitionCallbackReturn(const Transition&)>> init_callback_map();

    // predicate_map maps a transition id to a predicate function
    static const std::unordered_map<Transition, std::function<bool(const Transition&)>>& get_predicate_map() {
        static const auto predicate_map = init_predicate_map();
        return predicate_map;
    }
    static std::unordered_map<Transition, std::function<bool(const Transition&)>> init_predicate_map();

    uint8_t current_state_;
    mutable std::recursive_mutex state_mutex_;

    TransitionCallbackReturn activate_arcade_driver(const uint8_t transition_id);
};

} // namespace composition