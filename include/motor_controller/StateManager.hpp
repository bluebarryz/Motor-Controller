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
    enum class State : uint8_t {
        UNINIT = 0,
        TRANSITION_STATE_PRE_CAL,
        IDLE,
        TRANSITION_STATE_ACTIVATING_POS_CONTROL,
        POS_CONTROL,
        TRANSITION_STATE_DEACTIVATING_POS_CONTROL,
        TRANSITION_STATE_ACTIVATING_VEL_CONTROL,
        VEL_CONTROL,
        TRANSITION_STATE_DEACTIVATING_VEL_CONTROL,
        TRANSITION_STATE_SHUTTING_DOWN,
        FINALIZED,
        TRANSITION_STATE_ERR_PROCESSING
    };

    enum class Transition : uint8_t {
        TRANSITION_CALIBRATE = 4,
        TRANSITION_CALIBRATE_COMPLETE = 2,
        TRANSITION_ACTIVATE_POS_CONTROL = 4,
        TRANSITION_ACTIVATE_VEL_CONTROL = 6,
        TRANSITION_SHUTDOWN = 8,
        TRANSITION_ACTIVATE_POS_CONTROL_COMPLETE = 10,
        TRANSITION_ACTIVATE_VEL_CONTROL_COMPLETE = 12,
        TRANSITION_SHUTDOWN_COMPLETE = 14,
        TRANSITION_DEACTIVATE_POS_CONTROL = 16,
        TRANSITION_DEACTIVATE_VEL_CONTROL = 18,
        TRANSITION_DEACTIVATE_POS_CONTROL_COMPLETE = 20,
        TRANSITION_DEACTIVATE_VEL_CONTROL_COMPLETE = 22,
        TRANSITION_CALIBRATE_ERROR = 99,
        TRANSITION_ACTIVATE_POS_CONTROL_ERROR = 99,
        TRANSITION_DEACTIVATE_POS_CONTROL_ERROR = 99,
        TRANSITION_ACTIVATE_VEL_CONTROL_ERROR = 99,
        TRANSITION_DEACTIVATE_VEL_CONTROL_ERROR = 99,
        TRANSITION_SHUTDOWN_ERROR = 99
    };
    enum class TransitionType : uint8_t {
        TYPE_A = 1,
        TYPE_B = 2,
        TYPE_C = 3
    };
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


    // transition_map maps a pair containing {a state and a transition from that state} to the next state.
    static const std::map<std::pair<State, Transition>, State>& get_transition_map() {
        static const auto transition_map = init_transition_map();
        return transition_map;
    }
    static std::map<std::pair<State, Transition>, State> init_transition_map();

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

    // transition_type_map maps each transition id to a Transition Type
    static const std::unordered_map<Transition, TransitionType>& get_transition_type_map() {
        static const auto transition_type_map = init_transition_type_map();
        return transition_type_map;
    }
    static std::unordered_map<Transition, TransitionType> init_transition_type_map();

    State current_state_;
    mutable std::recursive_mutex state_mutex_;

    TransitionCallbackReturn activate_arcade_driver(const uint8_t transition_id);
};

} // namespace composition