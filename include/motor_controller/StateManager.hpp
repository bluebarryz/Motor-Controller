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
    void handle_change_state(const std::shared_ptr<motor_controller::srv::ChangeState::Request> request,
        std::shared_ptr<motor_controller::srv::ChangeState::Response> response);
    void handle_get_state(const std::shared_ptr<motor_controller::srv::GetState::Request> request,
        std::shared_ptr<motor_controller::srv::GetState::Response> response);

    // must only be called while holding the mutex
    bool try_transition(uint8_t transition_id);
    TransitionCallbackReturn execute_callback(
        std::function<TransitionCallbackReturn(const uint8_t transition_id)> callback, uint8_t transition_id);

    std::string state_to_string(uint8_t state);
    static const std::map<std::pair<uint8_t, uint8_t>, uint8_t> transition_map_;
    static const std::map<std::uint8_t, std::uint8_t> transition_fall_back_state_map_;
    static const std::map<std::uint8_t, std::uint8_t> transition_error_transition_map_;
    std::map<
        std::uint8_t,
        std::pair<uint8_t, std::function<TransitionCallbackReturn(const uint8_t transition_id)>>> callback_map_;
    void init_callback_map();

    uint8_t current_state_;
    mutable std::recursive_mutex state_mutex_;

    TransitionCallbackReturn activate_arcade_driver(const uint8_t transition_id);
};

} // namespace composition