#include "motor_controller/StateManager.hpp"

using namespace std::placeholders;

namespace composition {

StateManager::StateManager(const rclcpp::NodeOptions &options)
    : LifecycleNode("state_manager", options) {
    RCLCPP_INFO(get_logger(), "Initializing StateManager lifecycle");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManager::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Configuring StateManager");
    
    change_state_server = create_service<motor_controller::srv::ChangeState>(
        "~/change_mc_state",
        std::bind(&StateManager::handle_change_state, this, _1, _2));
    
    get_state_server = create_service<motor_controller::srv::GetState>(
        "~/get_mc_state",
        std::bind(&StateManager::handle_get_state, this, _1, _2));

    if (!change_state_server || !get_state_server) {
        RCLCPP_ERROR(get_logger(), "Failed to create services");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "StateManager configured successfully");
    return CallbackReturn::SUCCESS;
}

void StateManager::handle_change_state(const std::shared_ptr<motor_controller::srv::ChangeState::Request> request,
    std::shared_ptr<motor_controller::srv::ChangeState::Response> response) {
    try {
        bool transition_success = try_transition(request->transition.id);
        response->success = transition_success;
        if (!transition_success) {
            // TODO
        }
    } catch (const std::exception& e) {
        response->success = false;
        // TODO
    }
    
}

void StateManager::handle_get_state(const std::shared_ptr<motor_controller::srv::GetState::Request> request,
        std::shared_ptr<motor_controller::srv::GetState::Response> response) {
    (void)request;
    response->current_state.id = current_state;
    response->current_state.label = state_to_string(current_state);
}

bool StateManager::try_transition(uint8_t transition_id) {
    auto it = transition_map.find({current_state, transition_id});
    if (it == transition_map.end()) {
        // did not find transition in the transition table
        RCLCPP_WARN(get_logger(), "Invalid transition %d from state %d",
                    transition_id, current_state);
        return false;
    }

    uint8_t new_state = it->second;
    RCLCPP_INFO(get_logger(), "Transitioning from %s to %s",
                state_to_string(current_state).c_str(),
                state_to_string(new_state).c_str());
    
    current_state = new_state;
    return true;
}

// TODO: do this for Transitions too
std::string StateManager::state_to_string(uint8_t state) {
    switch (state) {
        case motor_controller::msg::State::UNINIT: 
            return "UNINIT";
        case motor_controller::msg::State::TRANSITION_STATE_PRE_CAL: 
            return "PRE_CAL";
        case motor_controller::msg::State::IDLE: 
            return "IDLE";
        case motor_controller::msg::State::POS_CONTROL: 
            return "POS_CONTROL";
        case motor_controller::msg::State::VEL_CONTROL: 
            return "VEL_CONTROL";
        case motor_controller::msg::State::TRANSITION_STATE_SHUTTING_DOWN: 
            return "SHUTTING_DOWN";
        case motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING: 
            return "ERROR_PROCESSING";
        case motor_controller::msg::State::FINALIZED: 
            return "FINALIZED";
        default: 
            return "UNKNOWN";
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManager::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating StateManager");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManager::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating StateManager");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManager::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Cleaning up StateManager");
    
    change_state_server.reset();
    get_state_server.reset();
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManager::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Shutting down StateManager");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

const std::map<std::pair<uint8_t, uint8_t>, uint8_t> StateManager::transition_map = {
    // From UNINIT
    {{motor_controller::msg::State::UNINIT, motor_controller::msg::Transition::CONFIGURE},
     motor_controller::msg::State::TRANSITION_STATE_PRE_CAL},
    
    // From PRE_CAL
    {{motor_controller::msg::State::TRANSITION_STATE_PRE_CAL, motor_controller::msg::Transition::CALIBRATION_SUCCESS},
     motor_controller::msg::State::IDLE},
    {{motor_controller::msg::State::TRANSITION_STATE_PRE_CAL, motor_controller::msg::Transition::CALIBRATION_FAILURE},
     motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},
    
    // From IDLE
    {{motor_controller::msg::State::IDLE, motor_controller::msg::Transition::ACTIVATE_POS},
     motor_controller::msg::State::POS_CONTROL},
    {{motor_controller::msg::State::IDLE, motor_controller::msg::Transition::ACTIVATE_VEL},
     motor_controller::msg::State::VEL_CONTROL},
    
    // From POS_CONTROL
    {{motor_controller::msg::State::POS_CONTROL, motor_controller::msg::Transition::DEACTIVATE},
     motor_controller::msg::State::IDLE},
    {{motor_controller::msg::State::POS_CONTROL, motor_controller::msg::Transition::ACTIVATE_VEL},
     motor_controller::msg::State::VEL_CONTROL},
    
    // From VEL_CONTROL
    {{motor_controller::msg::State::VEL_CONTROL, motor_controller::msg::Transition::DEACTIVATE},
     motor_controller::msg::State::IDLE},
    
    // Error handling from any state
    {{motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING, motor_controller::msg::Transition::ERROR_RESOLVED},
     motor_controller::msg::State::IDLE},
};

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::StateManager)