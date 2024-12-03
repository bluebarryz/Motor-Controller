#include "motor_controller/StateManager.hpp"

using namespace std::placeholders;

namespace composition {

StateManager::StateManager(const rclcpp::NodeOptions &options)
    : LifecycleNode("state_manager", options), current_state{motor_controller::msg::State::UNINIT} {
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
    RCLCPP_INFO(get_logger(), "calling handle_change_state");

    try {
        std::lock_guard<std::mutex> lock(state_mutex_);
        RCLCPP_INFO(get_logger(), "preparing to call try_transition");
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
    std::lock_guard<std::mutex> lock(state_mutex_);
    (void)request;
    response->current_state.id = current_state;
    response->current_state.label = state_to_string(current_state);
}

bool StateManager::try_transition(uint8_t transition_id) {
    RCLCPP_INFO(get_logger(), "Calling try_transition");

    // check if transition is legal
    auto tr_id = transition_map.find({current_state, transition_id});
    if (tr_id == transition_map.end()) {
        // did not find transition in the transition table
        RCLCPP_WARN(get_logger(), "Invalid transition %d from state %d",
                    transition_id, current_state);
        return false;
    }

    // call associated transition callback
    auto cb_success = StateManager::execute_callback(transition_id);
    if (cb_success == StateManager::TransitionCallbackReturn::ERROR) {
        RCLCPP_ERROR(
            get_logger(),
            "Transition with id %d failed.", transition_id);
    }

    uint8_t new_state = tr_id->second;
    RCLCPP_INFO(get_logger(), "Transitioning from %s to %s",
                state_to_string(current_state).c_str(),
                state_to_string(new_state).c_str());
    
    current_state = new_state;
    return true;
}

StateManager::TransitionCallbackReturn StateManager::execute_callback(uint8_t transition_id) {
    auto tr_cb = callback_map_.find(transition_id);
    auto cb_success = StateManager::TransitionCallbackReturn::SUCCESS;
    if (tr_cb == callback_map_.end()) {
        // did not find callback in the callback_map_
        RCLCPP_WARN(get_logger(), "Could not find callback for transition %d",
                    transition_id);
        cb_success = StateManager::TransitionCallbackReturn::ERROR;
    } else {
        auto callback = tr_cb->second;
        try {
            cb_success = callback(transition_id);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                get_logger(),
                "Caught exception in callback for transition %d", tr_cb->first);
            RCLCPP_ERROR(
                get_logger(),
                "Original error: %s", e.what());
            cb_success = StateManager::TransitionCallbackReturn::ERROR;
        }
    }

    return cb_success;
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
    {{motor_controller::msg::State::UNINIT, motor_controller::msg::Transition::TRANSITION_CALIBRATE},
        motor_controller::msg::State::TRANSITION_STATE_PRE_CAL},
    {{motor_controller::msg::State::TRANSITION_STATE_PRE_CAL, motor_controller::msg::Transition::TRANSITION_ON_CALIBRATE_SUCCESS},
        motor_controller::msg::State::IDLE},
    {{motor_controller::msg::State::TRANSITION_STATE_PRE_CAL, motor_controller::msg::Transition::TRANSITION_ON_CALIBRATE_FAILURE},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},

    {{motor_controller::msg::State::IDLE, motor_controller::msg::Transition::TRANSITION_ACTIVATE_POS_CONTROL},
        motor_controller::msg::State::TRANSITION_STATE_ACTIVATING_POS_CONTROL},
    {{motor_controller::msg::State::TRANSITION_STATE_ACTIVATING_POS_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_POS_CONTROL_SUCCESS},
        motor_controller::msg::State::POS_CONTROL},
    {{motor_controller::msg::State::TRANSITION_STATE_ACTIVATING_POS_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_POS_CONTROL_FAILURE},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},
    {{motor_controller::msg::State::POS_CONTROL, motor_controller::msg::Transition::TRANSITION_POS_CONTROL_ERR},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},

    {{motor_controller::msg::State::POS_CONTROL, motor_controller::msg::Transition::TRANSITION_DEACTIVATE_POS_CONTROL},
        motor_controller::msg::State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL},
    {{motor_controller::msg::State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_POS_CONTROL_SUCCESS},
        motor_controller::msg::State::IDLE},
    {{motor_controller::msg::State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_POS_CONTROL_FAILURE},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},

    {{motor_controller::msg::State::IDLE, motor_controller::msg::Transition::TRANSITION_ACTIVATE_VEL_CONTROL},
        motor_controller::msg::State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL},
    {{motor_controller::msg::State::POS_CONTROL, motor_controller::msg::Transition::TRANSITION_ACTIVATE_VEL_CONTROL},
        motor_controller::msg::State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL},
    {{motor_controller::msg::State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_VEL_CONTROL_SUCCESS},
        motor_controller::msg::State::VEL_CONTROL},
    {{motor_controller::msg::State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_VEL_CONTROL_FAILURE},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},
    {{motor_controller::msg::State::VEL_CONTROL, motor_controller::msg::Transition::TRANSITION_VEL_CONTROL_ERR},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},
        
    {{motor_controller::msg::State::VEL_CONTROL, motor_controller::msg::Transition::TRANSITION_DEACTIVATE_VEL_CONTROL},
        motor_controller::msg::State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL},
    {{motor_controller::msg::State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_VEL_CONTROL_SUCCESS},
        motor_controller::msg::State::IDLE},
    {{motor_controller::msg::State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL, motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_VEL_CONTROL_FAILURE},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},

    {{motor_controller::msg::State::IDLE, motor_controller::msg::Transition::TRANSITION_SHUTDOWN},
        motor_controller::msg::State::TRANSITION_STATE_SHUTTING_DOWN},
    {{motor_controller::msg::State::POS_CONTROL, motor_controller::msg::Transition::TRANSITION_SHUTDOWN},
        motor_controller::msg::State::TRANSITION_STATE_SHUTTING_DOWN},
    {{motor_controller::msg::State::VEL_CONTROL, motor_controller::msg::Transition::TRANSITION_SHUTDOWN},
        motor_controller::msg::State::TRANSITION_STATE_SHUTTING_DOWN},
    {{motor_controller::msg::State::TRANSITION_STATE_SHUTTING_DOWN, motor_controller::msg::Transition::TRANSITION_ON_SHUTDOWN_SUCCESS},
        motor_controller::msg::State::FINALIZED},
    {{motor_controller::msg::State::TRANSITION_STATE_SHUTTING_DOWN, motor_controller::msg::Transition::TRANSITION_ON_SHUTDOWN_FAILURE},
        motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING},

    {{motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING, motor_controller::msg::Transition::TRANSITION_ERR_HANDLER_SUCCESS},
        motor_controller::msg::State::UNINIT},
    {{motor_controller::msg::State::TRANSITION_STATE_ERR_PROCESSING, motor_controller::msg::Transition::TRANSITION_ERR_HANDLER_FAILURE},
        motor_controller::msg::State::FINALIZED}
};

const std::map<
        std::uint8_t,
        std::function<StateManager::TransitionCallbackReturn(const uint8_t transition_id)>> StateManager::callback_map_ = {
    
    {motor_controller::msg::Transition::TRANSITION_CALIBRATE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_CALIBRATE_SUCCESS, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_CALIBRATE_FAILURE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},


    {motor_controller::msg::Transition::TRANSITION_ACTIVATE_POS_CONTROL, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_POS_CONTROL_SUCCESS, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_POS_CONTROL_FAILURE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_POS_CONTROL_ERR, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},


    {motor_controller::msg::Transition::TRANSITION_DEACTIVATE_POS_CONTROL, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_POS_CONTROL_SUCCESS, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_POS_CONTROL_FAILURE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},


    {motor_controller::msg::Transition::TRANSITION_ACTIVATE_VEL_CONTROL, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_VEL_CONTROL_SUCCESS, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_ACTIVATE_VEL_CONTROL_FAILURE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_VEL_CONTROL_ERR, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},


    {motor_controller::msg::Transition::TRANSITION_DEACTIVATE_VEL_CONTROL, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_VEL_CONTROL_SUCCESS, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_DEACTIVATE_VEL_CONTROL_FAILURE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},


    {motor_controller::msg::Transition::TRANSITION_SHUTDOWN, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_SHUTDOWN_SUCCESS, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ON_SHUTDOWN_FAILURE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},


    {motor_controller::msg::Transition::TRANSITION_ERR_HANDLER_SUCCESS, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }},
    {motor_controller::msg::Transition::TRANSITION_ERR_HANDLER_FAILURE, [](const uint8_t transition_id) {
        (void)transition_id;
        return StateManager::TransitionCallbackReturn::SUCCESS;
    }}
};

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::StateManager)