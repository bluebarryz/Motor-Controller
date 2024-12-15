#include "motor_controller/StateManager.hpp"

using namespace motor_controller::msg;
namespace composition {

StateManager::StateManager(const rclcpp::NodeOptions &options)
    : LifecycleNode("state_manager", options), current_state_{motor_controller::msg::State::UNINIT} {
    RCLCPP_INFO(get_logger(), "Initializing StateManager lifecycle");

    init_callback_map();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManager::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Configuring StateManager");
    
    change_state_server = create_service<motor_controller::srv::ChangeState>(
        "~/change_mc_state",
        std::bind(&StateManager::handle_change_state, this, std::placeholders::_1, std::placeholders::_2));
    
    get_state_server = create_service<motor_controller::srv::GetState>(
        "~/get_mc_state",
        std::bind(&StateManager::handle_get_state, this, std::placeholders::_1, std::placeholders::_2));

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
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    (void)request;
    response->current_state.id = current_state_;
    response->current_state.label = state_to_string(current_state_);
}

bool StateManager::try_transition(uint8_t transition_id) {
    RCLCPP_INFO(get_logger(), "Calling try_transition");

    // check if transition is legal
    auto tr_id = transition_map_.find({current_state_, transition_id});
    if (tr_id == transition_map_.end()) {
        // did not find transition in the transition table
        RCLCPP_WARN(get_logger(), "Invalid transition %d from state %d",
                    transition_id, current_state_);
        return false;
    }

    {
        // set current_state_ to be the transitional state
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        current_state_ = tr_id->second;
    }
    
    // search for the associated transition callback and destination primary state
    auto tr_cb = callback_map_.find(transition_id);
    if (tr_cb == callback_map_.end()) {
        RCLCPP_ERROR(
            get_logger(),
            "No callback associated with transition %d", transition_id);
    }
    auto callback = tr_cb->second.second;
    auto cb_success = StateManager::execute_callback(callback, transition_id);
    
    if (cb_success == StateManager::TransitionCallbackReturn::SUCCESS) {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        current_state_ = tr_cb->second.first;
    } else if (cb_success == StateManager::TransitionCallbackReturn::ERROR) {
        auto tr_err = transition_error_transition_map_.find(current_state_);
        if (tr_err == transition_error_transition_map_.end()) {
            RCLCPP_ERROR(
                get_logger(),
                "No error transition associated with state %d", current_state_);
        }
        StateManager::try_transition(tr_err->second);
    } else if (cb_success == StateManager::TransitionCallbackReturn::FAILURE) {
        auto tr_fail = transition_fall_back_state_map_.find(current_state_);
        if (tr_fail == transition_fall_back_state_map_.end()) {
            RCLCPP_ERROR(
                get_logger(),
                "No fallback state associated with state %d", current_state_);
        }
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        current_state_ = tr_fail->second;
    }

    return true;
}

StateManager::TransitionCallbackReturn StateManager::execute_callback(
    std::function<TransitionCallbackReturn(const uint8_t transition_id)> callback, uint8_t transition_id) {
    
    auto cb_success = StateManager::TransitionCallbackReturn::SUCCESS;
   
    try {
        cb_success = callback(transition_id);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(
            get_logger(),
            "Caught exception in callback for transition %d", transition_id);
        RCLCPP_ERROR(
            get_logger(),
            "Original error: %s", e.what());
        cb_success = StateManager::TransitionCallbackReturn::ERROR;
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


const std::map<std::pair<uint8_t, uint8_t>, uint8_t> StateManager::transition_map_ = {
    {{State::UNINIT, Transition::TRANSITION_CALIBRATE},
        State::TRANSITION_STATE_PRE_CAL},
    {{State::TRANSITION_STATE_PRE_CAL, Transition::TRANSITION_ON_CALIBRATE_ERROR},
        State::TRANSITION_STATE_ERR_PROCESSING},

    {{State::IDLE, Transition::TRANSITION_ACTIVATE_POS_CONTROL},
        State::TRANSITION_STATE_ACTIVATING_POS_CONTROL},
    {{State::TRANSITION_STATE_ACTIVATING_POS_CONTROL, Transition::TRANSITION_ON_ACTIVATE_POS_CONTROL_ERROR},
        State::TRANSITION_STATE_ERR_PROCESSING},

    {{State::POS_CONTROL, Transition::TRANSITION_DEACTIVATE_POS_CONTROL},
        State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL},
    {{State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL, Transition::TRANSITION_ON_DEACTIVATE_POS_CONTROL_ERROR},
        State::TRANSITION_STATE_ERR_PROCESSING},

    {{State::IDLE, Transition::TRANSITION_ACTIVATE_VEL_CONTROL},
        State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL},
    {{State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL, Transition::TRANSITION_ON_ACTIVATE_VEL_CONTROL_ERROR},
        State::TRANSITION_STATE_ERR_PROCESSING},

    {{State::VEL_CONTROL, Transition::TRANSITION_DEACTIVATE_VEL_CONTROL},
        State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL},
    {{State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL, Transition::TRANSITION_ON_DEACTIVATE_VEL_CONTROL_ERROR},
        State::TRANSITION_STATE_ERR_PROCESSING},

    {{State::IDLE, Transition::TRANSITION_SHUTDOWN},
        State::TRANSITION_STATE_SHUTTING_DOWN},
    {{State::POS_CONTROL, Transition::TRANSITION_SHUTDOWN},
        State::TRANSITION_STATE_SHUTTING_DOWN},
    {{State::VEL_CONTROL, Transition::TRANSITION_SHUTDOWN},
        State::TRANSITION_STATE_SHUTTING_DOWN},
    {{State::TRANSITION_STATE_SHUTTING_DOWN, Transition::TRANSITION_ON_SHUTDOWN_ERROR},
        State::TRANSITION_STATE_ERR_PROCESSING}
};

StateManager::TransitionCallbackReturn StateManager::activate_arcade_driver(const uint8_t transition_id) {
    (void)transition_id;
    auto lifecycle_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/arcade_driver/change_state");
    
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

    while (!lifecycle_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for lifecycle service. Exiting.");
            return StateManager::TransitionCallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    auto future = lifecycle_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(get_logger(), "Successfully activated ArcadeDriver");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to activate ArcadeDriver");
            return StateManager::TransitionCallbackReturn::ERROR;
        }
    }

    return TransitionCallbackReturn::SUCCESS;
}

void StateManager::init_callback_map() {
    callback_map_ = {
        // From UNINIT to TRANSITION_STATE_PRE_CAL to IDLE
        {Transition::TRANSITION_CALIBRATE, {State::IDLE, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},
        // From TRANSITION_STATE_PRE_CAL to TRANSITION_STATE_ERR_PROCESSING to UNINIT
        {Transition::TRANSITION_ON_CALIBRATE_ERROR, {State::UNINIT, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},
        

        // From IDLE to TRANSITION_STATE_ACTIVATING_POS_CONTROL to POS_CONTROL
        {Transition::TRANSITION_ACTIVATE_POS_CONTROL, {State::POS_CONTROL, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},
        // From TRANSITION_STATE_ACTIVATING_POS_CONTROL to TRANSITION_STATE_ERR_PROCESSING to UNINIT
        {Transition::TRANSITION_ON_ACTIVATE_POS_CONTROL_ERROR, {State::UNINIT, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},


        // From POS_CONTROL to TRANSITION_STATE_DEACTIVATING_POS_CONTROL to IDLE
        {Transition::TRANSITION_DEACTIVATE_POS_CONTROL, {State::IDLE, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},
        // From TRANSITION_STATE_DEACTIVATING_POS_CONTROL to TRANSITION_STATE_ERR_PROCESSING to UNINIT
        {Transition::TRANSITION_ON_DEACTIVATE_POS_CONTROL_ERROR, {State::UNINIT, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},


        // From IDLE to TRANSITION_STATE_ACTIVATING_VEL_CONTROL to VEL_CONTROL
        {Transition::TRANSITION_ACTIVATE_VEL_CONTROL, {State::VEL_CONTROL, 
            std::bind(&StateManager::activate_arcade_driver, this, std::placeholders::_1)}},

        // From TRANSITION_STATE_ACTIVATING_VEL_CONTROL to TRANSITION_STATE_ERR_PROCESSING to UNINIT
        {Transition::TRANSITION_ON_ACTIVATE_VEL_CONTROL_ERROR, {State::UNINIT, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},


        // From VEL_CONTROL to TRANSITION_STATE_DEACTIVATING_VEL_CONTROL to IDLE
        {Transition::TRANSITION_DEACTIVATE_VEL_CONTROL, {State::IDLE, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},
        // From TRANSITION_STATE_DEACTIVATING_VEL_CONTROL to TRANSITION_STATE_ERR_PROCESSING to UNINIT
        {Transition::TRANSITION_ON_DEACTIVATE_VEL_CONTROL_ERROR, {State::UNINIT, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},


        // From IDLE/POS_CONTROL/VEL_CONTROL to TRANSITION_STATE_SHUTTING_DOWN to FINALIZED
        {Transition::TRANSITION_SHUTDOWN, {State::FINALIZED, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},
        // From TRANSITION_STATE_SHUTTING_DOWN to TRANSITION_STATE_ERR_PROCESSING to UNINIT
        {Transition::TRANSITION_ON_SHUTDOWN_ERROR, {State::UNINIT, [](const uint8_t transition_id) {
            (void)transition_id;
            return StateManager::TransitionCallbackReturn::SUCCESS;
        }}},
    };
}


const std::map<std::uint8_t, std::uint8_t> StateManager::transition_fall_back_state_map_ = {
    {State::TRANSITION_STATE_PRE_CAL, State::UNINIT},
    {State::TRANSITION_STATE_ACTIVATING_POS_CONTROL, State::IDLE},
    {State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL, State::IDLE},
    {State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL, State::POS_CONTROL},
    {State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL, State::VEL_CONTROL},
    {State::TRANSITION_STATE_ERR_PROCESSING, State::FINALIZED}
};

const std::map<std::uint8_t, std::uint8_t> StateManager::transition_error_transition_map_ = {
    {State::TRANSITION_STATE_PRE_CAL, Transition::TRANSITION_ON_CALIBRATE_ERROR},
    {State::TRANSITION_STATE_ACTIVATING_POS_CONTROL, Transition::TRANSITION_ON_ACTIVATE_POS_CONTROL_ERROR},
    {State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL, Transition::TRANSITION_ON_ACTIVATE_VEL_CONTROL_ERROR},
    {State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL, Transition::TRANSITION_ON_DEACTIVATE_POS_CONTROL_ERROR},
    {State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL, Transition::TRANSITION_ON_DEACTIVATE_VEL_CONTROL_ERROR},
    {State::TRANSITION_STATE_SHUTTING_DOWN, Transition::TRANSITION_ON_SHUTDOWN_ERROR}
};

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::StateManager)