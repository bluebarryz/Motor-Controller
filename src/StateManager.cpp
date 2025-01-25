#include "motor_controller/StateManager.hpp"

// using namespace motor_controller::msg;

namespace composition {

using Transition = motor_controller::msg::Transition;
using State = motor_controller::msg::State;

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

    arcade_driver_lifecycle_client = create_client<lifecycle_msgs::srv::ChangeState>("/arcade_driver/change_state");

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

    const auto& transition_map = get_transition_map();
    // check if transition is legal
    auto tr = transition_map.find({current_state_, transition_id});
    if (tr == transition_map.end()) {
        // did not find transition in the transition table
        RCLCPP_WARN(get_logger(), "Invalid transition %d from state %d",
                    transition_id, current_state_);
        response->success = false;
    }

    Transition transition = tr->first;
    State next_state = tr->second;
    if (tr_id.type == Transition::TYPE_A) {
        {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            current_state_ = tr_id->second;
        }

        auto callback = get_callback_map().find(transition);
        callback();
    } else if (tr_id.type == Transition::TYPE_B) {
        auto predicate = get_predicate_map().find(transition);
        if (predicate(transition)) {
            {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            current_state_ = tr_id->second;
        }
        }
    } else if (tr_id.type == Transition::TYPE_C) {
        //TODO
    } 
}

std::unordered_map<std::pair<State, Transition>, State> StateManager::init_transition_map() {
    return {
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
    }
}

void StateManager::handle_get_state(const std::shared_ptr<motor_controller::srv::GetState::Request> request,
        std::shared_ptr<motor_controller::srv::GetState::Response> response) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    (void)request;
    response->current_state.id = current_state_;
    response->current_state.label = state_to_string(current_state_);
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