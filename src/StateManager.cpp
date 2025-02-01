#include "motor_controller/StateManager.hpp"

// using namespace motor_controller::msg;

namespace composition {

using TransitionCallbackReturn = StateManager::TransitionCallbackReturn;

StateManager::StateManager(const rclcpp::NodeOptions &options)
    : LifecycleNode("state_manager", options), current_state_{State::UNINIT} {
    RCLCPP_INFO(get_logger(), "Initializing StateManager lifecycle");

    init_transition_map();
    init_callback_map();
    init_predicate_map();
    init_transition_type_map();
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
    RCLCPP_INFO(get_logger(), "calling handle_change_state. Current state: %d. Transition: %d", current_state_, request->transition.id);

    // const auto& transition_map = get_transition_map();
    // check if transition is legal
    uint8_t transition = request->transition.id;
    auto tr = transition_map_.find({current_state_, transition});
    if (tr == transition_map_.end()) {
        // did not find transition in the transition table
        RCLCPP_WARN(get_logger(), "Invalid transition %d from state %d",
                    transition, current_state_);
        response->success = false;
    }

    uint8_t next_state = tr->second;
    uint8_t transition_type = transition_type_map_.at(transition);
    RCLCPP_INFO(get_logger(), "Transition type for transition %d is %d", transition, transition_type);
    if (transition_type == Transition::TYPE_A) {
        {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            current_state_ = next_state;
        }

        auto callback = callback_map_.at(transition);
        callback(transition);
    } else if (transition_type == Transition::TYPE_B) {
        RCLCPP_INFO(get_logger(), "Type B transition.");
        auto predicate = predicate_map_.at(transition);
        if (predicate(transition)) {
            {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            current_state_ = next_state;
        }
        }
    } else if (transition_type == Transition::TYPE_C) {
        //TODO
    } else if (transition_type == Transition::TYPE_D) {
    }

    response->success = true;
}

void StateManager::init_transition_map() {
    transition_map_ = {
        {{State::UNINIT, Transition::TRANSITION_CALIBRATE},
            State::TRANSITION_STATE_PRE_CAL},
        {{State::TRANSITION_STATE_PRE_CAL, Transition::TRANSITION_CALIBRATE_COMPLETE},
            State::IDLE},


        {{State::IDLE, Transition::TRANSITION_ACTIVATE_POS_CONTROL},
            State::TRANSITION_STATE_ACTIVATING_POS_CONTROL},
        {{State::IDLE, Transition::TRANSITION_ACTIVATE_VEL_CONTROL},
            State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL},


        {{State::IDLE, Transition::TRANSITION_SHUTDOWN},
            State::TRANSITION_STATE_SHUTTING_DOWN},
        {{State::POS_CONTROL, Transition::TRANSITION_SHUTDOWN},
            State::TRANSITION_STATE_SHUTTING_DOWN},
        {{State::VEL_CONTROL, Transition::TRANSITION_SHUTDOWN},
            State::TRANSITION_STATE_SHUTTING_DOWN},


        {{State::TRANSITION_STATE_ACTIVATING_POS_CONTROL, Transition::TRANSITION_ACTIVATE_ARCADE_CONTROL_COMPLETE},
            State::POS_CONTROL},
        {{State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL, Transition::TRANSITION_ACTIVATE_ARCADE_CONTROL_COMPLETE},
            State::VEL_CONTROL},
        {{State::TRANSITION_STATE_SHUTTING_DOWN, Transition::TRANSITION_SHUTDOWN_COMPLETE},
            State::FINALIZED},
   

        {{State::POS_CONTROL, Transition::TRANSITION_DEACTIVATE_POS_CONTROL},
            State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL},
        {{State::VEL_CONTROL, Transition::TRANSITION_DEACTIVATE_VEL_CONTROL},
            State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL},

        
        {{State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL, Transition::TRANSITION_DEACTIVATE_ARCADE_CONTROL_COMPLETE},
            State::IDLE},
        {{State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL, Transition::TRANSITION_DEACTIVATE_ARCADE_CONTROL_COMPLETE},
            State::IDLE},
    };
}

void StateManager::init_callback_map() {
    callback_map_ = std::unordered_map<uint8_t, std::function<TransitionCallbackReturn(const uint8_t)>>{
        // From UNINIT to TRANSITION_STATE_PRE_CAL to IDLE
        {Transition::TRANSITION_CALIBRATE, 
            std::bind(&StateManager::pre_calibration, this, std::placeholders::_1)},
        {Transition::TRANSITION_SHUTDOWN, 
            std::bind(&StateManager::shutdown, this, std::placeholders::_1)},    
        {Transition::TRANSITION_ACTIVATE_VEL_CONTROL,
            [this](const uint8_t transition_id) {
                (void)transition_id;
                return this->change_arcade_driver_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            }},
        {Transition::TRANSITION_DEACTIVATE_VEL_CONTROL,
            [this](const uint8_t transition_id) {
                (void)transition_id;
                return this->change_arcade_driver_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            }}
    };
}

void StateManager::init_predicate_map() {
    predicate_map_ = {
        {Transition::TRANSITION_CALIBRATE_COMPLETE, 
            [this](const uint8_t t) { (void) t; return current_state_ == State::TRANSITION_STATE_PRE_CAL; }},
        {Transition::TRANSITION_SHUTDOWN_COMPLETE, 
            [this](const uint8_t t) { (void) t; return current_state_ == State::TRANSITION_STATE_SHUTTING_DOWN; }},
        {Transition::TRANSITION_ACTIVATE_ARCADE_CONTROL_COMPLETE, 
            [this](const uint8_t t) { 
                (void) t; 
                return (current_state_ == State::TRANSITION_STATE_ACTIVATING_VEL_CONTROL || 
                    current_state_ == State::TRANSITION_STATE_ACTIVATING_POS_CONTROL); 
            }},
        {Transition::TRANSITION_DEACTIVATE_ARCADE_CONTROL_COMPLETE, 
            [this](const uint8_t t) { 
                (void) t; 
                return (current_state_ == State::TRANSITION_STATE_DEACTIVATING_VEL_CONTROL || 
                    current_state_ == State::TRANSITION_STATE_DEACTIVATING_POS_CONTROL); 
            }}
    };
}

void StateManager::init_transition_type_map() {
    transition_type_map_ = {
        // TYPE_A transitions
        {Transition::TRANSITION_CALIBRATE, Transition::TYPE_A},
        {Transition::TRANSITION_ACTIVATE_POS_CONTROL, Transition::TYPE_A},
        {Transition::TRANSITION_ACTIVATE_VEL_CONTROL, Transition::TYPE_A},
        {Transition::TRANSITION_SHUTDOWN, Transition::TYPE_A},
        {Transition::TRANSITION_DEACTIVATE_POS_CONTROL, Transition::TYPE_A},
        {Transition::TRANSITION_DEACTIVATE_VEL_CONTROL, Transition::TYPE_A},

        // TYPE_B transitions
        {Transition::TRANSITION_CALIBRATE_COMPLETE, Transition::TYPE_B},
        {Transition::TRANSITION_SHUTDOWN_COMPLETE, Transition::TYPE_B},
        {Transition::TRANSITION_ERR_PROCESSING_COMPLETE, Transition::TYPE_B},
        {Transition::TRANSITION_ERR_PROCESSING_ERROR, Transition::TYPE_B},
        {Transition::TRANSITION_ACTIVATE_ARCADE_CONTROL_COMPLETE, Transition::TYPE_B},
        {Transition::TRANSITION_DEACTIVATE_ARCADE_CONTROL_COMPLETE, Transition::TYPE_B}, 

        // TYPE_C transitions (all error transitions)
        {Transition::TRANSITION_CALIBRATE_ERROR, Transition::TYPE_C},
        {Transition::TRANSITION_ACTIVATE_POS_CONTROL_ERROR, Transition::TYPE_C},
        {Transition::TRANSITION_DEACTIVATE_POS_CONTROL_ERROR, Transition::TYPE_C},
        {Transition::TRANSITION_ACTIVATE_VEL_CONTROL_ERROR, Transition::TYPE_C},
        {Transition::TRANSITION_DEACTIVATE_VEL_CONTROL_ERROR, Transition::TYPE_C},
        {Transition::TRANSITION_SHUTDOWN_ERROR, Transition::TYPE_C}
    };
}

void StateManager::handle_get_state(const std::shared_ptr<motor_controller::srv::GetState::Request> request,
        std::shared_ptr<motor_controller::srv::GetState::Response> response) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    (void)request;
    response->current_state.id = current_state_;
    // response->current_state.label = state_to_string(current_state_);
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

TransitionCallbackReturn StateManager::change_arcade_driver_state(const uint8_t arcade_lifecycle_transition) {
    auto arcade_driver_lifecycle_client = this->create_client<lifecycle_msgs::srv::ChangeState>("/arcade_driver/change_state");
    
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = arcade_lifecycle_transition;

    RCLCPP_INFO(get_logger(), "Calling activate arcade driver");

    while (!arcade_driver_lifecycle_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for lifecycle service. Exiting.");
            return TransitionCallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    auto future = arcade_driver_lifecycle_client->async_send_request(request);

    return TransitionCallbackReturn::SUCCESS;
}

TransitionCallbackReturn StateManager::pre_calibration(const uint8_t transition_id) {
    (void)transition_id;
    RCLCPP_INFO(get_logger(), "Pre-calibration complete!");

    auto request = std::make_shared<motor_controller::srv::ChangeState::Request>();
    auto response = std::make_shared<motor_controller::srv::ChangeState::Response>();
    request->transition.id = Transition::TRANSITION_CALIBRATE_COMPLETE;
    StateManager::handle_change_state(request, response);
    return TransitionCallbackReturn::SUCCESS;
}

TransitionCallbackReturn StateManager::shutdown(const uint8_t transition_id) {
    (void)transition_id;
    RCLCPP_INFO(get_logger(), "Shutdown complete!");

    auto request = std::make_shared<motor_controller::srv::ChangeState::Request>();
    auto response = std::make_shared<motor_controller::srv::ChangeState::Response>();
    request->transition.id = Transition::TRANSITION_SHUTDOWN_COMPLETE;
    StateManager::handle_change_state(request, response);
    return TransitionCallbackReturn::SUCCESS;
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::StateManager)