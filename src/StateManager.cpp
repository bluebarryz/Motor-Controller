#include "motor_controller/StateManager.hpp"


using TransitionCallbackReturn = StateManager::TransitionCallbackReturn;

StateManager::StateManager(const rclcpp::NodeOptions &options)
    : Node("state_manager", options), current_state_{State::UNINIT} {
    RCLCPP_INFO(get_logger(), "Initializing StateManager");

    change_state_server = create_service<motor_controller::srv::ChangeState>(
        "~/change_mc_state",
        std::bind(&StateManager::handle_change_state, this, std::placeholders::_1, std::placeholders::_2));
    
    get_state_server = create_service<motor_controller::srv::GetState>(
        "~/get_mc_state",
        std::bind(&StateManager::handle_get_state, this, std::placeholders::_1, std::placeholders::_2));

    arcade_driver_lifecycle_client = create_client<lifecycle_msgs::srv::ChangeState>("/arcade_driver/change_state");
    odrive_sub = create_subscription<std_msgs::msg::String>(
        "/OdriveJsonPub", 10,
        std::bind(&StateManager::odrive_sub_callback, this, std::placeholders::_1));

    odrive_pub = create_publisher<std_msgs::msg::String>(
        "/OdriveJsonSub", 10);

    init_transition_map();
    init_callback_map();
    init_predicate_map();
    init_transition_type_map();
}


// Receives response from ODrive
void StateManager::odrive_sub_callback(const std_msgs::msg::String::SharedPtr odrive_response) {
    json_msg_ = nlohmann::json::parse(odrive_response->data);
    RCLCPP_INFO(get_logger(), "received json msg");
    RCLCPP_INFO(get_logger(), json_msg_["Payload"].dump().c_str());
}

void StateManager::handle_change_state(const std::shared_ptr<motor_controller::srv::ChangeState::Request> request,
    std::shared_ptr<motor_controller::srv::ChangeState::Response> response) {
    RCLCPP_INFO(get_logger(), "calling handle_change_state. Current state: %d. Transition: %d", current_state_, request->transition.id);

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
        if (current_state_ == State::VEL_CONTROL) {    
            auto deactivate_vel_request = std::make_shared<motor_controller::srv::ChangeState::Request>();
            auto deactivate_vel_response = std::make_shared<motor_controller::srv::ChangeState::Response>();
            deactivate_vel_request->transition.id = Transition::TRANSITION_DEACTIVATE_VEL_CONTROL;
            deactivate_vel_request->transition.label = "Deactivate vel control";
            handle_change_state(deactivate_vel_request, deactivate_vel_response);
        }
        
        {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            current_state_ = next_state;
        }
    } else if (transition_type == Transition::TYPE_D) {
        // TODO: error handler transitions
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

        {{State::VEL_CONTROL, Transition::TRANSITION_RESET},
            State::UNINIT},
        {{State::IDLE, Transition::TRANSITION_RESET},
            State::UNINIT},
        {{State::VEL_CONTROL, Transition::TRANSITION_RESET},
            State::UNINIT},
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
        // {Transition::TRANSITION_RESET,
        //     std::bind(&StateManager::reset_state, this, std::placeholders::_1)},
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
        
        // TYPE_C transition (reset)
        {Transition::TRANSITION_RESET, Transition::TYPE_C},

        // TYPE_D transitions (all error transitions)
        {Transition::TRANSITION_CALIBRATE_ERROR, Transition::TYPE_D},
        {Transition::TRANSITION_ACTIVATE_POS_CONTROL_ERROR, Transition::TYPE_D},
        {Transition::TRANSITION_DEACTIVATE_POS_CONTROL_ERROR, Transition::TYPE_D},
        {Transition::TRANSITION_ACTIVATE_VEL_CONTROL_ERROR, Transition::TYPE_D},
        {Transition::TRANSITION_DEACTIVATE_VEL_CONTROL_ERROR, Transition::TYPE_D},
        {Transition::TRANSITION_SHUTDOWN_ERROR, Transition::TYPE_D}
    };
}

void StateManager::handle_get_state(const std::shared_ptr<motor_controller::srv::GetState::Request> request,
        std::shared_ptr<motor_controller::srv::GetState::Response> response) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    (void)request;
    response->current_state.id = current_state_;
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

void StateManager::publish_odrive_request(const nlohmann::json& req_json) {
    std_msgs::msg::String req_msg;
    req_msg.data = req_json.dump();  // Convert JSON to string
    odrive_pub->publish(req_msg);
}

TransitionCallbackReturn StateManager::pre_calibration(const uint8_t transition_id) {
    (void)transition_id;
    RCLCPP_INFO(get_logger(), "Pre-calibration complete!");

    std_msgs::msg::String string_msg;
    
    // Get shared_ptr to this node
    auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this());
    
    publish_odrive_request(odrive_req1_json);

    // Wait for message
    bool response1 = rclcpp::wait_for_message(
        string_msg,
        node_ptr,
        "/OdriveJsonPub",
        std::chrono::seconds(60)
    );
    auto json_res = nlohmann::json::parse(string_msg.data);
    if (response1 && json_res["Payload"].dump() == "\"Success\"") {
        RCLCPP_INFO(get_logger(), "First odrive response success!");
        RCLCPP_INFO(get_logger(), string_msg.data.c_str());
        RCLCPP_INFO(get_logger(), json_res["Payload"].dump().c_str());
        RCLCPP_INFO(get_logger(),  "truth: %d", json_res["Payload"].dump() == "\"Success\"");
        publish_odrive_request(odrive_req2_json);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive odrive response within timeout");
    }

    bool response2 = rclcpp::wait_for_message(
        string_msg,
        node_ptr,
        "/OdriveJsonPub",
        std::chrono::seconds(60)
    );

    json_res = nlohmann::json::parse(string_msg.data);
    if (response2 && json_res["Payload"].dump() == "\"Success\"") {
        RCLCPP_INFO(get_logger(), "Second odrive response success!");
        RCLCPP_INFO(get_logger(), string_msg.data.c_str());
        publish_odrive_request(odrive_req3_json);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive odrive response within timeout");
    }

    bool response3 = rclcpp::wait_for_message(
        string_msg,
        node_ptr,
        "/OdriveJsonPub",
        std::chrono::seconds(60)
    );

    json_res = nlohmann::json::parse(string_msg.data);
    if (response3 && json_res["Payload"].dump() == "\"Success\"") {
        RCLCPP_INFO(get_logger(), "Third odrive response success!");
        RCLCPP_INFO(get_logger(), string_msg.data.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive odrive response within timeout");
    }

    // Can complete the transition now
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

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<StateManager>(options);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}

