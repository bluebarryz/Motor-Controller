cmake_minimum_required(VERSION 3.8)
project(motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # Added for Joy messages

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArcadeSpeed.msg"
  "msg/MotorSpeeds.msg"
  "msg/Transition.msg"
  "srv/ChangeState.srv"
  "srv/GetState.srv"
  "msg/State.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

#### joy_to_twist component ###
add_library(joy_to_twist SHARED
  src/JoyToTwist.cpp
)
target_compile_definitions(joy_to_twist
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(joy_to_twist
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
)
rclcpp_components_register_nodes(joy_to_twist "motor_controller::JoyToTwist")
#########

#### arcade_driver component ###
add_library(arcade_driver SHARED src/ArcadeDriver.cpp)
target_compile_definitions(arcade_driver
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(arcade_driver
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "geometry_msgs"
)
rosidl_target_interfaces(arcade_driver ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(arcade_driver "composition::ArcadeDriver")
#########

#### motor_speed_controller component ###
add_library(motor_speed_controller SHARED src/MotorSpeedController.cpp)
target_compile_definitions(motor_speed_controller
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(motor_speed_controller
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
)
rosidl_target_interfaces(motor_speed_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(motor_speed_controller "composition::MotorSpeedController")
#########

#### state_manager component ###
add_library(state_manager SHARED src/StateManager.cpp)
target_compile_definitions(state_manager
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(state_manager
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
)
rosidl_target_interfaces(state_manager ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(state_manager "composition::StateManager")
#########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(joystick_test test/joystick_test.cpp)
  
  target_link_libraries(joystick_test
    arcade_driver
    motor_speed_controller
    state_manager
    joy_to_twist
  )
  
  rosidl_target_interfaces(joystick_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  ament_target_dependencies(joystick_test
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
  )
endif()

# Install targets
install(TARGETS
  arcade_driver
  motor_speed_controller
  state_manager
  joy_to_twist
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()