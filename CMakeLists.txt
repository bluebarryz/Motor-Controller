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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArcadeSpeed.msg"
  "msg/MotorSpeeds.msg"
	DEPENDENCIES builtin_interfaces geometry_msgs
	)
ament_export_dependencies(rosidl_default_runtime)


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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(joystick_test test/joystick_test.cpp)
  
  # Link against the arcade_driver and motor_speed_controller component libraries
  target_link_libraries(joystick_test
    arcade_driver
    motor_speed_controller
  )
  
  # Link the test with the interfaces
  rosidl_target_interfaces(joystick_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  ament_target_dependencies(joystick_test
    rclcpp
    rclcpp_components
    geometry_msgs
  )
endif()

# Install targets
install(TARGETS
  arcade_driver
  motor_speed_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()