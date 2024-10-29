cmake_minimum_required(VERSION 3.8)
project(arcade_control)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JoystickInput.srv"
  "msg/Speed.msg"
	DEPENDENCIES builtin_interfaces
	)
ament_export_dependencies(rosidl_default_runtime)

# motor_driver component
add_library(motor_driver SHARED src/MotorDriver.cpp)
target_compile_definitions(motor_driver
	PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(motor_driver
	"rclcpp"
	"rclcpp_components"
)

rosidl_target_interfaces(motor_driver ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(motor_driver "composition::MotorDriver")

add_executable(motor_executable src/main.cpp)
target_link_libraries(motor_executable
  motor_driver
)
ament_target_dependencies(motor_executable
  rclcpp
  rclcpp_components
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(joystick_test test/joystick_test.cpp)
  
  # Link against the motor_driver component library
  target_link_libraries(joystick_test
    motor_driver
  )
  
  # Link the test with the interfaces
  rosidl_target_interfaces(joystick_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  ament_target_dependencies(joystick_test
    rclcpp
    rclcpp_components
  )
endif()

# Install targets
install(TARGETS
  motor_driver
  motor_executable  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()