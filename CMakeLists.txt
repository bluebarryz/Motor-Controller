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
rosidl_target_interfaces(motor_driver ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv
rclcpp_components_register_nodes(motor_driver "composition::MotorDriver")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
  motor_driver
  ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

ament_package()
