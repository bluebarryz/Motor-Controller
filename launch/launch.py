from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js2',
        description='Joystick device path'
    )

    return LaunchDescription([
        device_arg,
        
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_device'),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        
        ComposableNodeContainer(
            name='motor_controller_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='motor_controller',
                    plugin='motor_controller::JoyToTwist',
                    name='joy_to_twist',
                    parameters=[{
                        'axis_linear': 1,    # Left stick vertical
                        'axis_angular': 2,   # Right stick horizontal
                        'scale_linear': 1.0,
                        'scale_angular': -1.0, # Logitech controller is reversed (left is positive, right negative)
                    }]
                ),
                ComposableNode(
                    package='motor_controller',
                    plugin='composition::ArcadeDriver',
                    name='arcade_driver'
                ),
                ComposableNode(
                    package='motor_controller',
                    plugin='composition::MotorSpeedController',
                    name='motor_speed_controller'
                ),
                ComposableNode(
                    package='motor_controller',
                    plugin='composition::StateManager',
                    name='state_manager'
                )
            ]
        )
    ])