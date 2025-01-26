from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='motor_controller_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='motor_controller',
                    plugin='composition::ArcadeDriver',
                    name='arcade_driver'),
                ComposableNode(
                    package='motor_controller',
                    plugin='composition::MotorSpeedController',
                    name='motor_speed_controller'),
                ComposableNode(
                    package='motor_controller',
                    plugin='composition::StateManager',
                    name='state_manager')
            ]
        )
    ])