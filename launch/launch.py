from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='arcade_control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='arcade_control',
                    plugin='composition::ArcadeDriver',
                    name='arcade_driver')
            ]
        )
    ])