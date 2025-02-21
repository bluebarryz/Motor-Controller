from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    container = ComposableNodeContainer(
        name='motor_controller_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='motor_controller',
                plugin='composition::ArcadeDriver',
                name='arcade_driver'
            ),
            ComposableNode(
                package='motor_controller',
                plugin='composition::MotorSpeedController',
                name='motor_speed_controller'
            )
        ]
    )

    state_manager = Node(
        package='motor_controller',
        executable='state_manager',
        name='state_manager',
    )

    # Configure commands
    configure_arcade = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/arcade_driver', 'configure'],
        output='screen'
    )

    configure_motor = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/motor_speed_controller', 'configure'],
        output='screen'
    )

    activate_motor = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/motor_speed_controller', 'activate'],
        output='screen'
    )

    calibrate_command = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/state_manager/change_mc_state', 
             'motor_controller/srv/ChangeState',
             '{\"transition\": {\"id\": 6, \"label\": \"calibrate\"}}'],
        output='screen'
    )

    activate_vel_control = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/state_manager/change_mc_state',
             'motor_controller/srv/ChangeState',
             '{\"transition\": {\"id\": 12, \"label\": \"active_vel_control\"}}'],
        output='screen'
    )

    # Event handlers
    container_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=container,
            on_start=[configure_arcade]
        )
    )

    arcade_config_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=configure_arcade,
            on_completion=[configure_motor]
        )
    )

    motor_config_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=configure_motor,
            on_completion=[activate_motor]
        )
    )

    motor_activate_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=activate_motor,
            on_completion=[calibrate_command]
        )
    )

    calibrate_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=calibrate_command,
            on_completion=[activate_vel_control]
        )
    )

    return LaunchDescription([
        state_manager,
        container,
        container_handler,
        arcade_config_handler,
        motor_config_handler,
        motor_activate_handler,
        calibrate_handler
    ])