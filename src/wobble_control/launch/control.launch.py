import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('wobble_control')
    params_file = os.path.join(pkg_share, 'config', 'balance_params.yaml')

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawner for left wheel effort controller
    left_wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_wheel_effort_controller'],
        output='screen'
    )

    # Spawner for right wheel effort controller
    right_wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_wheel_effort_controller'],
        output='screen'
    )

    # Spawner for hip servos position controller
    hip_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hip_position_controller'],
        output='screen'
    )

    # Wobble Cascaded PID Balance Controller Node
    balance_controller_node = Node(
        package='wobble_control',
        executable='balance_controller',
        name='wobble_balance_controller',
        parameters=[params_file],
        output='screen'
    )

    # Start balance node after controllers are active
    delay_balance_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hip_controller_spawner,
            on_exit=[balance_controller_node]
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        left_wheel_controller_spawner,
        right_wheel_controller_spawner,
        hip_controller_spawner,
        delay_balance_node
    ])
