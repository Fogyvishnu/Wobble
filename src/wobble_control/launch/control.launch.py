import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('wobble_control')
    params_file = os.path.join(pkg_share, 'config', 'balance_params.yaml')

    # Consolidated Controller Spawner (Activates all controllers simultaneously)
    spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'left_wheel_effort_controller',
            'right_wheel_effort_controller',
            'hip_position_controller',
            '--controller-manager-timeout', '30',
            '--activate-as-group'
        ],
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
            target_action=spawner_node,
            on_exit=[balance_controller_node]
        )
    )

    return LaunchDescription([
        spawner_node,
        delay_balance_node
    ])
