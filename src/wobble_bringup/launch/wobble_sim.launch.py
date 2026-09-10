import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_wobble_gazebo = get_package_share_directory('wobble_gazebo')
    pkg_wobble_control = get_package_share_directory('wobble_control')
    pkg_wobble_description = get_package_share_directory('wobble_description')

    rviz_config_path = os.path.join(pkg_wobble_description, 'rviz', 'wobble.rviz')

    use_rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    use_remote = LaunchConfiguration('remote')

    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 if true'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode without GUI if true'
    )

    declare_use_remote = DeclareLaunchArgument(
        'remote',
        default_value='true',
        description='Start remote control GUI alongside Gazebo if true'
    )

    actions = [
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        declare_use_rviz,
        declare_headless,
        declare_use_remote
    ]

    if os.path.exists('/usr/lib/libdrm_amdgpu.so.1'):
        actions.append(SetEnvironmentVariable('LD_PRELOAD', '/usr/lib/libdrm_amdgpu.so.1'))

    # 1. Gazebo Harmonic Simulation & Spawner
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wobble_gazebo, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'headless': headless
        }.items()
    )

    # 2. Control System & Balance Node
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wobble_control, 'launch', 'control.launch.py')
        )
    )

    # 3. RViz Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # 4. Remote Control (WASD + O/P) GUI Node
    remote_node = Node(
        package='wobble_control',
        executable='remote_control',
        name='wobble_remote_control',
        condition=IfCondition(use_remote),
        output='screen'
    )

    actions.extend([
        sim_launch,
        control_launch,
        rviz_node,
        remote_node
    ])

    return LaunchDescription(actions)
