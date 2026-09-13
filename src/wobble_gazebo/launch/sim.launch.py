import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_wobble_description = get_package_share_directory('wobble_description')
    pkg_wobble_gazebo = get_package_share_directory('wobble_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([pkg_wobble_gazebo, 'worlds', world_file])
    bridge_config = os.path.join(pkg_wobble_gazebo, 'config', 'ros_gz_bridge.yaml')
    xacro_file = os.path.join(pkg_wobble_description, 'urdf', 'wobble.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode without GUI if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='wobble_hurdle_course.sdf',
        description='World file to load in wobble_gazebo/worlds'
    )

    # 1. Critical Environment Variables for Linux / Wayland & Network Discovery
    pixi_lib = '/home/vish/PROJECTS/Wobble/.pixi/envs/default/lib'
    python_prefix_lib = os.path.join(os.path.dirname(os.path.dirname(sys.executable)), 'lib')
    conda_prefix = os.environ.get('CONDA_PREFIX', '')
    conda_lib = os.path.join(conda_prefix, 'lib') if conda_prefix else ''
    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    valid_libs = [d for d in [pixi_lib, python_prefix_lib, conda_lib] if os.path.isdir(d)]
    if existing_plugin_path:
        valid_libs.append(existing_plugin_path)
    plugin_path = ':'.join(valid_libs)

    # Force gz-transport to use loopback to prevent Wi-Fi router multicast packet drops
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')
    # Set QT_QPA_PLATFORM to xcb for OGRE / RViz / Gazebo Xwayland compatibility
    set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    set_plugin_path = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path)

    # Process URDF/Xacro
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Gazebo Harmonic Simulation Launchers (GUI or Headless)
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world_path]
        }.items(),
        condition=UnlessCondition(headless)
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-s -r -v 3 ', world_path]
        }.items(),
        condition=IfCondition(headless)
    )

    # 4. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'wobble',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.041', # Spawn upright with wheels touching ground
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )

    # 5. ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wobble_ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        set_gz_ip,
        set_qt_platform,
        set_plugin_path,
        declare_use_sim_time,
        declare_headless,
        declare_world,
        robot_state_publisher_node,
        gz_sim_gui,
        gz_sim_headless,
        spawn_robot,
        ros_gz_bridge
    ])
