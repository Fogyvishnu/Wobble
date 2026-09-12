import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, GroupAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_wobble_description = get_package_share_directory('wobble_description')
    pkg_wobble_gazebo = get_package_share_directory('wobble_gazebo')
    pkg_wobble_control = get_package_share_directory('wobble_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_wobble_gazebo, 'worlds', 'wobble_hurdle_course.sdf')
    bridge_config = os.path.join(pkg_wobble_gazebo, 'config', 'ros_gz_bridge.yaml')
    xacro_file = os.path.join(pkg_wobble_description, 'urdf', 'wobble.urdf.xacro')
    rviz_config = os.path.join(pkg_wobble_description, 'rviz', 'wobble.rviz')
    balance_params = os.path.join(pkg_wobble_control, 'config', 'balance_params.yaml')

    use_rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    manual_mode = LaunchConfiguration('manual')
    use_remote = LaunchConfiguration('remote')
    enable_balance = LaunchConfiguration('balance')
    enable_yolo = LaunchConfiguration('yolo')

    declare_use_rviz = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2 if true')
    declare_headless = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless if true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time')
    declare_manual = DeclareLaunchArgument('manual', default_value='false', description='Enable manual mode instead of autonomous navigator if true')
    declare_use_remote = DeclareLaunchArgument('remote', default_value=manual_mode, description='Start remote control GUI if true')
    declare_balance = DeclareLaunchArgument('balance', default_value='true', description='Start balance controller if true')
    declare_yolo = DeclareLaunchArgument('yolo', default_value='false', description='Start YOLOv8 perception node if true')

    # Environment variables for Wayland/X11 & Loopback Discovery
    pixi_lib = '/home/vish/PROJECTS/Wobble/.pixi/envs/default/lib'
    python_prefix_lib = os.path.join(os.path.dirname(os.path.dirname(sys.executable)), 'lib')
    conda_prefix = os.environ.get('CONDA_PREFIX', '')
    conda_lib = os.path.join(conda_prefix, 'lib') if conda_prefix else ''
    existing_plugin = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    valid_libs = [d for d in [pixi_lib, python_prefix_lib, conda_lib] if os.path.isdir(d)]
    if existing_plugin:
        valid_libs.append(existing_plugin)
    plugin_path = ':'.join(valid_libs)

    actions = [
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        declare_use_rviz,
        declare_headless,
        declare_use_sim_time,
        declare_manual,
        declare_use_remote,
        declare_balance,
        declare_yolo
    ]

    if os.path.exists('/usr/lib/libdrm_amdgpu.so.1'):
        actions.append(SetEnvironmentVariable('LD_PRELOAD', '/usr/lib/libdrm_amdgpu.so.1'))

    # Robot Description
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )
    actions.append(rsp_node)

    # 2. Gazebo Simulation (GUI or Headless)
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
        condition=UnlessCondition(headless)
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-s -r -v 3 {world_path}'}.items(),
        condition=IfCondition(headless)
    )
    actions.extend([gz_sim_gui, gz_sim_headless])

    # 3. Spawn Robot in Hurdle Course (z=0.041 places wheels gently on ground)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'wobble', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.041', '-R', '0.0', '-P', '0.0', '-Y', '0.0']
    )
    actions.append(spawn_robot)

    # 4. Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wobble_ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time}],
        arguments=[
            '/world/wobble_hurdle_course/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            '/world/wobble_hurdle_course/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean'
        ]
    )
    actions.append(bridge_node)

    # 5. Consolidated Controller Spawner (Activates all controllers simultaneously)
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
    actions.append(spawner_node)

    # 6. Balance Controller Node
    balance_node = Node(
        package='wobble_control', executable='balance_controller',
        name='wobble_balance_controller', parameters=[balance_params], output='screen'
    )

    # 7. Autonomous Course Navigator Node (runs when manual:=false)
    navigator_node = Node(
        package='wobble_control', executable='course_navigator',
        name='wobble_course_navigator', condition=UnlessCondition(manual_mode), output='screen'
    )

    # 7b. Remote Control Node (runs when manual:=true)
    remote_node = Node(
        package='wobble_control', executable='remote_control',
        name='wobble_remote_control', condition=IfCondition(manual_mode), output='screen'
    )

    # 7c. YOLOv8 Deep Learning Perception Node (runs when yolo:=true)
    yolo_node = Node(
        package='wobble_control', executable='yolo_detector',
        name='wobble_yolo_detector', condition=IfCondition(enable_yolo), output='screen'
    )

    # Launch control nodes after all controllers are fully activated
    delay_control_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_node,
            on_exit=[
                GroupAction(actions=[balance_node], condition=IfCondition(enable_balance)),
                GroupAction(actions=[navigator_node], condition=UnlessCondition(manual_mode)),
                GroupAction(actions=[remote_node], condition=IfCondition(use_remote)),
                GroupAction(actions=[yolo_node], condition=IfCondition(enable_yolo))
            ]
        )
    )
    actions.append(delay_control_nodes)

    # 8. RViz Visualization
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz), output='screen'
    )
    actions.append(rviz_node)

    return LaunchDescription(actions)
