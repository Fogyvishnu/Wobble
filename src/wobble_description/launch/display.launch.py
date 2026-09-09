import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('wobble_description')
    default_xacro_path = os.path.join(pkg_share, 'urdf', 'wobble.urdf.xacro')
    default_rviz_path = os.path.join(pkg_share, 'rviz', 'wobble.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_xacro_path,
        description='Absolute path to robot urdf/xacro file'
    )

    declare_rviz = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_path,
        description='Absolute path to rviz config file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizconfig],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_rviz,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
