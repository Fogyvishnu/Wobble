import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_wobble_bringup = get_package_share_directory('wobble_bringup')

    use_rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    use_remote = LaunchConfiguration('remote')
    manual_mode = LaunchConfiguration('manual')
    enable_balance = LaunchConfiguration('balance')

    declare_use_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz2 if true'
    )
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false', description='Run Gazebo headless if true'
    )
    declare_manual = DeclareLaunchArgument(
        'manual', default_value='true', description='Enable manual drive mode if true'
    )
    declare_use_remote = DeclareLaunchArgument(
        'remote', default_value=manual_mode, description='Start remote control GUI if true'
    )
    declare_balance = DeclareLaunchArgument(
        'balance', default_value='true', description='Start balance controller if true'
    )

    # Master hurdle course launch with YOLOv8 perception enabled
    hurdle_course_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wobble_bringup, 'launch', 'hurdle_course.launch.py')
        ),
        launch_arguments={
            'rviz': use_rviz,
            'headless': headless,
            'manual': manual_mode,
            'remote': use_remote,
            'balance': enable_balance,
            'yolo': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_headless,
        declare_use_remote,
        declare_manual,
        declare_balance,
        hurdle_course_launch
    ])
