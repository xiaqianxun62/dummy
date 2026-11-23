from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('dummy_ros2_description'), 'launch', 'gazebo.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('dummy_bringup'), 'launch', 'bringup.launch.py'])
        ]),
        launch_arguments={
            'use_fake_hardware': 'true',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'run_move_group': LaunchConfiguration('run_move_group'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('launch_gazebo', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_move_group', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        gazebo,
        bringup,
    ])
