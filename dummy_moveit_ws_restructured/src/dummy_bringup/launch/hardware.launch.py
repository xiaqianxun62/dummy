from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('dummy_bringup'), 'launch', 'bringup.launch.py'])
        ]),
        launch_arguments={
            'use_fake_hardware': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'run_move_group': LaunchConfiguration('run_move_group'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'command_mode': LaunchConfiguration('command_mode'),
            'joint_command_prefix': LaunchConfiguration('joint_command_prefix'),
            'joint_speed_deg': LaunchConfiguration('joint_speed_deg'),
            'position_offsets_rad': LaunchConfiguration('position_offsets_rad'),
            'position_signs': LaunchConfiguration('position_signs'),
            'write_rate_hz': LaunchConfiguration('write_rate_hz'),
            'read_timeout_ms': LaunchConfiguration('read_timeout_ms'),
            'slow_command_timeout_ms': LaunchConfiguration('slow_command_timeout_ms'),
            'start_on_activate': LaunchConfiguration('start_on_activate'),
            'disable_on_deactivate': LaunchConfiguration('disable_on_deactivate'),
            'auto_home_on_activate': LaunchConfiguration('auto_home_on_activate'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_move_group', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('command_mode', default_value='2'),
        DeclareLaunchArgument('joint_command_prefix', default_value='>'),
        DeclareLaunchArgument('joint_speed_deg', default_value='0.0'),
        DeclareLaunchArgument('position_offsets_rad', default_value='0,0,1.57079,0,0,0'),
        DeclareLaunchArgument('position_signs', default_value='1,1,1,1,-1,-1'),
        DeclareLaunchArgument('write_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('read_timeout_ms', default_value='200.0'),
        DeclareLaunchArgument('slow_command_timeout_ms', default_value='15000.0'),
        DeclareLaunchArgument('start_on_activate', default_value='false'),
        DeclareLaunchArgument('disable_on_deactivate', default_value='true'),
        DeclareLaunchArgument('auto_home_on_activate', default_value='false'),
        bringup,
    ])
