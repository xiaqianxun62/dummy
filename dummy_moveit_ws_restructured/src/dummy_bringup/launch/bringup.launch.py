import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def as_bool(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def load_defaults():
    defaults_file = os.path.join(get_package_share_directory('dummy_bringup'), 'config', 'hardware_defaults.yaml')
    with open(defaults_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context).lower()
    use_rviz = as_bool(LaunchConfiguration('use_rviz').perform(context))
    run_move_group = as_bool(LaunchConfiguration('run_move_group').perform(context))
    use_sim_time = as_bool(LaunchConfiguration('use_sim_time').perform(context))

    serial_port = LaunchConfiguration('serial_port').perform(context)
    baud_rate = LaunchConfiguration('baud_rate').perform(context)
    command_mode = LaunchConfiguration('command_mode').perform(context)
    joint_command_prefix = LaunchConfiguration('joint_command_prefix').perform(context)
    joint_speed_deg = LaunchConfiguration('joint_speed_deg').perform(context)
    position_offsets_rad = LaunchConfiguration('position_offsets_rad').perform(context)
    position_signs = LaunchConfiguration('position_signs').perform(context)
    write_rate_hz = LaunchConfiguration('write_rate_hz').perform(context)
    read_timeout_ms = LaunchConfiguration('read_timeout_ms').perform(context)
    slow_command_timeout_ms = LaunchConfiguration('slow_command_timeout_ms').perform(context)
    start_on_activate = LaunchConfiguration('start_on_activate').perform(context)
    disable_on_deactivate = LaunchConfiguration('disable_on_deactivate').perform(context)
    auto_home_on_activate = LaunchConfiguration('auto_home_on_activate').perform(context)

    moveit_config = (
        MoveItConfigsBuilder('dummy-ros2', package_name='dummy_moveit_config')
        .robot_description(
            file_path='config/dummy-ros2.urdf.xacro',
            mappings={
                'use_fake_hardware': use_fake_hardware,
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'command_mode': command_mode,
                'joint_command_prefix': joint_command_prefix,
                'joint_speed_deg': joint_speed_deg,
                'position_offsets_rad': position_offsets_rad,
                'position_signs': position_signs,
                'write_rate_hz': write_rate_hz,
                'read_timeout_ms': read_timeout_ms,
                'slow_command_timeout_ms': slow_command_timeout_ms,
                'start_on_activate': start_on_activate,
                'disable_on_deactivate': disable_on_deactivate,
                'auto_home_on_activate': auto_home_on_activate,
            },
        )
        .robot_description_semantic(file_path='config/dummy-ros2.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )

    controllers_yaml_path = os.path.join(
        get_package_share_directory('dummy_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('dummy_moveit_config'),
        'config',
        'moveit.rviz',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[moveit_config.robot_description, controllers_yaml_path, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dummy_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    arm_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[arm_controller_spawner])
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_after_jsb,
    ]

    if run_move_group:
        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'publish_robot_description': True,
                    'publish_robot_description_semantic': True,
                    'allow_trajectory_execution': True,
                    'capabilities': '',
                    'disable_capabilities': '',
                    'monitor_dynamics': False,
                    'use_sim_time': use_sim_time,
                },
            ],
        )
        nodes.append(move_group_node)

    if use_rviz:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    defaults = load_defaults()

    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_move_group', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value=str(defaults['serial_port'])),
        DeclareLaunchArgument('baud_rate', default_value=str(defaults['baud_rate'])),
        DeclareLaunchArgument('command_mode', default_value=str(defaults['command_mode'])),
        DeclareLaunchArgument('joint_command_prefix', default_value=str(defaults['joint_command_prefix'])),
        DeclareLaunchArgument('joint_speed_deg', default_value=str(defaults['joint_speed_deg'])),
        DeclareLaunchArgument('position_offsets_rad', default_value=str(defaults['position_offsets_rad'])),
        DeclareLaunchArgument('position_signs', default_value=str(defaults['position_signs'])),
        DeclareLaunchArgument('write_rate_hz', default_value=str(defaults['write_rate_hz'])),
        DeclareLaunchArgument('read_timeout_ms', default_value=str(defaults['read_timeout_ms'])),
        DeclareLaunchArgument('slow_command_timeout_ms', default_value=str(defaults['slow_command_timeout_ms'])),
        DeclareLaunchArgument('start_on_activate', default_value=str(defaults['start_on_activate']).lower()),
        DeclareLaunchArgument('disable_on_deactivate', default_value=str(defaults['disable_on_deactivate']).lower()),
        DeclareLaunchArgument('auto_home_on_activate', default_value=str(defaults['auto_home_on_activate']).lower()),
        OpaqueFunction(function=launch_setup),
    ])
