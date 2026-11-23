from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('dummy-ros2', package_name='dummy_moveit_config')
        .robot_description(file_path='config/dummy-ros2.urdf.xacro')
        .robot_description_semantic(file_path='config/dummy-ros2.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

    declared_arguments = [
        DeclareLaunchArgument('mode', default_value='roundtrip'),
        DeclareLaunchArgument('group_name', default_value='dummy_arm'),
        DeclareLaunchArgument('tip_link', default_value='link6_1_1'),
        DeclareLaunchArgument('named_state', default_value='reset'),
        DeclareLaunchArgument('ik_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('ik_attempts', default_value='5'),
    ]

    validator = Node(
        package='dummy_kinematics_validation',
        executable='ik_fk_validator',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'mode': LaunchConfiguration('mode'),
                'group_name': LaunchConfiguration('group_name'),
                'tip_link': LaunchConfiguration('tip_link'),
                'named_state': LaunchConfiguration('named_state'),
                'ik_timeout_s': LaunchConfiguration('ik_timeout_s'),
                'ik_attempts': LaunchConfiguration('ik_attempts'),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [validator])
