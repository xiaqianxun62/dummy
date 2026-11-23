import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('dummy-ros2', package_name='dummy_moveit_config')
        .robot_description(file_path='config/dummy-ros2.urdf.xacro')
        .robot_description_semantic(file_path='config/dummy-ros2.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    servo_params = (
        ParameterBuilder('dummy_moveit_config')
        .yaml(parameter_namespace='moveit_servo', file_path='config/dummy_simulated_config.yaml')
        .to_dict()
    )

    servo_node = Node(
        package='dummy_moveit_config',
        executable='dummy_servo_cpp_interface_demo',
        output='screen',
        parameters=[servo_params, moveit_config.robot_description, moveit_config.robot_description_semantic],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    rviz_full_config = os.path.join(get_package_share_directory('dummy_moveit_config'), 'config', 'moveit_empty.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_full_config],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory('dummy_moveit_config'), 'config', 'ros2_controllers.yaml'),
        ],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '120'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['dummy_arm_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '120'],
        output='screen',
    )

    arm_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[arm_controller_spawner])
    )

    return LaunchDescription([
        rviz_node,
        servo_node,
        ros2_control_node,
        robot_state_publisher,
        joint_state_spawner,
        arm_controller_after_jsb,
    ])
