from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('dummy-ros2', package_name='dummy_moveit_config')
        .robot_description(file_path='config/dummy-ros2.urdf.xacro')
        .robot_description_semantic(file_path='config/dummy-ros2.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)
