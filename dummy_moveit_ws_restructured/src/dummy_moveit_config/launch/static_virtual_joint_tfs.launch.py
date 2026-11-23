from launch import LaunchDescription


def generate_launch_description():
    # Intentionally empty: the cleaned workspace keeps the fixed world joint in the URDF,
    # so no extra static world->base_link TF publisher is launched here.
    return LaunchDescription([])
