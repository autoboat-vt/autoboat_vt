from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="object_detection",
                executable="cam_corder",
                name="cam_corder",
                respawn=True,
                respawn_delay=2.0,
                # output="log"
            ),
        ]
    )
