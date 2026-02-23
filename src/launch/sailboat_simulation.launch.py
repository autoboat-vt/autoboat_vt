from launch_ros.actions import Node

from launch import LaunchDescription


# VERY MUCH IN DEVELOPMENT THIS IS JUST A TEMPLATE
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autopilot",
                executable="sailboat_autopilot",
                name="sailboat_autopilot",
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            ),
            Node(package="autopilot", executable="telemetry", name="telemetry", respawn=True, respawn_delay=2.0, output="log"),
            Node(
                package="sailboat_simulation",
                executable="simulation",
                name="sailboat_simulation",
                respawn=True,
                respawn_delay=2.0,
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
