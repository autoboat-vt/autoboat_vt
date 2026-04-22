from launch import LaunchDescription
from launch_ros.actions import Node


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
            Node(package="sensors", executable="rc", name="rc", respawn=True, respawn_delay=2.0, output="log"),
            Node(package="sensors", executable="gps", name="gps", respawn=True, respawn_delay=2.0, output="log"),
            Node(package="sensors", executable="wind_sensor", name="wind_sensor", respawn=True, respawn_delay=2.0, output="log"),


            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                arguments=["serial", "--dev", "/dev/pico", "-b", "115200"],
            )
        ]
    )
