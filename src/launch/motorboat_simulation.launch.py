
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')

    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch','gz_sim.launch.py'])

    return LaunchDescription([
        # SetEnvironmentVariable(
        #     'GZ_SIM_RESOURCE_PATH',
        #     PathJoinSubstitution([example_pkg_path, 'models'])
        # ),
        # SetEnvironmentVariable(
        #     'GZ_SIM_PLUGIN_PATH',
        #     PathJoinSubstitution([example_pkg_path, 'plugins'])
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': "-s --headless-rendering -r /home/ws/src/motorboat_simulation/motorboat_model.sdf",
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
         
        Node(
            package='autopilot_transform',
            executable='autopilot_transform',
            name='autopilot_transform',
            respawn=True,
            respawn_delay=2.0,
        ),


        Node(
            package='autopilot',
            executable='telemetry',
            name='telemetry',

            respawn=True,
            respawn_delay=2.0,
            output="log"
        ),
         
         Node(
            package='autopilot',
            executable='motorboat_autopilot',
            name='motorboat_autopilot',

            respawn=True,
            respawn_delay=2.0,
            # remappings=[('/motorboat_simulation/position', '/position')],
            output="log"
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/motorboat_simulation/desired_propeller_rpm@std_msgs/msg/Float64]gz.msgs.Double',
                '/motorboat_simulation/desired_rudder_angle@std_msgs/msg/Float64]gz.msgs.Double',
                '/motorboat_simulation/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/motorboat_simulation/position@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
            ],
            remappings=[('/motorboat_simulation/position', '/position')],

            output='screen'
        ),
    ])
