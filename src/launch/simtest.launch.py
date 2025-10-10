from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    motorboat_pkg_path = FindPackageShare('motorboat_sim_testing')  # Replace with your own package name
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
                'gz_args': "/home/ws/src/motorboat_sim_testing/buoyant_cylinder.sdf",  # Replace with your own world file
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        # Node(
        #     package='autopilot',
        #     executable='motorboat_autopilot',
        #     name='motorboat_autopilot',

        #     respawn=True,
        #     respawn_delay=2.0,
        #     # output="log"
        # ),

        # Node(
        #     package='autopilot',
        #     executable='telemetry',
        #     name='telemetry',

        #     respawn=True,
        #     respawn_delay=2.0,
        #     # output="log"
        # ),

        # Node(
        #     package='sensors',
        #     executable='gps',
        #     name='gps',

        #     respawn=True,
        #     respawn_delay=2.0,
        #     # output="log"
        # ),

        # Node(
        #     package='sensors',
        #     executable='rc',
        #     name='rc',

        #     respawn=True,
        #     respawn_delay=2.0,
        #     # output="log"
        # ),


        # Node(
        #     package='vesc',
        #     executable='vesc',
        #     name='vesc',

        #     respawn=True,
        #     respawn_delay=0.01,
        #     # output="log"
        # ),

        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     name='micro_ros_agent',
        #     output='screen',
        #     arguments=['serial', '--dev', "/dev/pico", "-b", "115200"]
        # ),
    

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['motorboat/propeller_topic@std_msgs/msg/Float64]gz.msgs.Double',],
            # remappings=[('/example_imu_topic',
            #              '/remapped_imu_topic'),],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/rudder@std_msgs/msg/Float64]gz.msgs.Double',],
            # remappings=[('/example_imu_topic',
            #              '/remapped_imu_topic'),],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',],
            # remappings=[('/example_imu_topic',
            #              '/remapped_imu_topic'),],
            output='screen'
        )
    ]
    
    
    
    )