from launch import LaunchDescription
from launch_ros.actions import Node

# VERY MUCH IN DEVELOPMENT THIS IS JUST A TEMPLATE
def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='autopilot_cpp',
        #     executable='sailboat_autopilot',
        #     name='sailboat_autopilot',

        #     respawn=True, 
        #     respawn_delay=2.0,
        #     output="screen"
        # ),
        
        
        # Node(
        #     package='autopilot',
        #     executable='telemetry',
        #     name='telemetry',

        #     respawn=True, 
        #     respawn_delay=2.0,
        #     output="log"
        # ),
        
        Node(
            package='autopilot_cpp',
            executable='telemetry',
            name='telemetry',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        
        Node(
            package='simulation',
            executable='simulation',
            name='simulation',

            respawn=True, 
            respawn_delay=2.0,
            
            output="screen",
            emulate_tty=True
        )
    ])