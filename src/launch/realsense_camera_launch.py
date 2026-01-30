from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    my_package_share_dir = get_package_share_directory("realsense2_camera")

    included_python_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_package_share_dir, "launch", "rs_launch.py")),
        launch_arguments=[
            ("color_qos", "SENSOR_DATA"),
            ("depth_qos", "SENSOR_DATA"),
            ("enable_depth", "false"),
            ("spatial_filter.enable", "false"),
            ("temporal_filter.enable", "false"),
            ("pointcloud.enable", "false"),
            ("hole_filling_filter.enable", "false"),
            ("depth_module.emitter_enabled", "false"),
        ],
    )

    return LaunchDescription(
        [
            included_python_launch,
        ]
    )
