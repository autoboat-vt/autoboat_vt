import json
from math import atan2, degrees, floor
from pathlib import Path

import navpy
import rclpy
from autoboat_msgs.msg import ObjectDetectionResult, ObjectDetectionResultsList
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix


class SimulationCameraEngine(Node):
    def __init__(self) -> None:
        super().__init__("simulation_camera_engine_node")

        parameters_path = Path(__file__).resolve().parent.parent / "config/simulation_camera_engine_config.json"
        with open(parameters_path, "r", encoding="utf-8") as parameters_file:
            camera_engine_config = json.load(parameters_file)

        self.camera_focal_length_meter = camera_engine_config["camera"]["focal_length_millimeter"] / 1000
        self.camera_max_detection_depth_meter = camera_engine_config["camera"]["max_detection_depth_meter"]
        self.object_array = camera_engine_config["object_array"]

        self.camera_pixel_pitch_meter = camera_engine_config["camera"]["pixel_pitch_micrometer"] / 1000000
        camera_sensor_array_size_x_meter = camera_engine_config["camera"]["sensor_array_size_millimeter"]["x"] / 1000
        camera_sensor_array_size_y_meter = camera_engine_config["camera"]["sensor_array_size_millimeter"]["y"] / 1000

        self.camera_number_of_pixels_along_width = int(camera_sensor_array_size_x_meter / self.camera_pixel_pitch_meter)
        self.camera_number_of_pixels_along_height = int(camera_sensor_array_size_y_meter / self.camera_pixel_pitch_meter)

        self.simulation_transform_refresh_timer = self.create_timer(0.02, self.update_ros_topics)

        self.create_subscription(NavSatFix, "/position", self.position_callback, 10)
        self.create_subscription(Odometry, "/motorboat_simulation/odometry", self.odometry_callback, 10)

        self.object_detection_results_publisher = self.create_publisher(
            ObjectDetectionResultsList, "/object_detection_results", 10
        )

        self.odometry = Odometry()
        self.position = NavSatFix()

    def position_callback(self, position: NavSatFix) -> None:
        """A callback function to get the current position of the boat."""
        self.position = position

    def odometry_callback(self, odometry: Odometry) -> None:
        """A callback function to get the current odometry of the boat."""
        self.odometry = odometry

    def update_ros_topics(self) -> None:
        """A periodically called function that publishes fake object detection data to the autopilot and triangulation engine."""

        twist = Twist()
        twist.linear = self.odometry.twist.twist.linear
        twist.angular = self.odometry.twist.twist.angular

        orientation = self.odometry.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        rotation: Rotation = Rotation.from_quat([x, y, z, w])

        object_detection_results: list = []

        # Compute perspective projection
        # https://cseweb.ucsd.edu/classes/fa12/cse252A-a/lec4.pdf
        # https://www.cse.unr.edu/~bebis/CS791E/Notes/PerspectiveProjection.pdf
        for obj in self.object_array:
            local_y, local_x, local_down = navpy.lla2ned(
                obj["latitude"], obj["longitude"], 0,
                self.position.latitude, self.position.longitude, self.position.altitude
            )
            local_z = -1 * local_down

            print(f"original_location: {[local_x, local_y, local_z]}")
            print(f"result: {rotation.apply([local_x, local_y, local_z], inverse=True)}")

            object_location_relative_to_camera = rotation.apply([local_x, local_y, local_z], inverse=True)

            # Just to explain the coordinate scheme of object_location_relative_to_camera, the first element measures
            # the distance straight out of the camera, the second element measures the distance to the LEFT of where
            # the boat is facing, and the third element measures the distance above where the boat is facing
            # We will swap this right now to match most of the material online for perspective projection (see UNR notes) where
            # "Z" (third element) is the distance straight out of the camera, "X" (first element) is the distance to the left
            # of where the camera is facing, and "Y" (second element) is the distance above where the camera is facing.
            # This way, the vector simply reads [x, y, z].
            # TL:DR we are just transforming FLU to LUF coordinate scheme (forward, left, up to left, up, forward)
            object_location_relative_to_camera = [
                object_location_relative_to_camera[1],  # distance to the left of where the camera is facing
                object_location_relative_to_camera[2],  # distance above where the camera is facing
                object_location_relative_to_camera[0],  # distance straight out of the camera
            ]


            # This is the classic perspective projection formula found here:
            # https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/building-basic-perspective-projection-matrix.html
            # If you would like a slightly more advanced/ rigorous perspective, I would recommend looking at MIT's textbook where
            # they represent perspective projection as a homography transformation using homogeneous matrices/ coordinates:
            # https://visionbook.mit.edu/imaging_geometry.html#from-meters-to-pixels
            f = self.camera_focal_length_meter
            x = object_location_relative_to_camera[0]
            y = object_location_relative_to_camera[1]
            z = object_location_relative_to_camera[2]
            object_location_on_camera_sensor_array_x = f * x / z
            object_location_on_camera_sensor_array_y = f * y / z

            object_pixel_location_x = floor(object_location_on_camera_sensor_array_x / self.camera_pixel_pitch_meter)
            object_pixel_location_y = floor(object_location_on_camera_sensor_array_y / self.camera_pixel_pitch_meter)

            # Make sure that we don't see any objects that are too far away or behind us.
            # Also make sure that we don't see any objects out of frame
            # Also check if the object is horizontally and vertically out of bounds
            if (
                object_location_relative_to_camera[2] > self.camera_max_detection_depth_meter or
                object_location_relative_to_camera[2] < 0 or
                abs(object_pixel_location_x) > (self.camera_number_of_pixels_along_width / 2) or
                abs(object_pixel_location_y) > (self.camera_number_of_pixels_along_height / 2)
            ):
                continue

            # Shift X so 0 is the left edge and shift Y so 0 is the top edge which is the standard coordinate scheme\
            # This is the standard top left coordinate scheme for pixel indices
            # See: Figure 39.5 Convention 2 n, m coordinates https://visionbook.mit.edu/imaging_geometry.html#from-meters-to-pixels
            standard_pixel_x = (self.camera_number_of_pixels_along_width / 2) - object_pixel_location_x
            standard_pixel_y = (self.camera_number_of_pixels_along_height / 2) - object_pixel_location_y

            angle_to_object = -1 * degrees(atan2(object_location_relative_to_camera[0], object_location_relative_to_camera[2]))

            object_detection_results.append(
                ObjectDetectionResult(
                    detector_confidence=1.0,
                    tracker_confidence=1.0,
                    x_position=float(standard_pixel_x),
                    y_position=float(standard_pixel_y),
                    object_id=0,
                    class_id=0,
                    angle_to_object=float(angle_to_object),
                )
            )

        self.object_detection_results_publisher.publish(ObjectDetectionResultsList(detection_results=object_detection_results))




def main() -> None:
    rclpy.init()
    simulation_camera_engine_node = SimulationCameraEngine()
    rclpy.spin(simulation_camera_engine_node)
    simulation_camera_engine_node.destroy_node()
    rclpy.shutdown()
