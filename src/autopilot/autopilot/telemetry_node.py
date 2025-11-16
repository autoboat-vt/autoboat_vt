#!usr/bin/python3

# TODO: ADD TRACKING OF THE ACTUAL SAIL AND RUDDER ANGLES TO THE TELEMETRY DATA

from .autopilot_library.utils import *

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Bool, String, Int32
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import NavSatFix, Image
from autoboat_msgs.msg import WaypointList, VESCTelemetryData
from cv_bridge import CvBridge
from urllib.parse import urljoin

import numpy as np
import time, json, requests
import cv2
import yaml
import os
import base64
from typing import Any

# don't put '/' at the end of the URL
TELEMETRY_SERVER_URL = "https://vt-autoboat-telemetry.uk:8443"


class TelemetryNode(Node):
    """
    This ROS node collects information from multiple topics and transmits it to
    the groundstation via the telemetry server. It also receives commands and
    parameters from the groundstation to control the autopilot.

    Inherits
    -------
    `Node`
    """

    def __init__(self) -> None:
        super().__init__("telemetry")

        # DEFAULT VALUES IN CASE THESE ARE NEVER SENT THROUGH ROS OR THE TELEMETRY SERVER
        # If these values aren't changing then the ros node or telemetry server thats supposed to be sending these values may not be working correctly
        self.current_waypoints_list: list[tuple[float, float]] = []
        self.current_waypoint_index = 0
        self.position = NavSatFix(latitude=0.0, longitude=0.0)

        self.autopilot_mode = "N/A"
        self.full_autonomy_maneuver = "N/A"

        self.velocity_vector = np.array([0.0, 0.0])
        self.speed = 0.0
        self.heading = 0.0
        self.desired_heading = 0.0

        self.apparent_wind_vector = np.array([0.0, 0.0])
        self.apparent_wind_speed = 0.0
        self.apparent_wind_angle = 0.0

        self.base64_encoded_current_rgb_image = None

        self.desired_sail_angle = 0.0
        self.desired_rudder_angle = 0.0

        self.vesc_telemetry_data_rpm = 0
        self.vesc_telemetry_data_duty_cycle = 0
        self.vesc_telemetry_data_amp_hours = 0
        self.vesc_telemetry_data_amp_hours_charged = 0
        self.vesc_telemetry_data_current_to_vesc = 0
        self.vesc_telemetry_data_voltage_to_motor = 0
        self.vesc_telemetry_data_voltage_to_vesc = 0
        self.vesc_telemetry_data_wattage_to_motor = 0
        self.vesc_telemetry_data_time_since_vesc_startup_in_ms = 0
        self.vesc_telemetry_data_motor_temperature = 0
        self.vesc_telemetry_data_vesc_temperature = 0

        self.logger = self.get_logger()

        self.boat_status_session = requests.Session()
        self.autopilot_parameters_session = requests.Session()
        self.waypoints_session = requests.Session()

        while True:
            new_id = self.get_raw_response_from_telemetry_server("instance_manager/create", session=self.boat_status_session)
            if isinstance(new_id, int):
                self.instance_id = new_id
                self.boat_status_session.post(urljoin(TELEMETRY_SERVER_URL, f"instance_manager/set_user/{self.instance_id}/{os.environ['USER']}"))
                self.logger.info(f"Created new telemetry server instance with ID {self.instance_id}")
                break

        self.create_timer(0.01, self.update_boat_status) # 10 ms
        self.create_timer(0.5, self.update_waypoints_from_telemetry)
        self.create_timer(0.5, self.update_autopilot_parameters_from_telemetry)

        self.cv_bridge = CvBridge()

        current_folder_path = os.path.dirname(os.path.realpath(__file__))
        print(current_folder_path)
        with open(current_folder_path + "/config/sailboat_default_parameters.yaml", "r") as stream:
            self.autopilot_parameters_dictionary: dict = yaml.safe_load(stream)

        # region DECLARE ROS2 PUBLISHERS AND SUBSCRIBERS
        self.autopilot_parameters_publisher = self.create_publisher(msg_type=String, topic="/autopilot_parameters", qos_profile=10)

        self.sensors_parameters_publisher = self.create_publisher(msg_type=String, topic="/sensors_parameters", qos_profile=10)

        self.waypoints_list_publisher = self.create_publisher(msg_type=WaypointList, topic="/waypoints_list", qos_profile=10)

        self.desired_heading_listener = self.create_subscription(
            msg_type=Float32, topic="/desired_heading", callback=self.desired_heading_callback, qos_profile=10
        )

        self.current_waypoint_index_listener = self.create_subscription(
            msg_type=Int32, topic="/current_waypoint_index", callback=self.current_waypoint_index_callback, qos_profile=10
        )

        self.full_autonomy_maneuver_listener = self.create_subscription(
            msg_type=String,
            topic="/full_autonomy_maneuver",
            callback=self.full_autonomy_maneuver_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.autopilot_mode_listener = self.create_subscription(
            msg_type=String,
            topic="/autopilot_mode",
            callback=self.autopilot_mode_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.position_listener = self.create_subscription(
            msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=qos_profile_sensor_data
        )

        self.velocity_listener = self.create_subscription(
            msg_type=Twist, topic="/velocity", callback=self.velocity_callback, qos_profile=qos_profile_sensor_data
        )

        self.heading_listener = self.create_subscription(
            msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=qos_profile_sensor_data
        )

        self.apparent_wind_vector_listener = self.create_subscription(
            msg_type=Vector3,
            topic="/apparent_wind_vector",
            callback=self.apparent_wind_vector_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.camera_rgb_image_listener = self.create_subscription(
            msg_type=Image,
            topic="/camera/camera/color/image_raw",
            callback=self.camera_rgb_image_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.vesc_telemetry_data_listener = self.create_subscription(
            msg_type=VESCTelemetryData,
            topic="/vesc_telemetry_data",
            callback=self.vesc_telemetry_data_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.desired_sail_angle_listener = self.create_subscription(
            msg_type=Float32,
            topic="/desired_sail_angle",
            callback=self.desired_sail_angle_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.desired_rudder_angle_listener = self.create_subscription(
            msg_type=Float32,
            topic="/desired_rudder_angle",
            callback=self.desired_rudder_angle_callback,
            qos_profile=qos_profile_sensor_data,
        )
        # endregion DECLARE ROS2 PUBLISHERS AND SUBSCRIBERS

    def position_callback(self, position: NavSatFix) -> None:
        """
        Callback function for the position topic. Updates the boat's current position.

        Parameters
        ----------
        position
            The current GPS position of the boat.
        """

        self.position = position

    def velocity_callback(self, velocity_vector: Twist) -> None:
        """
        Callback function for the velocity topic. Updates the boat's current velocity vector and speed.

        Parameters
        ----------
        velocity_vector
            The current velocity vector of the boat.
        """

        self.velocity_vector = np.array([velocity_vector.linear.x, velocity_vector.linear.y])
        self.speed = np.linalg.norm(self.velocity_vector)

    def heading_callback(self, heading: Float32) -> None:
        """
        Callback function for the heading topic. Updates the boat's current heading.
        
        Parameters
        ----------
        heading
            The current heading of the boat.
        """

        self.heading = heading.data

    def apparent_wind_vector_callback(self, apparent_wind_vector: Vector3) -> None:
        """
        Callback function for the apparent wind vector topic. Updates the boat's current apparent wind vector, speed, and angle.

        Parameters
        ----------
        apparent_wind_vector
            The current apparent wind vector of the boat.
        """

        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y])
        self.apparent_wind_speed, self.apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector.x, apparent_wind_vector.y)

    def desired_heading_callback(self, desired_heading: Float32) -> None:
        """
        Callback function for the desired heading topic. Updates the boat's desired heading.

        Parameters
        ----------
        desired_heading
            The desired heading of the boat.
        """

        self.desired_heading = desired_heading.data

    def vesc_telemetry_data_callback(self, vesc_telemetry_data: VESCTelemetryData) -> None:
        """
        Callback function for the VESC telemetry data topic. Updates the boat's VESC telemetry data.
        
        Parameters
        ----------
        vesc_telemetry_data
            The current VESC telemetry data of the boat.
        """

        self.vesc_telemetry_data_rpm = vesc_telemetry_data.rpm
        self.vesc_telemetry_data_duty_cycle = vesc_telemetry_data.duty_cycle
        self.vesc_telemetry_data_amp_hours = vesc_telemetry_data.amp_hours
        self.vesc_telemetry_data_amp_hours_charged = vesc_telemetry_data.amp_hours_charged
        self.vesc_telemetry_data_current_to_vesc = vesc_telemetry_data.current_to_vesc
        self.vesc_telemetry_data_voltage_to_motor = vesc_telemetry_data.voltage_to_motor
        self.vesc_telemetry_data_voltage_to_vesc = vesc_telemetry_data.voltage_to_vesc
        self.vesc_telemetry_data_wattage_to_motor = vesc_telemetry_data.wattage_to_motor
        self.vesc_telemetry_data_time_since_vesc_startup_in_ms = vesc_telemetry_data.time_since_vesc_startup_in_ms
        self.vesc_telemetry_data_motor_temperature = vesc_telemetry_data.motor_temperature
        self.vesc_telemetry_data_vesc_temperature = vesc_telemetry_data.vesc_temperature

    def current_waypoint_index_callback(self, current_waypoint_index: Int32) -> None:
        """
        Callback function for the current waypoint index topic. Updates the boat's current waypoint index.
        
        Parameters
        ----------
        current_waypoint_index
            The current waypoint index of the boat.
        """
    
        self.current_waypoint_index = current_waypoint_index.data

    def full_autonomy_maneuver_callback(self, full_autonomy_maneuver: String) -> None:
        """
        Callback function for the full autonomy maneuver topic. Updates the boat's full autonomy maneuver.

        Parameters
        ----------
        full_autonomy_maneuver
            The current full autonomy maneuver of the boat.
        """

        self.full_autonomy_maneuver = full_autonomy_maneuver.data

    def autopilot_mode_callback(self, autopilot_mode: String) -> None:
        """
        Callback function for the autopilot mode topic. Updates the boat's autopilot mode.
        
        Parameters
        ----------
        autopilot_mode
            The current autopilot mode of the boat.
        """

        self.autopilot_mode = autopilot_mode.data

    def camera_rgb_image_callback(self, camera_rgb_image: Image) -> None:
        """
        Refer to this stack overflow post: https://stackoverflow.com/questions/40928205/python-opencv-image-to-byte-string-for-json-transfer
        
        Parameters
        ----------
        camera_rgb_image
            The current RGB image from the boat's camera.
        """

        rgb_image_cv = self.cv_bridge.imgmsg_to_cv2(camera_rgb_image, "rgb8")
        rgb_image_cv = rgb_image_cv[80:1200, 40:680]  # crop the image to 640,640
        retval, buffer = cv2.imencode(".jpg", rgb_image_cv)

        # swap red and blue channels for correction
        red = rgb_image_cv[:, :, 2].copy()
        blue = rgb_image_cv[:, :, 0].copy()
        rgb_image_cv[:, :, 0] = red
        rgb_image_cv[:, :, 2] = blue

        cv2.imwrite("test.jpg", rgb_image_cv)

        self.base64_encoded_current_rgb_image = base64.b64encode(buffer).decode()

    def desired_sail_angle_callback(self, desired_sail_angle: Float32) -> None:
        """
        Callback function for the desired sail angle topic. Updates the boat's desired sail angle.

        Parameters
        ----------
        desired_sail_angle
            The desired sail angle of the boat.
        """

        self.desired_sail_angle = desired_sail_angle.data

    def desired_rudder_angle_callback(self, desired_rudder_angle: Float32) -> None:
        self.desired_rudder_angle = desired_rudder_angle.data

    def should_terminate_callback(self, msg: Bool) -> None:
        if msg.data == False:
            return
        rclpy.shutdown()

    def get_raw_response_from_telemetry_server(self, route: str, session: requests.Session = None) -> Any:
        """
        This is essentially just a helper function to send a GET request to a specific telemetry server route and automatically retry if it cannot connect to that route.

        Parameters
        ----------
        route
            The specific route on the telemetry server to send the GET request to.
        session
            An optional requests.Session object to use for the GET request.

        Returns
        -------
        Any
            The JSON response from the telemetry server.
        """

        try:
            return session.get(urljoin(TELEMETRY_SERVER_URL, route), timeout=10).json()

        except Exception as e:
            self.logger.info(f"Could not connect to telemetry server route {route}, retrying... \nError: {e}")
            time.sleep(0.5)
            return self.get_raw_response_from_telemetry_server(route)

    def update_boat_status(self) -> None:
        """
        Gathers data from the autopilot and sensors through ROS, and then this node makes an API call to send that data over to the groundstation so that it can view what is going on.

        THIS IS BUGGED. YOU NEED TO ACCOUNT FOR THE VELOCITY VECTOR BEING MEASURED GLOBALLY RATHER THAN THE APPARENT WIND VECTOR WHICH IS MEASURED LOCALLY.
        """

        true_wind_vector = self.apparent_wind_vector + self.velocity_vector
        self.true_wind_speed, self.true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])

        if self.current_waypoints_list != []:
            current_position = Position(self.position.latitude, self.position.longitude)
            next_waypoint_position = Position(
                self.current_waypoints_list[self.current_waypoint_index][0],
                self.current_waypoints_list[self.current_waypoint_index][1],
            )
            distance_to_next_waypoint = get_distance_between_positions(current_position, next_waypoint_position)

        else:
            distance_to_next_waypoint = 0.0

        # boat_status_dict = {
        #     "position": (self.position.latitude, self.position.longitude),
        #     "state": self.autopilot_mode,
        #     "full_autonomy_maneuver": self.full_autonomy_maneuver,
        #     "speed": self.speed,
        #     "velocity_vector": (self.velocity_vector[0], self.velocity_vector[1]),
        #     "bearing": self.desired_heading, "heading": self.heading,
        #     "true_wind_speed": self.true_wind_speed, "true_wind_angle": self.true_wind_angle,
        #     "apparent_wind_speed": self.apparent_wind_speed, "apparent_wind_angle": self.apparent_wind_angle,
        #     "sail_angle": self.desired_sail_angle, "rudder_angle": self.desired_rudder_angle,
        #     "current_waypoint_index": self.current_waypoint_index,
        #     "parameters": self.autopilot_parameters_dict,
        #     "current_camera_image": self.base64_encoded_current_rgb_image,

        #     "vesc_telemetry_data_rpm": self.vesc_telemetry_data_rpm,
        #     "vesc_telemetry_data_duty_cycle": self.vesc_telemetry_data_duty_cycle,
        #     "vesc_telemetry_data_amp_hours": self.vesc_telemetry_data_amp_hours,
        #     "vesc_telemetry_data_amp_hours_charged": self.vesc_telemetry_data_amp_hours_charged,
        #     "vesc_telemetry_data_current_to_vesc": self.vesc_telemetry_data_current_to_vesc,
        #     "vesc_telemetry_data_voltage_to_motor": self.vesc_telemetry_data_voltage_to_motor,
        #     "vesc_telemetry_data_voltage_to_vesc": self.vesc_telemetry_data_voltage_to_vesc,
        #     "vesc_telemetry_data_wattage_to_motor": self.vesc_telemetry_data_wattage_to_motor,
        #     "vesc_telemetry_data_time_since_vesc_startup_in_ms": self.vesc_telemetry_data_time_since_vesc_startup_in_ms,
        #     "vesc_telemetry_data_motor_temperature": self.vesc_telemetry_data_motor_temperature,
        #     "vesc_telemetry_data_vesc_temperature": self.vesc_telemetry_data_vesc_temperature
        # }

        boat_status_dictionary = {
            "position": [self.position.latitude, self.position.longitude],
            "state": self.autopilot_mode,
            "full_autonomy_maneuver": self.full_autonomy_maneuver,
            "speed": self.speed,
            "velocity_vector": (self.velocity_vector[0], self.velocity_vector[1]),
            "bearing": self.desired_heading,
            "heading": self.heading,
            "true_wind_speed": self.true_wind_speed,
            "true_wind_angle": self.true_wind_angle,
            "apparent_wind_speed": self.apparent_wind_speed,
            "apparent_wind_angle": self.apparent_wind_angle,
            "sail_angle": self.desired_sail_angle,
            "rudder_angle": self.desired_rudder_angle,
            "current_waypoint_index": self.current_waypoint_index,
            "distance_to_next_waypoint": distance_to_next_waypoint,
        }

        start_time = time.time()
        try:
            self.boat_status_session.post(urljoin(TELEMETRY_SERVER_URL, f"boat_status/set/{self.instance_id}"), json=boat_status_dictionary)
        except Exception as e:
            self.logger.info(f"Could not connect to telemetry server to send boat status update. \nError: {e}")
            
        self.logger.info(f"{time.time() - start_time}")

    def update_waypoints_from_telemetry(self) -> None:
        """
        Makes an API call to gather the waypoints that the groundstation set,
        and then publishes the waypoints over ROS so that the autopilot can see them.
        """

        new_waypoints_list = self.get_raw_response_from_telemetry_server(urljoin(TELEMETRY_SERVER_URL, f"waypoints/get_new/{self.instance_id}"), session=self.waypoints_session)
        self.logger.info(f"{new_waypoints_list}")

        if new_waypoints_list == {}:
            self.logger.info("No new waypoints received from telemetry server.")
            return

        if not isinstance(new_waypoints_list, list):
            self.logger.info(f"Invalid waypoints format: {new_waypoints_list}. Expected a list.")
            return

        # parse waypoints
        waypoints_nav_sat_fix_list = []
        for waypoint in new_waypoints_list:
            try:
                if not (isinstance(waypoint, list) or isinstance(waypoint, tuple)):
                    raise TypeError("Waypoint is not a list or tuple.")
                
                if len(waypoint) != 2:
                    raise ValueError("Waypoint does not have exactly 2 elements.")
                else:
                    latitude, longitude = waypoint

                if not (isinstance(latitude, float) or isinstance(latitude, int)):
                    raise TypeError("Latitude is not a float or int.")
                if not (isinstance(longitude, float) or isinstance(longitude, int)):
                    raise TypeError("Longitude is not a float or int.")
                
                waypoints_nav_sat_fix_list.append(NavSatFix(latitude=float(latitude), longitude=float(longitude)))
                
            except (ValueError, TypeError) as e:
                self.logger.info(f"Invalid waypoint format: {waypoint}. Error: {e}")
                continue

        self.current_waypoints_list = new_waypoints_list
        self.waypoints_list_publisher.publish(WaypointList(waypoints=waypoints_nav_sat_fix_list))

    def update_autopilot_parameters_from_telemetry(self) -> None:
        """
        Makes an API call to gather the autopilot parameters that the groundstation set,
        and then publishes the autopilot parameters over ROS so that the autopilot can see them
        """

        new_autopilot_parameters_dictionary = self.get_raw_response_from_telemetry_server(
            urljoin(TELEMETRY_SERVER_URL, f"autopilot_parameters/get_new/{self.instance_id}"), session=self.autopilot_parameters_session
        )

        if new_autopilot_parameters_dictionary == {}:
            return

        for new_autopilot_parameter_name, new_autopilot_parameters_value in new_autopilot_parameters_dictionary.items():
            self.autopilot_parameters_dictionary[new_autopilot_parameter_name] = new_autopilot_parameters_value

        # if this is a new set of parameters, then update the ROS2 topic so that the autopilot actually knows what the new parameters are
        serialized_autopilot_parameters_string = String(data=json.dumps(self.autopilot_parameters_dictionary))
        self.autopilot_parameters_publisher.publish(serialized_autopilot_parameters_string)


def main():
    rclpy.init()
    telemetry_node = TelemetryNode()
    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
