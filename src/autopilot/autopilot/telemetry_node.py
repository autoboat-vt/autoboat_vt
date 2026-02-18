#!usr/bin/python3

import base64
import hashlib
import json
import os
import time
from typing import Any
from urllib.parse import urljoin

import cv2
import numpy as np
import numpy.typing as npt
import rclpy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool, Float32, Int32, String

from autoboat_msgs.msg import VESCTelemetryData, WaypointList

from .autopilot_library.utils.constants import CONFIG_DIRECTORY, TELEMETRY_SERVER_URL
from .autopilot_library.utils.position import Position
from .autopilot_library.utils.utils_function_library import (
    cartesian_vector_to_polar,
    get_distance_between_positions,
)


class TelemetryNode(Node):
    """
    This ROS node collects information from multiple topics and transmits it to
    the groundstation via the telemetry server. It also receives commands and
    parameters from the groundstation to control the autopilot.

    Inherits
    -------
    ``Node``
    """

    def __init__(self) -> None:
        super().__init__("telemetry")

        # If these values aren't changing then the ros node or telemetry server
        # thats supposed to be sending these values may not be working correctly
        self.autopilot_mode: str = "N/A"
        self.full_autonomy_maneuver: str = "N/A"

        self.current_waypoints: list[tuple[float, float]] = []
        self.current_waypoint_index: int = 0

        # see https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html
        self.position = NavSatFix(latitude=0.0, longitude=0.0)

        self.velocity_vector: npt.NDArray[np.float64] = np.zeros(2, dtype=np.float64)
        self.speed: float = 0.0

        self.heading: float = 0.0
        self.desired_heading: float = 0.0

        self.desired_sail_angle: float = 0.0
        self.desired_rudder_angle: float = 0.0

        self.apparent_wind_vector: npt.NDArray[np.float64] = np.zeros(2, dtype=np.float64)
        self.apparent_wind_speed: float = 0.0
        self.apparent_wind_angle: float = 0.0

        self.base64_encoded_current_rgb_image: str = None

        self.vesc_telemetry_data_rpm: float = 0.0
        self.vesc_telemetry_data_duty_cycle: float = 0.0
        self.vesc_telemetry_data_amp_hours: float = 0.0
        self.vesc_telemetry_data_amp_hours_charged: float = 0.0
        self.vesc_telemetry_data_current_to_vesc: float = 0.0
        self.vesc_telemetry_data_voltage_to_motor: float = 0.0
        self.vesc_telemetry_data_voltage_to_vesc: float = 0.0
        self.vesc_telemetry_data_wattage_to_motor: float = 0.0
        self.vesc_telemetry_data_time_since_vesc_startup_in_ms: float = 0.0
        self.vesc_telemetry_data_motor_temperature: float = 0.0
        self.vesc_telemetry_data_vesc_temperature: float = 0.0

        self.logger = self.get_logger()

        self.boat_status_session = requests.Session()
        self.autopilot_parameters_session = requests.Session()
        self.waypoints_session = requests.Session()

        parameters_path = CONFIG_DIRECTORY / "sailboat_default_parameters.json"
        with open(parameters_path, "r", encoding="utf-8") as parameters_file:
            self.autopilot_parameters: dict[str, dict[str, Any]] = json.load(parameters_file)

        config_hash = hashlib.sha256(
            json.dumps(self.autopilot_parameters, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()

        while True:
            new_id = self.get_raw_response_from_telemetry_server("instance_manager/create", session=self.boat_status_session)
            if isinstance(new_id, int):
                self.instance_id = new_id

                user_name = os.environ.get("USER")
                if not user_name:
                    user_name = "Unknown User"

                url = f"instance_manager/set_user/{self.instance_id}"
                self.send_raw_data_to_telemetry_server(url, user_name, self.boat_status_session)

                does_hash_exist = self.get_raw_response_from_telemetry_server(
                    f"autopilot_parameters/get_hash_exists/{config_hash}", session=self.autopilot_parameters_session
                )
                if not does_hash_exist:
                    url = f"autopilot_parameters/set_default/{self.instance_id}"
                    self.send_raw_data_to_telemetry_server(url, self.autopilot_parameters, self.autopilot_parameters_session)
                
                # if hash exists on server, just set the default from that hash
                # to avoid sending over all the parameters again
                else:
                    url = f"autopilot_parameters/set_default_from_hash/{self.instance_id}/{config_hash}"
                    self.send_raw_data_to_telemetry_server(url, "", self.autopilot_parameters_session)

                self.logger.info(f"Telemetry node instance ID: {self.instance_id}")
                self.logger.info(f"Using hash: {config_hash}")
                break

        self.create_timer(0.01, self.update_boat_status)  # 10 ms
        self.create_timer(0.5, self.update_waypoints_from_telemetry)  # 500 ms
        self.create_timer(0.5, self.update_autopilot_parameters_from_telemetry)  # 500 ms

        self.cv_bridge = CvBridge()

        self.autopilot_parameters_publisher = self.create_publisher(String, "/autopilot_parameters", 10)
        self.sensors_parameters_publisher = self.create_publisher(String, "/sensors_parameters", 10)

        self.waypoints_list_publisher = self.create_publisher(WaypointList, "/waypoints_list", 10)
        self.create_subscription(Float32, "/desired_heading", self.desired_heading_callback, 10)

        self.create_subscription(Int32, "/current_waypoint_index", self.current_waypoint_index_callback, 10)
        self.create_subscription(String, "/full_autonomy_maneuver", self.full_autonomy_maneuver_callback, qos_profile_sensor_data)
        self.create_subscription(String, "/autopilot_mode", self.autopilot_mode_callback, qos_profile_sensor_data)

        self.create_subscription(Float32, "/desired_sail_angle", self.desired_sail_angle_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, "/desired_rudder_angle", self.desired_rudder_angle_callback, qos_profile_sensor_data)

        self.create_subscription(Image, "/camera/camera/color/image_raw", self.camera_rgb_image_callback, qos_profile_sensor_data)

        self.create_subscription(NavSatFix, "/position", self.position_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, "/velocity", self.velocity_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, "/heading", self.heading_callback, qos_profile_sensor_data)
        self.create_subscription(Vector3, "/apparent_wind_vector", self.apparent_wind_vector_callback, qos_profile_sensor_data)
        self.create_subscription(
            VESCTelemetryData, "/vesc_telemetry_data", self.vesc_telemetry_data_callback, qos_profile_sensor_data
        )



    def camera_rgb_image_callback(self, camera_rgb_image: Image) -> None:
        """
        Callback function for the camera RGB image topic. Updates the boat's current RGB image in base64 encoded format.

        Note
        Refer to this stack overflow post: https://stackoverflow.com/questions/40928205/python-opencv-image-to-byte-string-for-json-transfer

        Parameters
        ----------
        camera_rgb_image
            The current RGB image from the boat's camera.
        """

        rgb_image_cv = self.cv_bridge.imgmsg_to_cv2(camera_rgb_image, desired_encoding="rgb8")
        rgb_image_cv = rgb_image_cv[80:1200, 40:680]  # crop the image to 640,640
        _, buffer = cv2.imencode(".jpg", rgb_image_cv)

        # swap red and blue channels for correction
        red = rgb_image_cv[:, :, 2].copy()
        blue = rgb_image_cv[:, :, 0].copy()
        rgb_image_cv[:, :, 0] = red
        rgb_image_cv[:, :, 2] = blue

        cv2.imwrite("test.jpg", rgb_image_cv)

        self.base64_encoded_current_rgb_image = base64.b64encode(buffer).decode()


    def position_callback(self, position: NavSatFix) -> None:
        """
        Callback function for the position topic. Updates the boat's current position.

        Parameters
        ----------
        position
            The current position of the boat.
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

        self.velocity_vector = np.array([velocity_vector.linear.x, velocity_vector.linear.y], dtype=np.float64)
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

        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y], dtype=np.float64)
        self.apparent_wind_speed, self.apparent_wind_angle = cartesian_vector_to_polar(
            apparent_wind_vector.x, apparent_wind_vector.y
        )



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
        Callback function for the VESC telemetry data topic. Updates the boat's current VESC telemetry data.
        
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
        Callback function for the full autonomy maneuver topic. Updates the boat's current full autonomy maneuver.
        
        Parameters
        ----------
        full_autonomy_maneuver
            The current full autonomy maneuver of the boat.
        """

        self.full_autonomy_maneuver = full_autonomy_maneuver.data


    def autopilot_mode_callback(self, autopilot_mode: String) -> None:
        """
        Callback function for the autopilot mode topic. Updates the boat's current autopilot mode.

        Parameters
        ----------
        autopilot_mode
            The current autopilot mode of the boat.
        """

        self.autopilot_mode = autopilot_mode.data


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
        """
        Callback function for the desired rudder angle topic. Updates the boat's desired rudder angle.
        
        Parameters
        ----------
        desired_rudder_angle
            The desired rudder angle of the boat.
        """

        self.desired_rudder_angle = desired_rudder_angle.data


    def should_terminate_callback(self, msg: Bool) -> None:
        """
        Callback function for the should terminate topic. Shuts down the ROS node if the message data is ``True``.

        Parameters
        ----------
        msg
            The message indicating whether to terminate the node.
        """

        if msg.data:
            rclpy.shutdown()




    def get_raw_response_from_telemetry_server(self, route: str, session: requests.Session) -> int | dict | list | bool:
        """
        This is essentially just a helper function to send a GET request to a specific telemetry server route
        and automatically retry if it cannot connect to that route.

        Parameters
        ----------
        route
            The specific route on the telemetry server to send the GET request to.
        session
            The requests session to use for the GET request.

        Returns
        -------
        Any
            The JSON response from the telemetry server.
        """

        try:
            return session.get(urljoin(TELEMETRY_SERVER_URL, route), timeout=10).json()

        except Exception:
            self.logger.error(f"Could not connect to telemetry server route {route}, retrying...")
            time.sleep(0.5)
            return self.get_raw_response_from_telemetry_server(route, session)




    def send_raw_data_to_telemetry_server(self, route: str, data: float | str | list | dict, session: requests.Session) -> None:
        """
        This is essentially just a helper function to send a POST request to a specific telemetry server route
        and automatically retry if it cannot connect to that route.

        Parameters
        ----------
        route
            The specific route on the telemetry server to send the POST request to.
        data
            The data to send in the POST request.
        session
            The requests session to use for the POST request.
        """

        try:
            url = urljoin(TELEMETRY_SERVER_URL, route)
            if isinstance(data, (float, str)):
                session.post(urljoin(url, data), timeout=10)
            
            else:
                session.post(url=url, json=data, timeout=10)

        except Exception:
            self.logger.error(f"Could not connect to telemetry server route {route}, retrying...")
            time.sleep(0.5)
            self.send_raw_data_to_telemetry_server(route, data, session)




    def update_boat_status(self) -> None:
        """
        Gathers data from the autopilot and sensors through ROS, and then this node
        makes an API call to send that data over to the groundstation so that it can view what is going on.

        Note
        -------
        THIS IS BUGGED. YOU NEED TO ACCOUNT FOR THE VELOCITY VECTOR BEING MEASURED GLOBALLY
        RATHER THAN THE APPARENT WIND VECTOR WHICH IS MEASURED LOCALLY.
        """

        true_wind_vector = self.apparent_wind_vector + self.velocity_vector
        self.true_wind_speed, self.true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])

        if self.current_waypoints != []:
            current_position = Position(self.position.latitude, self.position.longitude)
            next_waypoint_position = Position(
                self.current_waypoints[self.current_waypoint_index][0], self.current_waypoints[self.current_waypoint_index][1]
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

        try:
            url = urljoin(TELEMETRY_SERVER_URL, f"boat_status/set/{self.instance_id}")
            self.boat_status_session.post(url=url, json=boat_status_dictionary)

        except Exception as e:
            self.logger.error(f"Could not connect to telemetry server to send boat status update. \nError: {e}")



    def update_waypoints_from_telemetry(self) -> None:
        """
        Makes an API call to gather the waypoints that the groundstation set,
        and then publishes the waypoints over ROS so that the autopilot can see them.
        """

        route = urljoin(TELEMETRY_SERVER_URL, f"waypoints/get_new/{self.instance_id}")
        new_waypoints = self.get_raw_response_from_telemetry_server(route=route, session=self.waypoints_session)

        if new_waypoints == {}:
            self.logger.info("No new waypoints received from telemetry server.")
            return

        self.logger.info(f"Received waypoints: {new_waypoints}")

        if not isinstance(new_waypoints, list):
            self.logger.error(f"Invalid waypoints format: {new_waypoints}. Expected a list.")
            return

        self.current_waypoints = new_waypoints
        waypoints_nav_sat_fix_list = [
            NavSatFix(latitude=waypoint[0], longitude=waypoint[1]) for waypoint in self.current_waypoints
        ]
        self.waypoints_list_publisher.publish(WaypointList(waypoints=waypoints_nav_sat_fix_list))




    def update_autopilot_parameters_from_telemetry(self) -> None:
        """
        Makes an API call to gather the autopilot parameters that the groundstation set,
        and then publishes the autopilot parameters over ROS so that the autopilot can see them.
        """

        route = urljoin(TELEMETRY_SERVER_URL, f"autopilot_parameters/get_new/{self.instance_id}")
        new_autopilot_parameters = self.get_raw_response_from_telemetry_server(
            route=route, session=self.autopilot_parameters_session
        )

        if new_autopilot_parameters == {}:
            return

        for new_autopilot_parameter_name, new_autopilot_parameters_value in new_autopilot_parameters.items():
            self.autopilot_parameters[new_autopilot_parameter_name] = new_autopilot_parameters_value

        # update the ROS2 topic so that the autopilot actually knows what the new parameters are
        serialized_autopilot_parameters_string = String(data=json.dumps(self.autopilot_parameters))
        self.autopilot_parameters_publisher.publish(serialized_autopilot_parameters_string)




def main() -> None:
    rclpy.init()
    telemetry_node = TelemetryNode()
    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.shutdown()
