import base64
import hashlib
import json
import os
from collections.abc import Generator
from pathlib import Path
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

from .autopilot_library.utils.constants import (
    BOAT_STATUS_MAPPING,
    QOS_AUTOPILOT_PARAM_CONFIG_PATH,
    TELEMETRY_SERVER_URL,
    BoatStatusPayload,
    TelemetryStatus,
)
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

        self.autopilot_parameters_loaded: bool = False
        self.create_subscription(
            String, "/autopilot_param_config_path", self.autopilot_param_config_path_callback, QOS_AUTOPILOT_PARAM_CONFIG_PATH
        )

        while not self.autopilot_parameters_loaded:
            rclpy.spin_once(self)

        config_hash = hashlib.sha256(
            json.dumps(self.autopilot_parameters, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()

        for response, status in self.get_raw_response_from_telemetry_server("instance_manager/create", self.boat_status_session):
            if status == TelemetryStatus.SUCCESS and isinstance(response, int):
                self.instance_id = response

                user_name = os.environ.get("USER")
                if user_name:
                    url = f"instance_manager/set_user/{self.instance_id}"
                    self.send_raw_data_to_telemetry_server(url, user_name, self.boat_status_session)

                url = f"boat_status/set_mapping/{self.instance_id}"
                self.send_raw_data_to_telemetry_server(url, BOAT_STATUS_MAPPING, self.boat_status_session)

                boat_status: list[Any] = [
                    [self.position.latitude, self.position.longitude],
                    self.autopilot_mode,
                    self.full_autonomy_maneuver,
                    self.speed,
                    (self.velocity_vector[0], self.velocity_vector[1]),
                    self.desired_heading,
                    self.heading,
                    -1, # true wind speed
                    -1, # true wind angle
                    self.apparent_wind_speed,
                    self.apparent_wind_angle,
                    self.desired_sail_angle,
                    self.desired_rudder_angle,
                    self.current_waypoint_index,
                    -1, # distance to next waypoint
                ]
                
                url = f"boat_status/set_fast/{self.instance_id}"
                self.send_raw_data_to_telemetry_server(url, boat_status, self.boat_status_session)

                does_hash_exist: bool = False
                for hash_response, hash_status in self.get_raw_response_from_telemetry_server(
                    f"autopilot_parameters/get_hash_exists/{config_hash}", self.autopilot_parameters_session
                ):
                    if hash_status == TelemetryStatus.SUCCESS:
                        does_hash_exist = hash_response
                        break

                if not does_hash_exist:
                    url = f"autopilot_parameters/set_default/{self.instance_id}"
                    self.send_raw_data_to_telemetry_server(url, self.autopilot_parameters, self.autopilot_parameters_session)
                
                # if hash exists on server, just set the default from that hash
                # to avoid sending over all the parameters again
                else:
                    url = f"autopilot_parameters/set_default_from_hash/{self.instance_id}"
                    self.send_raw_data_to_telemetry_server(url, config_hash, self.autopilot_parameters_session)

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

    
    def autopilot_param_config_path_callback(self, autopilot_param_config_path: String) -> None:
        """
        Callback function for the autopilot parameter config path topic.
        Updates the boat's autopilot parameters from the new config path.

        Parameters
        ----------
        autopilot_param_config_path
            The new config path for the autopilot parameters.
        """

        parameters_path = Path(autopilot_param_config_path.data)
        with open(file=parameters_path, mode="r", encoding="utf-8") as parameters_file:
            self.autopilot_parameters = json.load(parameters_file)

        self.logger.info(f"Loaded autopilot parameters from new config path: {parameters_path}")
        self.autopilot_parameters_loaded = True


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




    def get_raw_response_from_telemetry_server(
        self, route: str, session: requests.Session
    ) -> Generator[tuple[Any, TelemetryStatus], None, None]:
        """
        This is essentially just a helper function to send a GET request to a specific telemetry server route
        and automatically retry if it cannot connect to that route.

        Parameters
        ----------
        route
            The specific route on the telemetry server to send the GET request to.
        session
            The requests session to use for the GET request.

        Yields
        ------
        tuple[Any, TelemetryStatus]
            The raw response from the telemetry server and the status of the request.
        """

        url = urljoin(TELEMETRY_SERVER_URL, route)

        try:
            response = session.get(url=url, timeout=10)
            response.raise_for_status()

            yield response.json(), TelemetryStatus.SUCCESS
        
        except Exception as e:
            self.logger.error(f"Error: {e} \n Could not recieve data with telemetry server route {route}, retrying...")
            yield from self.get_raw_response_from_telemetry_server(route, session)




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

        url = urljoin(TELEMETRY_SERVER_URL, route)
        response = None

        try:
            if isinstance(data, (float, int)):
                url += f"/{data}"
                response = session.post(url=url, timeout=10)

            elif isinstance(data, str):
                if " " in data:
                    data = data.replace(" ", "_")
                
                url += f"/{data}"
                response = session.post(url=url, timeout=10)

            elif isinstance(data, list):
                response = session.post(url=url, json=data, timeout=10)
            
            else:
                response = session.post(url=url, json=json.dumps(data, separators=(",", ":"), indent=False), timeout=10)

            response.raise_for_status()

        except Exception as e:
            self.logger.error(f"Error: {e} \n Could not send data with telemetry server route {route}, retrying...")
            if response is not None:
                self.logger.error(f"Response content: {response.content}")
            
            self.send_raw_data_to_telemetry_server(route, data, session)



    def update_boat_status(self) -> None:
        """
        Gathers the boat's current status and sends it to the telemetry server.

        Note
        ----
        See ``constants.py`` for the mapping of the boat status list and what each index represents.

        This is bugged! You need to account for the velocity vector being measured globally
        rather than the apparent wind vector which is measured locally.
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

        boat_status: list[Any] = [
            [self.position.latitude, self.position.longitude],
            self.autopilot_mode,
            self.full_autonomy_maneuver,
            self.speed,
            (self.velocity_vector[0], self.velocity_vector[1]),
            self.desired_heading,
            self.heading,
            self.true_wind_speed,
            self.true_wind_angle,
            self.apparent_wind_speed,
            self.apparent_wind_angle,
            self.desired_sail_angle,
            self.desired_rudder_angle,
            self.current_waypoint_index,
            distance_to_next_waypoint,
        ]

        self.send_raw_data_to_telemetry_server(
            f"boat_status/set_fast/{self.instance_id}",
            boat_status,
            self.boat_status_session,
        )



    def update_waypoints_from_telemetry(self) -> None:
        """Updates the boat's current waypoints from the telemetry server and publishes them over ROS."""

        route = urljoin(TELEMETRY_SERVER_URL, f"waypoints/get_new/{self.instance_id}")

        for new_waypoints, status in self.get_raw_response_from_telemetry_server(route, self.waypoints_session):
            if status == TelemetryStatus.SUCCESS and isinstance(new_waypoints, list):
                self.current_waypoints = new_waypoints
                
                # update the ROS2 topic so that the autopilot actually knows what the new waypoints are
                waypoints_nav_sat_fix_list = [
                    NavSatFix(latitude=waypoint[0], longitude=waypoint[1]) for waypoint in self.current_waypoints
                ]
                self.waypoints_list_publisher.publish(WaypointList(waypoints=waypoints_nav_sat_fix_list))
                
                break




    def update_autopilot_parameters_from_telemetry(self) -> None:
        """Updates the boat's current autopilot parameters from the telemetry server and publishes them over ROS."""

        route = urljoin(TELEMETRY_SERVER_URL, f"autopilot_parameters/get_new/{self.instance_id}")
        for new_autopilot_parameters, status in self.get_raw_response_from_telemetry_server(
            route, self.autopilot_parameters_session
        ):
            if status == TelemetryStatus.SUCCESS and isinstance(new_autopilot_parameters, dict):
                self.autopilot_parameters = new_autopilot_parameters

                # update the ROS2 topic so that the autopilot actually knows what the new parameters are
                serialized_autopilot_parameters_string = String(data=json.dumps(self.autopilot_parameters))
                self.autopilot_parameters_publisher.publish(serialized_autopilot_parameters_string)

                break




def main() -> None:
    rclpy.init()
    telemetry_node = TelemetryNode()
    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.shutdown()
