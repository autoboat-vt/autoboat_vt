#!usr/bin/python3

# TODO: ADD TRACKING OF THE ACTUAL SAIL AND RUDDER ANGLES TO THE TELEMETRY DATA

from .autopilot_library.utils import *

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool, String, Int32
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import NavSatFix, Image
from autoboat_msgs.msg import WaypointList, VESCTelemetryData
from cv_bridge import CvBridge

import numpy as np
import time, json, requests
import cv2
import yaml
import os
import base64
import geopy.distance


# TELEMETRY_SERVER_URL = 'http://54.165.159.151:8080/'
TELEMETRY_SERVER_URL = 'http://3.138.35.188:5000/'


def get_distance_to_waypoint(
        cur_position: list[float, float], next_waypoint: list[float, float]
    ) -> float:
        """
        Calculates the distance to the next waypoint from the current position using geopy.

        Parameters
        ----------
        cur_position
            The current position of the boat as a list of latitude and longitude.
        next_waypoint
            The next waypoint as a list of latitude and longitude.

        Returns
        -------
        float
            The distance to the next waypoint in meters.
        """

        if next_waypoint:
            return geopy.distance.geodesic(next_waypoint, cur_position).m
        else:
            return geopy.distance.Distance(0.0).m

 
 
 
 
 
class TelemetryNode(Node):

    # NOTE: All units are in standard SI units and angle is measured in degrees
    
    def __init__(self):
        super().__init__("telemetry")

        current_folder_path = os.path.dirname(os.path.realpath(__file__))
        with open(current_folder_path + "/config/sailboat_default_parameters.yaml", 'r') as stream:
            self.autopilot_parameters_dictionary: dict = yaml.safe_load(stream)


        self.create_timer(0.01, self.update_boat_status)
        self.create_timer(0.5, self.update_waypoints_from_telemetry)
        self.create_timer(0.5, self.update_autopilot_parameters_from_telemetry)

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.cv_bridge  = CvBridge()
        
        self.autopilot_parameters_publisher = self.create_publisher(msg_type=String, topic='/autopilot_parameters', qos_profile=10)
        self.sensors_parameters_publisher = self.create_publisher(msg_type=String, topic='/sensors_parameters', qos_profile=10)
        self.waypoints_list_publisher = self.create_publisher(WaypointList, '/waypoints_list', qos_profile=10)
        
        self.desired_heading_listener = self.create_subscription(Float32, '/desired_heading', self.desired_heading_callback, 10)
        self.current_waypoint_index_listener = self.create_subscription(Int32, '/current_waypoint_index', self.current_waypoint_index_callback, 10)
        
        self.full_autonomy_maneuver_listener = self.create_subscription(msg_type=String, topic="/full_autonomy_maneuver", callback=self.full_autonomy_maneuver_callback, qos_profile=sensor_qos_profile)
        self.autopilot_mode_listener = self.create_subscription(msg_type=String, topic="/autopilot_mode", callback=self.autopilot_mode_callback, qos_profile=sensor_qos_profile)
        
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.velocity_listener = self.create_subscription(msg_type=Twist, topic="/velocity", callback=self.velocity_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)
        self.apparent_wind_vector_listener = self.create_subscription(msg_type=Vector3, topic="/apparent_wind_vector", callback=self.apparent_wind_vector_callback, qos_profile=sensor_qos_profile)
        self.camera_rgb_image_listener = self.create_subscription(msg_type=Image, topic="/camera/camera/color/image_raw", callback=self.camera_rgb_image_callback, qos_profile=sensor_qos_profile)
        self.vesc_telemetry_data_listener = self.create_subscription(VESCTelemetryData, '/vesc_telemetry_data', self.vesc_telemetry_data_callback, sensor_qos_profile)

        
        self.desired_sail_angle_listener = self.create_subscription(msg_type=Float32, topic="/desired_sail_angle", callback=self.desired_sail_angle_callback, qos_profile=sensor_qos_profile)
        self.desired_rudder_angle_listener = self.create_subscription(msg_type=Float32, topic="/desired_rudder_angle", callback=self.desired_rudder_angle_callback, qos_profile=sensor_qos_profile)

        # Default values in case these are never sent through ROS
        # If these values aren't changing then the ros node thats supposed to be sending these values may not be working correctly
        self.current_waypoints_list: list[tuple[float, float]] = [(0., 0.),]
        self.current_waypoint_index = 0
        self.position = NavSatFix(latitude=0., longitude=0.)
        
        self.autopilot_mode = ""
        self.full_autonomy_maneuver = ""

        self.velocity_vector = np.array([0., 0.])
        self.speed = 0.
        self.heading = 0.
        self.desired_heading = 0.

        self.apparent_wind_vector = np.array([0., 0.])
        self.apparent_wind_speed = 0.
        self.apparent_wind_angle = 0.
        
        self.base64_encoded_current_rgb_image = None

        self.desired_sail_angle = 0.
        self.desired_rudder_angle = 0.
                
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

    
    
    def position_callback(self, position: NavSatFix):
        self.position = position
        
    def velocity_callback(self, velocity_vector: Twist):
        self.velocity_vector = np.array([velocity_vector.linear.x, velocity_vector.linear.y])
        self.speed = np.sqrt(velocity_vector.linear.x**2 + velocity_vector.linear.y**2)

    def heading_callback(self, heading: Float32):
        self.heading = heading.data

    def apparent_wind_vector_callback(self, apparent_wind_vector: Vector3):
        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y])

        self.apparent_wind_speed, self.apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector.x, apparent_wind_vector.y)
        
    def desired_heading_callback(self, desired_heading: Float32):
        self.desired_heading = desired_heading.data
    
    
    
    def vesc_telemetry_data_callback(self, vesc_telemetry_data: VESCTelemetryData):
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

    def current_waypoint_index_callback(self, current_waypoint_index: Int32):
        self.current_waypoint_index = current_waypoint_index.data
    
    def full_autonomy_maneuver_callback(self, full_autonomy_maneuver: String):
        self.full_autonomy_maneuver = full_autonomy_maneuver.data
        
    def autopilot_mode_callback(self, autopilot_mode: String):
        self.autopilot_mode = autopilot_mode.data
        
    

    def camera_rgb_image_callback(self, camera_rgb_image: Image):
        """refer to this stack overflow post: https://stackoverflow.com/questions/40928205/python-opencv-image-to-byte-string-for-json-transfer"""
        rgb_image_cv = self.cv_bridge.imgmsg_to_cv2(camera_rgb_image, "rgb8")
        rgb_image_cv = rgb_image_cv[80:1200,40:680] # crop the image to 640,640
        retval, buffer = cv2.imencode(".jpg", rgb_image_cv)
        
        # swap red and blue channels for correction
        red = rgb_image_cv[:,:,2].copy()
        blue = rgb_image_cv[:,:,0].copy()
        rgb_image_cv[:,:,0] = red
        rgb_image_cv[:,:,2] = blue
        
        cv2.imwrite('test.jpg', rgb_image_cv)

        self.base64_encoded_current_rgb_image = base64.b64encode(buffer).decode()


    
    def desired_sail_angle_callback(self, desired_sail_angle: Float32):
        self.desired_sail_angle = desired_sail_angle.data

    def desired_rudder_angle_callback(self, desired_rudder_angle: Float32):
        self.desired_rudder_angle = desired_rudder_angle.data

    
    
        
    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()
    
    
    def get_raw_response(self, route):
        try:
            return requests.get(TELEMETRY_SERVER_URL + route, timeout=10).json()
        except Exception as e:
            print(e)
            time.sleep(0.5)
            print("retrying connection")
            return self.get_raw_response(route)
    
    
        
    def update_boat_status(self):
        # self.get_logger().info("update boat status")
        true_wind_vector = self.apparent_wind_vector + self.velocity_vector
        self.true_wind_speed, self.true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])

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
        
        
        
        # NOTE: Whenever you change this dictionary, make sure that you change how the groundstation interprets the boat status
        # To compress this dictionary, we will just only take the list of the values of the dictionary (since the keys are very large), and simply properly interpret what the list means on the groundstation
        # The groundstation must know the order and meaning of all of the parameters ahead of time which is the cost of compressing all of this information so much
        boat_status_dictionary = {
            "position": [self.position.latitude, self.position.longitude], 
            "state": self.autopilot_mode,
            "full_autonomy_maneuver": self.full_autonomy_maneuver,
            "speed": self.speed,
            "velocity_vector": (self.velocity_vector[0], self.velocity_vector[1]),
            "bearing": self.desired_heading, "heading": self.heading,
            "true_wind_speed": self.true_wind_speed, "true_wind_angle": self.true_wind_angle,
            "apparent_wind_speed": self.apparent_wind_speed, "apparent_wind_angle": self.apparent_wind_angle,
            "sail_angle": self.desired_sail_angle, "rudder_angle": self.desired_rudder_angle,
            "current_waypoint_index": self.current_waypoint_index,
            "distance_to_next_waypoint": get_distance_to_waypoint([self.position.latitude, self.position.longitude], self.current_waypoints_list[self.current_waypoint_index])
        }
        
        # self.get_logger().info(f"{pympler.asizeof.asizeof(list(boat_status_dictionary.values()))}")
        
        # requests.post(url=TELEMETRY_SERVER_URL + "/boat_status/set", json={"value": list(boat_status_dictionary.values())})
        requests.post(url=TELEMETRY_SERVER_URL + "/boat_status/set", json={"value": boat_status_dictionary})


    def update_waypoints_from_telemetry(self):
        new_waypoints_list = self.get_raw_response("/waypoints/get_new")
        
        if new_waypoints_list == {}: return
        
        
        # parse waypoints        
        waypoints_nav_sat_fix_list = []
        for waypoint in new_waypoints_list:
            if not waypoint: continue
            
            lat, lon = waypoint
            
            try: float(lat) and float(lon)
            except: raise Exception("Waypoints from Server Were Improperly Formatted")
            
            waypoints_nav_sat_fix_list.append(NavSatFix(latitude=float(lat), longitude=float(lon)))
        
        self.current_waypoints_list = new_waypoints_list
        self.waypoints_list_publisher.publish(WaypointList(waypoints=waypoints_nav_sat_fix_list))
        
        
    def update_autopilot_parameters_from_telemetry(self):
        new_autopilot_parameters_dictionary = self.get_raw_response("/autopilot_parameters/get_new")
        
        if new_autopilot_parameters_dictionary == {}: return
        
        for new_autopilot_parameter_name, new_autopilot_parameters_value in new_autopilot_parameters_dictionary.items():
            self.autopilot_parameters_dictionary[new_autopilot_parameter_name] = new_autopilot_parameters_value
    
        # if this is a new set of parameters, then update the ros2 topic so that the autopilot actually knows what the new parameters are 
        serialized_autopilot_parameters_string = String(data=json.dumps(self.autopilot_parameters_dictionary)) 
        self.autopilot_parameters_publisher.publish(serialized_autopilot_parameters_string)




def main():

    rclpy.init()
    telemetry_node = TelemetryNode()
    rclpy.spin(telemetry_node)
    
    telemetry_node.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__": main()