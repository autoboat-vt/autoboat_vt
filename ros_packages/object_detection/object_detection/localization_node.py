import json
import os
import re
from threading import Lock

import numpy as np
import yaml
from jsonc_parser.parser import JsoncParser
from pymap3d import geodetic2enu

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String

from autoboat_msgs.msg import ObjectDetectionResultsList, TriangulationResult, TriangulationResultsList

from .cv_library.bbox import BBox
from .cv_library.stereo_depth import StereoMatcher
from .cv_library.triangulation import ObjectDetection, ObjectTriangulator

EARTH_RADIUS = 6378137.0 # Radius of Earth in meters

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"

PATH_TO_PARAMETERS_FILE = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/cv_default_parameters.jsonc"
CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"

class LocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('localization_node')
        self.parameters = {
            "buffer_window_size": None, # The number of frames to keep in the buffer for triangulation.
                                        # Should be large enough to have multiple observations of the same object,
                                        # but small enough to not cause too much delay in publishing results.
            "iou_threshold": None, # If two detections are less than this distance apart, they are considered the same
                                   # object and the older one is deleted.
                                   # This is to prevent duplicate detections in triangulation.
            "update_rate": None, # How often to publish triangulation results in seconds. We only publish the most recent
                                 # detection for each object, so we don't need to publish every frame.
        }
        self._read_default_parameters()
        self.cam_list = self._read_camera_config()

        self.triangulation_results_publisher = self.create_publisher(
            msg_type=TriangulationResultsList, topic="/triangulation_results_list", qos_profile=10
        )
        self.emergency_stop_publisher = self.create_publisher(
            msg_type=Bool, topic="/object_detection_emergency_stop", qos_profile=10
        )

        self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self._position_callback,
                                 qos_profile=qos_profile_sensor_data)
        self.create_subscription(msg_type=Float32, topic="/heading", callback=self._heading_callback,
                                 qos_profile=qos_profile_sensor_data)
        self.create_subscription(msg_type=ObjectDetectionResultsList, topic="/object_detection_results_list",
                                 callback=self._object_detection_callback, qos_profile=10)
        self.create_subscription(msg_type=String, topic="/localization_parameters",
                                 callback=self._localization_parameters_callback, qos_profile=10)

        self.position_lock = Lock()
        self.current_position = { # position of the left camera, not the gps sensor
            "x": 0,
            "y": 0,
            "z": 0,
            "head": 0 # default to true east. Heading is radians counterclockwise of true east
        }
        self.origin_position = {
            "latitude": 0,
            "longitude": 0,
            "altitude": 0
        }
        self.valid_origin_position = True
        self.valid_heading = True

        self.camera_K = np.array([[self.cam_list[0]["focal_px"], 0, self.cam_list[0]["width"] * 0.5],
                                  [0, self.cam_list[0]["focal_px"], self.cam_list[0]["height"] * 0.5],
                                  [0, 0, 1]])
        self.camera_K_inv = np.linalg.inv(self.camera_K)
        self.triangulator = ObjectTriangulator(camera_matrix=self.camera_K,
                                               frame_size=(self.cam_list[0]["width"], self.cam_list[0]["height"]),
                                               buffer_window_size=self.parameters["buffer_window_size"],
                                               iou_threshold=self.parameters["iou_threshold"],
                                               logger=self.get_logger().info)
        self.stereo_matcher = StereoMatcher(self.get_logger().info, self.cam_list)

        # self.timer = self.create_timer(timer_period_sec=self.parameters["update_rate"], callback=self.triangulate)
    
    def _read_default_parameters(self) -> None:
        try:
            parameters = JsoncParser.parse_file(PATH_TO_PARAMETERS_FILE)
            for key in parameters:
                if key in self.parameters:
                    self.parameters[key] = parameters[key]["default"]
        except Exception as e:
            self.get_logger().error(f"Error reading parameters file: {e}")
    
    def _read_camera_config(self) -> dict:
        with open(CAMERA_CONFIG, 'r') as file:
            return yaml.safe_load(file)

    def _heading_callback(self, msg: Float32) -> None:
        # Heading input is degrees counterclockwise from east. Convert to radians
        with self.position_lock:
            self.current_position["head"] = np.radians(msg.data)
            self.valid_heading = True

    def _position_callback(self, msg: NavSatFix) -> None:
        with self.position_lock:
            if not self.valid_origin_position:
                self.origin_position["latitude"] = msg.latitude
                self.origin_position["longitude"] = msg.longitude
                self.origin_position["altitude"] = 0
                self.valid_origin_position = True
            if self.valid_heading:
                enu = geodetic2enu(msg.latitude, msg.longitude, self.cam_list[0]["altitude"], self.origin_position["latitude"],
                                   self.origin_position["longitude"], self.origin_position["altitude"])
                forward = self.cam_list[0]["gps_dist"]
                right = 0 - self.cam_list[0]["baseline"] / 2 # using the left camera.

                d_x = np.cos(self.current_position["head"]) * forward + np.sin(self.current_position["head"]) * right
                d_y = np.sin(self.current_position["head"]) * forward - np.cos(self.current_position["head"]) * right
                self.current_position["x"] = enu[0] + d_y
                self.current_position["y"] = enu[1] + d_x
                self.current_position["z"] = self.cam_list[0]["altitude"] # We could set it to enu[2]

    def _localization_parameters_callback(self, msg: String) -> None:
        new_parameters_json = json.loads(msg.data)
        # TODO: change to if (key in new_parameters)...
        for key in new_parameters_json:
            match (key):
                case "buffer_window_size":
                    self._update_buffer_window(new_parameters_json[key])
                case "iou_threshold":
                    self._update_iou_threshold(new_parameters_json[key])
                case "update_rate":
                    self._update_publish_frequency(new_parameters_json[key])
                case _:
                    self.get_logger().warn(f"Parameter {key} not recognized, ignoring")
    
    def _object_detection_callback(self, msg: ObjectDetectionResultsList) -> None:
        if self.valid_origin_position and self.valid_heading:
            
            # TODO This is not a good world frame ideally this should be in meters or whatever
            # TODO This also needs to account for the offset of the camera from the GPS antenna
            # pose_matrix = self._get_current_pose(self.current_position["latitude"], self.current_position["longitude"],
            #                                      self.current_position["head"])
            with self.position_lock:
                # Copy position data so it doesn't change mid-calculation
                curr_pos = {
                    "x": self.current_position["x"],
                    "y": self.current_position["y"],
                    "z": self.current_position["z"],
                    "head": self.current_position["head"]
                }
            frame_results = msg.detection_results
            # for obj in obj_list:
            #     self.triangulator.add_observation(ObjectDetection(
            #                                         frame_number = obj.frame_num,
            #                                         detector_confidence = obj.confidence,
            #                                         tracker_confidence = obj.tracker_confidence,
            #                                         x_position = obj.x_position,
            #                                         y_position = obj.y_position,
            #                                         width = obj.width,
            #                                         height = obj.height,
            #                                         object_id = obj.object_id,
            #                                         class_id = obj.class_id,
            #                                         obj_label = obj.obj_label,
            #                                         pose_matrix = pose_matrix,
            #                                         camera_matrix_inv = self.camera_K_inv
            #                                     ))
            stereo_objs: list[list[BBox]] = []
            # Use the right frame for stereo only. Use the left as the primary and for triangulation
            for i in range(len(frame_results)):
                stereo_objs.append([])
                obj_list = frame_results[i].detection_results
                for obj in obj_list:
                    new_detection = BBox()
                    new_detection.c_x = obj.x_position
                    new_detection.c_y = obj.y_position
                    new_detection.width = obj.width
                    new_detection.height = obj.height
                    new_detection.object_id = obj.object_id
                    new_detection.class_id = obj.class_id
                    stereo_objs[i].append(new_detection)
            stereo_results = self.stereo_matcher.process_stereo(stereo_objs[0], stereo_objs[1], curr_pos)
            frustum = self._calc_frustum(curr_pos)

    def _calc_frustum(self, curr_pos: dict[float, float, float, float]) -> dict:
        # For now assume camera is level
        forward = np.array([ # normalized forward vector of the camera
            np.cos(curr_pos["head"]),
            np.sin(curr_pos["head"]),
            0])
        right = np.array([ # normalized right vector of the camera
            np.cos(curr_pos["head"] - np.pi / 2),
            np.sin(curr_pos["head"] - np.pi / 2),
            0])
        up = np.array([0, 0, 1]) # normalized up vector of the camera
        position = np.array([curr_pos["x"], curr_pos["y"], curr_pos["z"]])
        near_clip = 0.5 # distance to close end of frustum
        far_clip = 2000 # distance to far end of frustum. would need real testing
        vfov = 2 * np.arctan(self.cam_list[0]["height"] / 2 / self.cam_list[0]["focal_px"])
        height_near = 2 * near_clip * np.tan(vfov / 2)
        width_near = height_near * self.cam_list[0]["width"] / self.cam_list[0]["height"]
        height_far = 2 * far_clip * np.tan(vfov / 2)
        width_far = height_far * self.cam_list[0]["width"] / self.cam_list[0]["height"]

        # Center points of the near and far faces
        center_near = position + forward * near_clip
        center_far = position + forward * far_clip

        # top/bottom, left/right, near/far
        tln = center_near + up * height_near / 2 - right * width_near / 2
        trn = center_near + up * height_near / 2 + right * width_near / 2
        bln = center_near - up * height_near / 2 - right * width_near / 2
        brn = center_near - up * height_near / 2 + right * width_near / 2
        tlf = center_far + up * height_far / 2 - right * width_far / 2
        trf = center_far + up * height_far / 2 + right * width_far / 2
        blf = center_far - up * height_far / 2 - right * width_far / 2
        brf = center_far - up * height_far / 2 + right * width_far / 2

        # Make the six faces
        # near, left, bottom, right, top, far
        # Should always point inside because of the calculation order
        left = np.cross(bln - tln, tlf - tln)
        left /= np.linalg.norm(left)
        far = np.cross(blf - tlf, trf - tlf)
        far /= np.linalg.norm(far)
        right = np.cross(brf - trf, trn - trf)
        right /= np.linalg.norm(right)
        near = np.cross(trn - tln, bln - tln)
        near /= np.linalg.norm(near)
        bottom = np.cross(brn - bln, blf - bln)
        bottom /= np.linalg.norm(bottom)
        top = np.cross(tlf - tln, trn - tln)
        top /= np.linalg.norm(top)

        # To calculate if a point is inside the frustum,
        # We need the vector and a point on the frustum.
        # We can just return two points on opposite corners.
        # like bln and trf
        return {
            "left": left,
            "right": right,
            "near": near,
            "bottom": bottom,
            "top": top,
            "far": far,
            "bln": bln,
            "trf": trf
        }
    
    def _get_current_pose(self, lat: float, long: float, heading: float) -> np.ndarray:
        # Delta lat/lon in radians
        d_lat = np.radians(lat - self.origin_position["latitude"])
        d_lon = np.radians(long - self.origin_position["longitude"])
        
        # Calculate Easting (x) and Northing (y)
        t_y = d_lat * EARTH_RADIUS
        t_x = d_lon * EARTH_RADIUS * np.cos(np.radians(self.origin_position["latitude"]))
        t_z = 0  # Assume sea level
        # if (self.origin_position["latitude"] != 0 and self.origin_position["longitude"] != 0):
        #     self.info_callback(f"Pose translation: {t_x} {t_y}")
        
        # 2. Calculate Rotation (R) from Heading
        psi = np.radians(heading)
        r_yaw = np.array([
            [ np.cos(psi), -np.sin(psi), 0],
            [ np.sin(psi),  np.cos(psi), 0],
            [ 0,            0,           1]
        ])
        
        # Columns are world-space representations of Camera X, Y, and Z
        # R = np.array([
        #     [ cos_p,  0,  sin_p],
        #     [-sin_p,  0,  cos_p],
        #     [     0, -1,      0]
        # ])
        r_cam_to_enu = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])

        r = r_yaw @ r_cam_to_enu
        
        # 3. Assemble 4x4 Matrix
        pose = np.eye(4)
        pose[:3, :3] = r
        pose[:3, 3] = [t_x, t_y, t_z]

        return pose

    def _update_publish_frequency(self, new_update_frequency: float) -> None:
        if new_update_frequency > 0:
            self.update_frequency = new_update_frequency
            self.timer.cancel()
            self.timer = self.create_timer(timer_period_sec=self.update_frequency, callback=self.vision_engine.triangulate)
            self.get_logger().info(f"Updated update frequency to {new_update_frequency}")
        else:
            self.get_logger().info(f"Bad update frequency {new_update_frequency}, must be > 0, not updating")
    
    def _update_buffer_window(self, new_buffer_window: int) -> None:
        self.triangulator.buffer_window = new_buffer_window
        self.parameters["buffer_window_size"] = new_buffer_window
        self.info_callback(f"Updated buffer window to {new_buffer_window}")

    def _update_iou_threshold(self, new_iou_threshold: float) -> None:
        self.triangulator.iou_threshold = new_iou_threshold
        self.parameters["iou_threshold"] = new_iou_threshold
        self.info_callback(f"Updated IOU threshold to {new_iou_threshold}")

    def triangulate(self) -> None:
        """Runs the triangulation algorithm on the current buffer of observations and publishes the results."""
        if not self.valid_origin_position:
            self.get_logger().info("No valid position yet. Skipping")
            return
        results = self.triangulator.triangulate(self.origin_position)
        self._publish_triangulation_results(results)
    
    def _publish_triangulation_results(self, triangulation_results: dict) -> None:
        msg = TriangulationResultsList()
        msg.iou_threshold = triangulation_results["iou_threshold"]
        msg.triangulation_results = []
        for obj_id in triangulation_results["triangulation_results"]:
            triangulation_result_msg = TriangulationResult()
            triangulation_result_msg.object_id = obj_id
            triangulation_result_msg.class_id = triangulation_results["triangulation_results"][obj_id]["class_id"]
            triangulation_result_msg.label = triangulation_results["triangulation_results"][obj_id]["label"]
            triangulation_result_msg.latitude = triangulation_results["triangulation_results"][obj_id]["lat"]
            triangulation_result_msg.longitude = triangulation_results["triangulation_results"][obj_id]["lon"]
            msg.triangulation_results.append(triangulation_result_msg)
        self.triangulation_results_publisher.publish(msg)
    
    def _publish_emergency_stop(self) -> None:
        msg = Bool()
        msg.data = True
        self.emergency_stop_publisher.publish(msg)

def main() -> None:
    rclpy.init()
    localization_node = LocalizationNode()
    try:
        rclpy.spin(localization_node)
    finally:
        localization_node.destroy_node()
        rclpy.shutdown()
