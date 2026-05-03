import json
import os
import re
import threading

import rclpy
from deepstream_engine import DeepStreamEngine
from jsonc_parser.parser import JsoncParser
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

# from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import Float32, String

from autoboat_msgs.msg import ObjectDetectionResult, ObjectDetectionResultsList, TriangulationResult, TriangulationResultsList

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_SRC_DIR = "/home/ws/src" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/src"

PATH_TO_PARAMETERS_FILE = f"{PATH_TO_SRC_DIR}/object_detection/object_detection/cv_default_parameters.jsonc"

class BuoyDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('buoy_detection_node')
        self.parameters = {
            "buffer_window_size": None, # The number of frames to keep in the buffer for triangulation.
                                        # Should be large enough to have multiple observations of the same object,
                                        # but small enough to not cause too much delay in publishing results.
            "iou_threshold": None, # If two detections are less than this distance apart, they are considered the same
                                   # object and the older one is deleted.
                                   # This is to prevent duplicate detections in triangulation.
            "update_rate": None, # How often to publish detection results in seconds. We only publish the most recent
                                 # detection for each object, so we don't need to publish every frame.
            "model_name": None, # model name without .pt.onnx. Ex. yolo11m.pt.onnx -> yolo11m
            "threshold": None # detection threshold
        }
        self._read_default_parameters()

        # ROS2 Initialization
        self.object_detection_results_publisher = self.create_publisher(
            msg_type=ObjectDetectionResultsList, topic="/object_detection_results_list", qos_profile=10
        )
        self.triangulation_results_publisher = self.create_publisher(
            msg_type=TriangulationResultsList, topic="/triangulation_results_list", qos_profile=10
        )
        self.position_listener = self.create_subscription(
            msg_type=NavSatFix, topic="/position", callback=self._position_callback, qos_profile=qos_profile_sensor_data
        )
        self.heading_listener = self.create_subscription( # heading is counterclockwise of true east
            msg_type=Float32, topic="/heading", callback=self._heading_callback, qos_profile=qos_profile_sensor_data
        )
        self.cv_parameters_listener = self.create_subscription(
            msg_type=String, topic="/cv_parameters", callback=self._cv_parameters_callback, qos_profile=10
        )

        self.vision_engine = DeepStreamEngine(
            triangulation_callback=self.publish_triangulation_results,
            info_callback=self._info_callback,
            warn_callback=self._warn_callback,
            error_callback=self._error_callback
        )

        vs = threading.Thread(target=self.vision_engine.run, daemon=True)
        vs.start()

        self.timer = self.create_timer(timer_period_sec=self.parameters["update_rate"], callback=self.vision_engine.triangulate)

    def _info_callback(self, msg: str) -> None:
        self.get_logger().info(msg)
    
    def _warn_callback(self, msg: str) -> None:
        self.get_logger().warn(msg)
    
    def _error_callback(self, msg: str) -> None:
        self.get_logger().error(msg)

    def _read_default_parameters(self) -> None:
        try:
            parameters = JsoncParser.parse_file(PATH_TO_PARAMETERS_FILE)
            for key in parameters:
                if key in self.parameters:
                    self.parameters[key] = parameters[key]["default"]
                else:
                    self.get_logger().warn(f"Parameter {key} not found in self.parameters")
        except Exception as e:
            self.get_logger().error(f"Error reading parameters file: {e}")
    
    def _position_callback(self, msg: NavSatFix) -> None:
        self.vision_engine.update_position(msg.latitude, msg.longitude)
    
    def _heading_callback(self, msg: Float32) -> None:
        self.vision_engine.update_heading(msg.data)
    
    def _cv_parameters_callback(self, msg: String) -> None:
        new_parameters_json = json.loads(msg.data)
        model_to_update = None
        threshold_to_update = None
        for key in new_parameters_json:
            match (key):
                case "model_name":
                    model_to_update = new_parameters_json[key]
                case "threshold":
                    threshold_to_update = new_parameters_json[key]
                case "buffer_window_size":
                    self.vision_engine.update_buffer_window(new_parameters_json[key])
                case "iou_threshold":
                    self.vision_engine.update_iou_threshold(new_parameters_json[key])
                case "update_rate":
                    self._update_publish_frequency(new_parameters_json[key])
                case _:
                    self.get_logger().warn(f"Parameter {key} not recognized, ignoring")
        self.vision_engine.update_model_or_threshold(model_to_update, threshold_to_update)

    def _update_publish_frequency(self, new_update_frequency: float) -> None:
        if hasattr(self, 'timer'):
            if new_update_frequency > 0:
                self.update_frequency = new_update_frequency
                self.timer.cancel()
                self.timer = self.create_timer(timer_period_sec=self.update_frequency, callback=self._iterate_results)
                self.get_logger().info(f"Updated update frequency to {new_update_frequency}")
            else:
                self.get_logger().info(f"Bad update frequency {new_update_frequency}, must be > 0, not updating")
        else:
            self.get_logger().warn("Inference is disabled, not updating update frequency")
