# ruff: noqa:I001 E402, F401
# The order of imports matters. Moving some of the imports like ROS2 causes a segfault.
# Some rearranging has been found to work, but for now, just keep it this way.
# Linter rules regarding import statements have been disabled.
# Despite not actually using Gst in this file, it needs to be imported here to prevent a segfault. Don't ask me why.
import gi

gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst

import json
import os
import re
import threading

import rclpy
from jsonc_parser.parser import JsoncParser
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

# from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import Float32, String

from autoboat_msgs.msg import ObjectDetectionResult, ObjectDetectionFrameResults, ObjectDetectionResultsList

from .cv_library.deepstream_engine import DeepStreamEngine

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"

PATH_TO_PARAMETERS_FILE = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/cv_default_parameters.jsonc"

class BuoyDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('buoy_detection_node')
        self.parameters = {
            "model_name": None, # model name without .onnx. Ex. yolo11m.onnx -> yolo11m
            "threshold": None # detection threshold
        }
        self._read_default_parameters()

        # ROS2 Initialization
        self.object_detection_results_publisher = self.create_publisher(
            msg_type=ObjectDetectionResultsList, topic="/object_detection_results_list", qos_profile=10
        )
        self.create_subscription(msg_type=String, topic="/cv_parameters", callback=self._cv_parameters_callback, qos_profile=10)
        self.vision_engine = DeepStreamEngine(
            detection_callback=self._publish_detection_results,
            info_callback=self._info_callback,
            warn_callback=self._warn_callback,
            error_callback=self._error_callback
        )

        vs = threading.Thread(target=self.vision_engine.run, daemon=True)
        vs.start()

    def close_pipeline(self) -> None:
        """Cleanly close the pipeline and shutdown the node."""
        self.vision_engine.close_pipeline()
        self.destroy_node()
        rclpy.shutdown()

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
        except Exception as e:
            self.get_logger().error(f"Error reading parameters file: {e}")
    
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
                case _:
                    self.get_logger().warn(f"Parameter {key} not recognized, ignoring")
        self.vision_engine.update_model_or_threshold(model_to_update, threshold_to_update)

    def _publish_detection_results(self, detection_results: dict) -> None:
        msg = ObjectDetectionResultsList()
        msg.ntp_timestamp = detection_results["ntp_timestamp"]
        msg.model_name = detection_results["model_name"]
        msg.yolo_version = detection_results["yolo_version"]
        msg.threshold = detection_results["threshold"]
        msg.detection_results = []
        for frame_detections in detection_results["detection_results"]:
            frame_results = ObjectDetectionFrameResults()
            frame_results.detection_results = []
            for detection in frame_detections:
                detection_msg = ObjectDetectionResult()
                detection_msg.detector_confidence = detection["detector_confidence"]
                detection_msg.tracker_confidence = detection["tracker_confidence"]
                detection_msg.x_position = detection["x_position"]
                detection_msg.y_position = detection["y_position"]
                detection_msg.width = detection["width"]
                detection_msg.height = detection["height"]
                detection_msg.object_id = detection["object_id"]
                detection_msg.class_id = detection["class_id"]
                detection_msg.obj_label = detection["obj_label"]
                detection_msg.angle_to_object = detection["angle_to_object"]
                frame_results.detection_results.append(detection_msg)
            msg.detection_results.append(frame_results)
        self.object_detection_results_publisher.publish(msg)

def main() -> None:
    rclpy.init()
    buoy_detection_node = BuoyDetectionNode()
    try:
        rclpy.spin(buoy_detection_node)
    finally:
        buoy_detection_node.close_pipeline()
