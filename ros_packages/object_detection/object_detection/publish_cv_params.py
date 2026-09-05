"""
This is a test file to publish CV parameters to the /cv_parameters topic as a JSON string.
This is for testing purposes only and should not be used in production.
The parameters published here will be used by the deepstream_buoy_detection_node to update the CV parameters in real-time.
"""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node

# from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String


class PublishCVParam(Node):
    def __init__(self) -> None:
        """Initialize the publish_cv_param_node."""
        super().__init__('publish_cv_param_node')
        self.cv_publisher = self.create_publisher(msg_type=String, topic='/cv_parameters', qos_profile=10)
        self.localization_publisher = self.create_publisher(msg_type=String, topic='/localization_parameters', qos_profile=10)

    def publish_cv_param(self, msg: String=None) -> None:
        """Publish CV parameters."""
        if msg is not None:
            self.cv_publisher.publish(msg)

    def publish_localization_param(self, msg: String=None) -> None:
        """Publish localization parameters."""
        if msg is not None:
            self.localization_publisher.publish(msg)

def main(args: tuple[dict, dict]) -> None:
    rclpy.init()
    publish_param_node = PublishCVParam()
    cv_params = args[0]
    localization_params = args[1]
    if cv_params != {}:
        param_msg = String()
        param_msg.data = json.dumps(cv_params)
        publish_param_node.publish_cv_param(param_msg)
        print(f"Published cv parameters: {param_msg.data}")
    if localization_params != {}:
        param_msg = String()
        param_msg.data = json.dumps(localization_params)
        publish_param_node.publish_localization_param(param_msg)
        print(f"Published localization parameters: {param_msg.data}")

def parse_args() -> tuple[dict, dict]:
    parser = argparse.ArgumentParser(description="CV parameter publisher")
    parser.add_argument(
        "-m",
        "--model",
        help="Model name to publish",
        required=False
    )
    parser.add_argument(
        "-t",
        "--threshold",
        help="Confidence threshold to publish",
        required=False
    )
    parser.add_argument(
        "-b",
        "--buffer",
        help="Buffer size to publish",
        required=False
    )
    parser.add_argument(
        "-u",
        "--update_rate",
        help="Update rate to publish",
        required=False
    )
    parser.add_argument(
        "-i",
        "--iou",
        help="IoU threshold to publish",
        required=False
    )
    if (len(sys.argv) == 1):
        parser.print_help(sys.stderr)
        sys.exit(1)
    args = parser.parse_args()
    cv_params = {}
    localization_params = {}
    if args.model:
        cv_params["model_name"] = args.model
    if args.threshold:
        cv_params["threshold"] = float(args.threshold)
    if args.buffer:
        localization_params["buffer_window_size"] = int(args.buffer)
    if args.update_rate:
        localization_params["update_rate"] = float(args.update_rate)
    if args.iou:
        localization_params["iou_threshold"] = float(args.iou)
    return cv_params, localization_params

if __name__ == '__main__':
    cv_params, localization_params = parse_args()
    sys.exit(main((cv_params, localization_params)))
