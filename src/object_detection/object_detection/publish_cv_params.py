import rclpy
from rclpy.node import Node

# from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String

import json
import argparse
import sys

class PublishCVParam(Node):
    def __init__(self):
        super().__init__('publish_cv_param_node')
        self.param_publisher_ = self.create_publisher(msg_type=String, topic='/cv_parameters', qos_profile=10)

    def publish_param_callback(self, msg=None):
        if msg is not None:
            self.param_publisher_.publish(msg)

def main(args):
    rclpy.init()
    publish_param_node = PublishCVParam()
    dict_msg = args
    param_msg = String()
    param_msg.data = json.dumps(dict_msg)
    publish_param_node.publish_param_callback(param_msg)  # Publish a message immediately
    print(f"Published parameters: {param_msg.data}")

def parse_args():
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
    json = {}
    if args.model:
        json["model_name"] = args.model
    if args.threshold:
        json["threshold"] = float(args.threshold)
    if args.buffer:
        json["buffer_window_size"] = int(args.buffer)
    if args.update_rate:
        json["update_rate"] = float(args.update_rate)
    if args.iou:
        json["iou_threshold"] = float(args.iou)
    return json

if __name__ == '__main__':
    args = parse_args()
    sys.exit(main(args))