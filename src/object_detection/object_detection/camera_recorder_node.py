import json
import os
import re
import shutil
import subprocess
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


class CamCorderNode(Node):
    def __init__(self) -> None:
        super().__init__('cam_corder')
        self.storage_cap = 0.6 # Do not go above this disk utilization
        self.save_interval = 5 # write to the log file every 5 frames
        
        if self.get_storage_util() > self.storage_cap:
            return
        self.CAM_LIST = {
            0: {
                "name": self._find_camera("YUYV"),
                "framerate": 15.,
                "format": "YUY2",
                "input_width": 1280,
                "input_height": 800
            }
        }
        
        os.makedirs("./frame_logs/frames", exist_ok=True)
        self.log_file = "./frame_logs/frame_logs.json"
        
        
        """
        formatting for self.frame_logs
        <frame_num>: {
            head: <current_heading>,
            lat: <current_lat>,
            long: <current_lon>,
            time: <current_time>
        }
        """
        self.frame_logs = {}


        self.position = {
            "lon": 0,
            "lat": 0,
            "head": 0
        }
        
        
        self.position_listener = self.create_subscription(
            msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=qos_profile_sensor_data
        )
        self.heading_listener = self.create_subscription( # heading is counterclockwise of true east
            msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=qos_profile_sensor_data
        )

        self.record()

    def record(self):
        device = self.CAM_LIST[0]["name"]
        width = self.CAM_LIST[0]["input_width"]
        height = self.CAM_LIST[0]["input_height"]
        fps = self.CAM_LIST[0]["framerate"]
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warn(f"Could not open video device {device}")
            raise OSError(f"Could not open video device {device}")

        # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*format))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)

        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f"Camera opened with resolution {actual_width}x{actual_height} at {actual_fps:.1f} FPS")

        try:
            count = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    break
                self.frame_logs[count] = {
                    "lat": self.position["lat"],
                    "lon": self.position["lon"],
                    "head": self.position["head"],
                    "time": time.time()
                }
                if count % 120 == 0:
                    self.get_logger().info(f"Current frame count: {count}")
                cv2.imwrite(f'./frame_logs/frames/frame{count}.png', frame)
                if count % self.save_interval == 1:
                    with open(self.log_file, 'w') as file:
                        file.write(json.dumps(self.frame_logs))
                    if self.get_storage_util() > self.storage_cap:
                        self.get_logger().info(f"Passed {(self.storage_cap * 100):.0f}% disk usage. Exitting")
                        break
                count += 1
        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            cv2.destroyAllWindows()

    def _find_camera(self, format: str):
        camera_devices_output = subprocess.run(['ls', '/sys/class/video4linux/'], capture_output=True, text=True).stdout
        for device in camera_devices_output.splitlines():
            if (re.search("RealSense", subprocess.run(['cat', f'/sys/class/video4linux/{device}/name'], capture_output=True, text=True).stdout) is not None):
                if (re.search(format, subprocess.run(['v4l2-ctl', '--device', f'/dev/{device}', '--list-formats'], capture_output=True, text=True).stdout) is not None):
                    return f"/dev/{device}"
        self.get_logger().warn(f"Could not find RealSense camera device with {format} format")
        raise OSError("Camera device not found")
    
    def position_callback(self, msg: NavSatFix):
        self.position["lat"] = msg.latitude
        self.position["lon"] = msg.longitude
    
    def heading_callback(self, msg: Float32):
        self.position["head"] = msg.data

    def get_storage_util(self):
        usage = shutil.disk_usage('/')
        total = usage.total
        used = usage.used
        return used / total

def main():
    rclpy.init()
    cam_corder_node = CamCorderNode()
    
    try:
        rclpy.spin(cam_corder_node)
    finally:
        cam_corder_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(main())
