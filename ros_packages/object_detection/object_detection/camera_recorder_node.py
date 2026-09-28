import json
import os
import re
import shutil
import subprocess
import sys
import time

import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"
CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"

class CamCorderNode(Node):
    def __init__(self) -> None:
        super().__init__('cam_corder')
        self.storage_cap = 0.6 # Do not go above this disk utilization
        self.last_time = time.time()
        
        if self._get_storage_util() > self.storage_cap:
            raise OSError(f"Current disk usage is above {self.storage_cap * 100:.0f}%. Exitting")

        os.makedirs("./frame_logs/", exist_ok=True)
        count = 0
        while os.path.exists(f"./frame_logs/run{count}"):
            count += 1
        self.log_file = f"./frame_logs/run{count}/frame_logs.jsonl"
        self.run_dir = f"./frame_logs/run{count}/frames/"
        os.makedirs(self.run_dir, exist_ok=True)

        self.cam_list = self._read_camera_config()
        self.cam_list[0]["device"] = self._find_camera()
        
        
        """
        formatting for self.log_file
        Each line is a JSON object representing a frame
        <frame_num>: {
            head: <current_heading>,
            lat: <current_lat>,
            long: <current_lon>,
            time: <current_time>
        }
        """

        self.position = {
            "lon": 0,
            "lat": 0,
            "head": 0
        }
        
        
        self.position_listener = self.create_subscription(
            msg_type=NavSatFix, topic="/position", callback=self._position_callback, qos_profile=qos_profile_sensor_data
        )
        self.heading_listener = self.create_subscription( # heading is counterclockwise of true east
            msg_type=Float32, topic="/heading", callback=self._heading_callback, qos_profile=qos_profile_sensor_data
        )

        self._record()

    def _record(self) -> None:
        device = self.cam_list[0]["device"]
        width = self.cam_list[0]["width"]
        height = self.cam_list[0]["height"]
        fps = self.cam_list[0]["framerate_n"] / self.cam_list[0]["framerate_d"]
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
                curr_log = {
                    "lat": self.position["lat"],
                    "lon": self.position["lon"],
                    "head": self.position["head"],
                    "time": time.time()
                }
                if count % 120 == 0:
                    current_time = time.time()
                    fps = 120 / (current_time - self.last_time)
                    self.last_time = current_time
                    self.get_logger().info(f"Current frame count: {count}, FPS: {fps:.2f}")
                cv2.imwrite(f'{self.run_dir}frame{count:06d}.png', frame)
                with open(self.log_file, 'a') as file:
                    file.write(json.dumps(curr_log) + '\n')
                if self._get_storage_util() > self.storage_cap:
                    self.get_logger().info(f"Passed {(self.storage_cap * 100):.0f}% disk usage. Exiting")
                    break
                count += 1
        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            cv2.destroyAllWindows()

    def _read_camera_config(self) -> dict:
        with open(CAMERA_CONFIG, 'r') as file:
            return yaml.safe_load(file)

    def _find_camera(self, cam_id: int = 0) -> str:
        """
        This is just a way to figure out which /dev/video* is the camera<br>
        The camera outputs on 3 devices<br>
        Each device is a different format, but the order can change or extra cameras can cause the number to increase<br>
        While this finds the device with the specified format,
        it does not guarantee that the correct resolution and framerate are available.
        
        Returns
        -------
            str: The /dev/video* device path.
        """

        cam_format = self.cam_list[cam_id]["v4l2_format"]
        cam_name = self.cam_list[cam_id]["name"]
        ls = shutil.which("ls")
        cat = shutil.which("cat")
        v4l2_ctl = shutil.which("v4l2-ctl")
        if ls is None or cat is None or v4l2_ctl is None:
            self.error_callback("ls, cat, or v4l2-ctl command not found. Cannot find camera device.")
            raise OSError("Required command not found")
        try:
            camera_devices_output = subprocess.run([ls, '/sys/class/video4linux/'], # noqa: S603
                                                   capture_output=True, text=True, check=True).stdout
        except subprocess.CalledProcessError as err:
            self.error_callback("Failed to list camera devices in /sys/class/video4linux/.")
            raise OSError("Failed to list camera devices in /sys/class/video4linux/.") from err
        for device in camera_devices_output.splitlines():
            try:
                if ((re.search(cam_name, subprocess.run([cat, f'/sys/class/video4linux/{device}/name'], # noqa: S603
                                                        capture_output=True, text=True, check=True).stdout) is not None) and
                (re.search(cam_format, subprocess.run([v4l2_ctl, '--device', f'/dev/{device}', '--list-formats'], # noqa: S603
                                                            capture_output=True, text=True, check=True).stdout) is not None)):
                        return f"/dev/{device}"
            except subprocess.CalledProcessError as err:
                self.error_callback(f"Command v4l2-ctl failed for device {device}.")
                raise OSError(f"Command v4l2-ctl failed for device {device}.") from err
        self.error_callback(f"Could not find {cam_name} device with {cam_format} format")
        raise OSError("Camera device not found")
    
    def _position_callback(self, msg: NavSatFix) -> None:
        self.position["lat"] = msg.latitude
        self.position["lon"] = msg.longitude
    
    def _heading_callback(self, msg: Float32) -> None:
        self.position["head"] = msg.data

    def _get_storage_util(self) -> float:
        usage = shutil.disk_usage('/')
        total = usage.total
        used = usage.used
        return used / total

def main() -> None:
    rclpy.init()
    cam_corder_node = CamCorderNode()
    try:
        rclpy.spin(cam_corder_node)
    finally:
        cam_corder_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(main())
