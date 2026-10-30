import os
import re
from collections.abc import Callable

import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment

from .bbox import BBox

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"

CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"

DUMMY_COST = 1e12 # dummy cost so it doesn't error for linear_sum_assignment when a row or column is all infinite
EARTH_RADIUS = 6_378_137.0

class StereoMatcher:
    def __init__(self, info_callback: Callable[[str], None]) -> None:
        self.info_callback = info_callback
        self.weights = [1, 5, 1] # Weight for x_diff, y_diff, and h_diff. Larger is more punishing
        self.buffer_x = 5 # How many pixels the right object can be right of the left object.
                          # When an object is very far away, the object will be approximately
                          # the same position in each frame. With the inconsistency from the inference,
                          # this could make the right object right of the left object.
        self.buffer_y = 20 # If the cameras are rectified, objects in the left and right should
                           # always fall along the same horizontal line. A buffer is good to
                           # account for inconsistency from inference.
        self.cam_list = self._read_camera_config()

    def process_stereo(self, objs_left: list[BBox], objs_right: list[BBox],
                       curr_pos: dict[float, float, float]) -> list[dict[int, int, float, float, float, float]]:
        """Calculate positions of objects using stereo depth."""
        self.info_callback("Received new objects")
        cost_matrix, row_ind, col_ind = self._match_points(objs_left, objs_right)

        results = []
        for i in range(len(row_ind)):
            row = row_ind[i]
            col = col_ind[i]
            if (cost_matrix[row][col] < DUMMY_COST):
                left_obj = objs_left[row]
                right_obj = objs_right[col]
                pos = self._calc_pos(left_obj, right_obj, curr_pos)
                results.append({
                    {
                        "left": left_obj.object_id,
                        "right": right_obj.object_id,
                        "lat": pos["lat"],
                        "lon": pos["lon"],
                        "depth": pos["depth"],
                        "dist": pos["dist"]
                    }
                })
                self.info_callback(f"Paired object {left_obj.object_id} to {right_obj.object_id}."
                                   f"Distance: {pos['dist']:.2f} meters")
        return results

    def _get_cost(self, left: BBox, right: BBox) -> float:
        if (left.class_id != right.class_id):
            return DUMMY_COST
        
        x_diff = left.x_position - right.x_position

        if (x_diff < 0 - self.buffer_x):
            return DUMMY_COST
        
        x_diff = abs(x_diff)
        
        y_diff = abs(left.y_position - right.y_position)
        if (y_diff > self.buffer_y):
            return DUMMY_COST
        
        w_x, w_y, w_h = self.weights
        h_diff = abs(left.height - right.height)
        
        return (w_x * x_diff) + (w_y * y_diff) + (w_h * h_diff)

    def _match_points(self, left: list[BBox], right: list[BBox]) -> tuple[list[int], list[int]]:
        cost_matrix = np.empty((len(left), len(right)))
        # Cost matrix
        # Left points make rows
        # Right points make columns
        for i in range(len(left)):
            for j in range(len(right)):
                cost_matrix[i][j] = self._get_cost(left[i], right[j])
        # self.info_callback(f"cost: {cost_matrix}")
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return (cost_matrix, row_ind, col_ind)
    
    def _calc_pos(self, left_obj: BBox, right_obj: BBox, curr_pos: dict[float, float, float]) -> dict[float, float, float, float]:
        depth = (self.cam_list[0]['focal_px'] * self.cam_list[0]['baseline']) / (left_obj.x_position - right_obj.x_position)
        forward = depth + self.cam_list[0]["gps_dist"]
        right = (left_obj.c_x - self.cam_list[0]["c_x"]) * depth / self.cam_list[0]["focal_px"] - self.cam_list[0]['baseline'] / 2
        h_rad = curr_pos['head'] * np.pi / 180
        d_east = forward * np.cos(h_rad) + right * np.sin(h_rad)
        d_north = forward * np.sin(h_rad) - right * np.cos(h_rad)
        lat = curr_pos["lat"] + d_north / EARTH_RADIUS * 180 / np.pi
        lon = curr_pos["lon"] + d_east / EARTH_RADIUS / np.cos(curr_pos["lat"]) * 180 / np.pi
        return {
            "depth": depth,
            "dist": np.sqrt(depth**2 + right**2),
            "lat": lat,
            "lon": lon
        }
        
    def _read_camera_config(self) -> dict:
        with open(CAMERA_CONFIG, 'r') as file:
            return yaml.safe_load(file)
