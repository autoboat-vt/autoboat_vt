import os
import re
from collections.abc import Callable

import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"

CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"

DUMMY_COST = 1e12 # dummy cost so it doesn't error for linear_sum_assignment when a row or column is all infinite

class Detection:
    def __init__(self) -> None:
        self.x_position = -1
        self.y_position = -1
        self.width = -1
        self.height = -1
        self.object_id = -1
        self.class_id = -1

class StereoMatcher():
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

    def process_stereo(self, objs_left: list[Detection], objs_right: list[Detection],
                       curr_pos: dict[float, float, float]) -> list[dict[float, float]]:
        """Calculate positions of objects using stereo depth."""
        self.info_callback("Received new objects")
        cost_matrix, row_ind, col_ind = self._match_points(objs_left, objs_right)
        
        for i in range(len(row_ind)):
            row = row_ind[i]
            col = col_ind[i]
            if (cost_matrix[row][col] < DUMMY_COST):
                left_obj = objs_left[row]
                right_obj = objs_right[col]
                pos = self._calc_pos(left_obj, right_obj, curr_pos)
                self.info_callback(f"Paired object {left_obj.object_id} to {right_obj.object_id}. Distance: {pos:.2f} meters")

    def _get_cost(self, left: Detection, right: Detection) -> float:
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

    def _match_points(self, left: list[Detection], right: list[Detection]) -> tuple[list[int], list[int]]:
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
    
    def _calc_pos(self, left_obj: Detection, right_obj: Detection, curr_pos: dict[float, float, float]) -> dict[float, float]:
        depth = (self.cam_list[0]['focal_px'] * self.cam_list[0]['baseline']) / (left_obj.x_position - right_obj.x_position)
        # angle_to_object = 0
        return depth
        
    def _read_camera_config(self) -> dict:
        with open(CAMERA_CONFIG, 'r') as file:
            return yaml.safe_load(file)
