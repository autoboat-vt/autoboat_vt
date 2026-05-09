from collections.abc import Callable
from math import pi
from threading import Lock

import numpy as np

EARTH_RADIUS = 6378137.0 # in meters

class ObjectDetection:
    def __init__(self, frame_number:int=-1, detector_confidence:float=0.0, tracker_confidence:float=0.0,
                 x_position:float=0.0, y_position:float=0.0, width:float=0.0, height:float=0.0, object_id:int=-1,
                 class_id:int=-1, obj_label:str="", pose_matrix:np.ndarray=None, camera_matrix_inv:np.ndarray=None) -> None:
        self.frame_number = frame_number
        self.detector_confidence = detector_confidence
        self.tracker_confidence = tracker_confidence
        self.x_position = x_position # center x
        self.y_position = y_position # center y
        self.width = width
        self.height = height
        self.object_id = object_id
        self.class_id = class_id
        self.obj_label = obj_label
        self.pose_matrix = pose_matrix

        pixel_point = np.array([[x_position], [y_position], [1]])
        bearing_cam = camera_matrix_inv @ pixel_point
        # bearing_cam = bearing_cam / np.linalg.norm(bearing_cam)

        r = pose_matrix[:3, :3]
        t = pose_matrix[:3, 3]

        ray_dir = r @ bearing_cam
        ray_dir = ray_dir / np.linalg.norm(ray_dir)
        self.triangle_position = (t.reshape(3, 1), ray_dir.reshape(3, 1))


class ObjectTrack:
    def __init__(self, object_id:int=-1, class_id:int=-1, obj_label:str="") -> None:
        self.detection_results = []
        self.last_updated_frame_number = -1
        self.object_id = object_id
        self.class_id = class_id
        self.obj_label = obj_label
        self.last_world_pos = None
        # self.eig_debug_log_count = 0
    
    def add_detection(self, detection: ObjectDetection, buffer_window: int) -> None:
        """Add a new detection to the track, maintaining a buffer of recent detections."""
        self.detection_results.append(detection)
        while len(self.detection_results) > buffer_window:
            self.detection_results.pop(0)


class ObjectTriangulator:
    def __init__(self, camera_matrix: np.ndarray, frame_size: tuple,
                 buffer_window_size: int, iou_threshold: float,
                 logger: Callable[[str], None]) -> None:
        self.K = camera_matrix
        self.K_inv = np.linalg.inv(camera_matrix)
        self.observations = {} # {obj_id: ObjectTrack}
        self.frame_size = frame_size
        self.buffer_window = buffer_window_size
        self.logger = logger
        self.iou_threshold = iou_threshold
        self.triangulation_lock = Lock()

    def add_observation(self, detection: ObjectDetection) -> None:
        """Add a new detection to the track, maintaining a buffer of recent detections."""
        with self.triangulation_lock:
            obj_id = detection.object_id
            if (detection.x_position < 0 or detection.y_position < 0 or detection.x_position >= self.frame_size[0] or
                detection.y_position >= self.frame_size[1] - 1):
                return
            if obj_id not in self.observations:
                self.observations[obj_id] = ObjectTrack(object_id=detection.object_id, class_id=detection.class_id,
                                                        obj_label=detection.obj_label)
            self.observations[obj_id].add_detection(detection, self.buffer_window)
    
    def _triangulate(self, obj_id: int) -> np.ndarray:
        """Triangulate the 3D position of the object with the given ID using its buffered detections."""
        track = self.observations[obj_id]
        obs = track.detection_results
        if len(obs) < 2:
            return track.last_world_pos # Need at least 2 observations to triangulate
        
        if (obs[-1].frame_number == track.last_updated_frame_number):
            return track.last_world_pos # If we already triangulated this frame, return the last result

        a = np.zeros((3, 3))
        b = np.zeros((3, 1))

        for detection in obs:
            p, d = detection.triangle_position
            identity_minus_ddt = np.eye(3) - (d @ d.T)
            a += identity_minus_ddt
            b += identity_minus_ddt @ p

        # Degenerate geometry (all rays nearly along one axis) makes A poorly conditioned.
        # In that case, the least-squares position is numerically unstable.
        eigvals = np.linalg.eigvalsh(a)
        lambda_min = float(np.min(eigvals))
        lambda_max = float(np.max(eigvals))
        condition_ratio = lambda_min / lambda_max if lambda_max > 0.0 else 0.0

        # Temporary tuning telemetry: keep sparse to avoid log spam.
        # current_frame = obs[-1].frame_number
        # should_log_tuning = (
        #     track.eig_debug_log_count < 20
        #     and (track.eig_debug_log_count < 5 or current_frame % 15 == 0)
        #     and self.logger
        # )
        # if should_log_tuning:
        #     self.logger(
        #         f"Object {obj_id} triangulation eigvals={eigvals.tolist()} "
        #         f"ratio={condition_ratio:.3e} n_obs={len(obs)}"
        #     )
        #     track.eig_debug_log_count += 1

        lambda_threshold = 1e-9
        condition_threshold = 1e-4
        if lambda_max < lambda_threshold or condition_ratio < condition_threshold:
            if self.logger:
                self.logger(
                    f"Object {obj_id} has ill-conditioned triangulation geometry "
                    f"(lambda_min={lambda_min:.3e}, lambda_max={lambda_max:.3e}), skipping triangulation"
                    f"(condition_ratio={condition_ratio:.3e} threshold={condition_threshold:.3e}, n_obs={len(obs)})"
                )
            return track.last_world_pos
        
        world_pos = np.linalg.lstsq(a, b, rcond=None)[0].flatten()
        track.last_world_pos = world_pos
        track.last_updated_frame_number = obs[-1].frame_number
        return world_pos
    
    def triangulate(self, origin_position: dict) -> dict:
        """Triangulate the 3D position of all objects with buffered detections."""
        with self.triangulation_lock:
            # Make a shallow copy to avoid locking the whole triangulator during triangulation
            observations_snapshot = dict(self.observations)

        detections = {}
        detections["iou_threshold"] = self.iou_threshold
        detections["triangulation_results"] = {}
        for obj_id, obs_track in observations_snapshot.items():
            # obs_list = obs_track.detection_results
            world_pos = self._triangulate(obj_id)
            if world_pos is not None:
                lat = origin_position["latitude"] + (world_pos[1] / EARTH_RADIUS) * (180 / pi)
                lon = (origin_position["longitude"] +
                      (world_pos[0] / (EARTH_RADIUS * np.cos(np.radians(origin_position["latitude"])))) * (180 / pi))
                detections["triangulation_results"][obj_id] = {
                    "label": obs_track.obj_label,
                    "class_id": obs_track.class_id,
                    "world_pos": world_pos,
                    "lat": lat,
                    "lon": lon,
                    "last_updated_frame_number": obs_track.last_updated_frame_number
                }
            else:
                self.logger(f"Object {obj_id} does not have enough observations to triangulate")
        
        self._filter_results(detections)
        return detections

    def _filter_results(self, detections: dict) -> None:
        """
        Define some nms/iou filtering.
        If 2 objects are very close together, we can assume they are the same object and only publish one of them.
        If overlapping detections have a recent detection and one with an old detection, we can delete the old detection
        and keep the new one.
        """
        ids_to_delete = []
        # Instead of an n^2 loop, filter on detections.keys()
        # This is a list, so we can modify the original detections dict in-place without affecting the loop
        # Outer loop: i = 0->n-1
        # Inner loop: j = i+1->n, so we only compare each pair once and don't compare an object with itself
        for obj_id_1, det1 in detections["triangulation_results"].items():
            for obj_id_2, det2 in detections["triangulation_results"].items():
                if obj_id_1 >= obj_id_2 or det1["class_id"] != det2["class_id"]:
                    # Don't compare the same pair twice or with different classes
                    continue

                dist = np.linalg.norm(det1["world_pos"] - det2["world_pos"])
                if dist < self.iou_threshold: # If detections are less than this many meters apart, keep the most recent one
                    if det1["last_updated_frame_number"] > det2["last_updated_frame_number"]:
                        ids_to_delete.append(obj_id_2)
                    else:
                        ids_to_delete.append(obj_id_1)

        for obj_id in ids_to_delete:
            del detections["triangulation_results"][obj_id]
