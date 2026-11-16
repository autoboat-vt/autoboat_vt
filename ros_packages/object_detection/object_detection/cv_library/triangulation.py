from collections.abc import Callable
from math import pi
from threading import Lock

import numpy as np

class ObjectTriangulator:
    def __init__(self, info_callback: Callable[[str], None], cam_list: dict) -> None:
        self.cam_list = cam_list
        self.info_callback = info_callback
        self.obj_tracks: dict[int, dict[str, np.array | list[float]]] = {} # {oid: {A, b, bearings}}
        self.max_tracks = 300
        self.min_baseline = np.radians(5) # The boat must have moved at least 5 degrees to be able to triangulate

    def triangulate(self, obj_list: dict[int, BBox], pos: dict[str, float]) -> dict[int, np.array]:
        """Add detection results to triangulation matrices and triangulate if possible."""
        # Currently does not account for z change as we get farther from the origin.
        results: dict[int, np.array] = {}
        for oid, bbox in obj_list.items():
            bearing = np.arctan((bbox.c_x - self.cam_list[0]["c_x"]) / self.cam_list[0]["focal_px"]) + pos["head"]
            A_new = np.array([-1 * np.sin(bearing)], np.cos(bearing))
            b_new = np.array([-1 * np.sin(bearing) * pos["x"] + np.cos(bearing) * pos["y"]])
            bearing %= np.pi
            if oid in self.obj_tracks:
                # np.vstack is kinda slow, could be optimized. It copies every row
                # May be good enough for what we need.
                self.obj_tracks[oid]["A"] = np.vstack((self.obj_tracks[oid]["A"], A_new))
                self.obj_tracks[oid]["b"] = np.vstack((self.obj_tracks[oid]["b"], b_new))
                self.obj_tracks[oid]["bearings"].append(bearing)
                num_tracks = len(self.obj_tracks[oid]["A"]) # length of A, b, and bearing should all be the same
                if (num_tracks >= self.max_tracks):
                    self.obj_tracks[oid]["A"] = self.obj_tracks[oid]["A"][num_tracks - self.max_tracks:]
                    self.obj_tracks[oid]["b"] = self.obj_tracks[oid]["b"][num_tracks - self.max_tracks:]
                    self.obj_tracks[oid]["bearings"] = self.obj_tracks[oid]["bearings"][num_tracks - self.max_tracks:]
            else:
                self.obj_tracks[oid]["A"] = np.array([A_new])
                self.obj_tracks[oid]["b"] = np.array([b_new])
                self.obj_tracks[oid]["bearings"] = [ bearing ]
            if self._can_triangulate(oid):
                result = np.linalg.lstsq(self.obj_tracks[oid]["A"], self.obj_tracks[oid]["b"])[0] # don't care about the residuals
                results[oid] = np.array([result[0], result[1], 0.2]) # convert to flat vector instead of vertical
        return results
    
    def _can_triangulate(self, oid: int) -> bool:
        # Find any two bearings that are at least 5 degrees apart
        for i in range(len(orientations)):
            for j in range(len(orientations)-1, i, -1):
                diff = abs(orientations[i] - orientations[j])
                ang_diff = min(diff, np.pi - diff)
                if (ang_diff >= self.min_baseline):
                    return True
        return False
