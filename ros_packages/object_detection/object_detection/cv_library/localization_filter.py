# Location and positions should be in a east (x), north (y), up (maybe?) in meters. Origin would be the first position we get

class FilteredResult:
    def __init__(self):
        self.current_result = None
        self.past_results = {}
        self.is_valid = False

class LocalizationFilter:
    def __init__(self, cam_list):
        self.location_history = {} # {frame_num: { x, y, head }}
        self.cam_list = cam_list
        pass
    
    def _calc_frustum(self):
        # Calculate the 3d frustum of the camera.
        # Camera is at z-coordinate height above the water
        # Buoy would be at maybe 0.25 z coordinate
        pass
    
    def _remove_duplicates(self):
        # if two objects are within so many meters, merge the old into the new
