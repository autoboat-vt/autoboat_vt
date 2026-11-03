import numpy as np

# Location and positions should be in a east (x), north (y), up (maybe?) in meters. Origin would be the first position we get

class FilteredResult:
    def __init__(self) -> None:
        self.current_result = None
        self.past_results = [] # array of tuples (frame_num, result)
        self.is_valid = False

class LocalizationFilter:
    def __init__(self) -> None:
        self.location_history = {} # {frame_num: { x, y, head }}
        pass
    
    def _remove_duplicates(self) -> None:
        # if two objects are within so many meters, merge the old into the new
        pass

    def _object_in_frustum(self, frustum: dict, obj: dict) -> bool:
        # Compare the object to all six vectors
        # Check the dot product, if it is < 0, it is outside the frustum
        v1 = obj - frustum['trf']
        v2 = obj - frustum['bln']
        return (np.dot(frustum['far'], v1) >= 0
            and np.dot(frustum['right'], v1) >= 0
            and np.dot(frustum['top'], v1) >= 0
            and np.dot(frustum['left'], v2) >= 0
            and np.dot(frustum['bottom'], v2) >= 0
            and np.dot(frustum['near'], v2) >= 0)
