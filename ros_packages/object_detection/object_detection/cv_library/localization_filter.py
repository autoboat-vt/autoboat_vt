import numpy as np

# Location and positions should be in a east (x), north (y), up (maybe?) in meters. Origin would be the first position we get

# Step 1: Calculate new position
# Step 1: remove duplicates. Merge tracks. Recalculate position
# Step 2: Monitor full-faith
# Step 3: calculate if it is in frustum
# Step 4: Adjust trust value

class FilteredResult:
    def __init__(self) -> None:
        self.current_result = None # Averaged result
        # I don't think we're even using past_results right now
        self.past_results: list[tuple[int, np.array, np.array]] = [] # array of tuples (frame_num, raw_result, filtered_result).
        self.is_valid = False
        self.oid = -1 # object id
        self.cid = -1 # class id
        self.trust = 1
        self.last_frame = -1 # Last time this object was updated

class LocalizationFilter:
    def __init__(self) -> None:
        self.trust_threshold = 15
        self.max_trust = 60
        self.trust_hit = 2 # Increase trust by this amount when we see it
        self.trust_miss = 1 # Lower trust by this amount when we should see it but don't
        self.duplicate_threshold = 10 # if two objects are this distance in meters, merge their detections
        self.mu_ewa = 0.5 # Exponential weighted average weight. Low mu means new results are valued more.
        self.results: dict[int, FilteredResult] = {}
        # Maybe add a lock, maybe with timeout?

    def filter_results(self, frustum: dict, frame_num: int, new_detections: list[tuple[int, int, np.array]]) -> list:
        """Filter results."""
        for oid, cid, pos in new_detections:
            if oid in self.results:
                self.results[oid].current_result = (self.results[oid].current_result * self.mu_ewa + (1 - self.mu_ewa) * pos)
                self.results[oid].past_results.append((frame_num, pos, self.results[oid].current_result))
                self.results[oid].last_frame = frame_num
            else:
                self.results[oid] = FilteredResult()
                self.results[oid].current_result = pos
                self.results[oid].past_results.append(frame_num, pos, pos)
                self.results[oid].oid = oid
                self.results[oid].cid = cid
                self.results[oid].last_frame = frame_num
        self._remove_duplicates()
        self.update_trust(frustum, frame_num)
        return self._get_results()
    
    def _remove_duplicates(self) -> None:
        # if two objects are within so many meters, merge the old into the new
        tracks_to_delete = []
        for oid1, obj1 in self.results.items():
            for oid2, obj2 in self.results.items():
                # Don't compare objects multiple times, only compare of the same class, and check distance
                if (oid1 < oid2 and obj1.cid == obj2.cid and self._dist(obj1, obj2) <= self.duplicate_threshold):
                    tracks_to_delete.append(self._merge_tracks(obj1, obj2))
        for oid in tracks_to_delete:
            del self.results[oid]

    def _merge_tracks(self, obj1: FilteredResult, obj2: FilteredResult) -> int:
        # Merge sort. Returns the id to delete
        merged_past_results: list[tuple[int, np.array, np.array]] = []
        obj1_ind = 0
        obj2_ind = 0
        while obj1_ind < len(obj1.past_results) or obj2_ind < len(obj2.past_results):
            # Probably need to check if they have the same frame number
            if ((obj2_ind >= len(obj2.past_results)) or (obj1.past_results[obj1_ind][0] < obj2.past_results[obj2_ind][0])):
                merged_past_results.append(obj1.past_results[obj1_ind])
                obj1_ind += 1
            else:
                merged_past_results.append(obj2.past_results[obj2_ind])
                obj2_ind += 1
        if (obj1.last_frame < obj2.last_frame):
            obj2.past_results = merged_past_results
            obj2.current_result = (obj2.current_result + obj1.current_result) / 2 # Maybe make it weighted
            obj2.trust = max(obj1.trust, obj2.trust)
            return obj1.id # delete obj1
        else:
            obj1.past_results = merged_past_results
            obj1.current_result = (obj2.current_result + obj1.current_result) / 2 # Maybe make it weighted
            obj1.trust = max(obj1.trust, obj2.trust)
            return obj2.id # Delete obj2

    def _update_trust(self, frustum: dict, frame_num: int) -> None:
        # If we've seen an object i times in the past j frames, it is valid
        for obj in self.results.values():
            if obj.last_frame == frame_num: # We see the object
                obj.trust = max(self.max_trust, obj.trust + self.trust_hit)
                if (obj.trust >= self.trust_threshold):
                    obj.is_valid = True
            elif self._object_in_frustum(frustum, obj): # We don't see it but it should be seen
                obj.trust = min(0, obj.trust - self.trust_miss)
                if (obj.trust < self.trust_threshold):
                    obj.is_valid = False

    def _object_in_frustum(self, frustum: dict, obj: FilteredResult) -> bool:
        # Compare the object to all six vectors
        # Check the dot product, if it is < 0, it is outside the frustum
        v1 = obj.current_result - frustum['trf']
        v2 = obj.current_result - frustum['bln']
        return (np.dot(frustum['far'], v1) >= 0
            and np.dot(frustum['right'], v1) >= 0
            and np.dot(frustum['top'], v1) >= 0
            and np.dot(frustum['left'], v2) >= 0
            and np.dot(frustum['bottom'], v2) >= 0
            and np.dot(frustum['near'], v2) >= 0)

    def _get_results(self) -> list[tuple[int, int, np.array]]:
        valid_results: list[tuple[int, int, np.array]] = []
        for oid, obj in self.results.items():
            if (obj.is_valid):
                valid_results.append((oid, obj.cid, obj.current_result))
        return valid_results

    def _dist(self, obj1: FilteredResult, obj2: FilteredResult) -> float:
        # Calculate the distance between two objects
        return np.linalg.norm(obj1.current_result - obj2.current_result)
