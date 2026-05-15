autoboat_msgs/TriangulationResult[] triangulation_results
# This message contains a list of triangulation results for multiple objects.

float32 iou_threshold
# If two detections are less than this distance apart in meters, they are considered to be the same object and will be merged into a single triangulation result.