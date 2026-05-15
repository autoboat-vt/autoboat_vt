# This is equivalent to detection results of a single frame.
# This array is a list of all objects seen in the frame
autoboat_msgs/ObjectDetectionResult[] detection_results

# This is the ntp_timestamp of the frame reaching the muxer
# https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_NTP_Timestamp.html
# Dividing this number by 1,000,000,000 appears to provide the date from Unix epoch (January 1, 1970)
# with the precision of seconds. Everything else is fractional seconds.
int64 ntp_timestamp

# This is the name of the model
string model_name

# This is the version of the model, e.g. 11 for YOLOv11 or 26 for YOLOv26
int8 yolo_version

# Threshold used for filtering detections, between 0 and 1. Detections with confidence scores below this threshold are discarded.
float32 threshold