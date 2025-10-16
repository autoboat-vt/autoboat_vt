# Detection results for an object in a frame

float32 detector_confidence
float32 tracker_confidence

# X and Y positions are for the center of the object
# These are from the tracker, not detector
float32 x_position
float32 y_position
float32 width
float32 height

int32 object_id
int32 class_id
