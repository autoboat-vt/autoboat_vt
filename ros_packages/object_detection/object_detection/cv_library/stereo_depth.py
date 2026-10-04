IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"

CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"

class StereoEstimation:
    def __init__(self):
        self.pairs = {}
        # { id_left: [ids_right], id_right: [ids_left]}
    
    def process_stereo(self, objs_left, objs_right):
        pass

    def _find_pair(self, obj_left_id, objs_right):
        
        pass
