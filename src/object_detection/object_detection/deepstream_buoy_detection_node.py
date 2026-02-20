import os
import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst
import pyds
import threading
import time
import subprocess
import re
from math import tan, pi
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import Float32, String, Int32
from sensor_msgs.msg import NavSatFix, Image
from autoboat_msgs.msg import ObjectDetectionResultsList, ObjectDetectionResult, TriangulationResultsList, TriangulationResult

os.environ["USE_NEW_NVSTREAMMUX"] = "yes"
# os.environ['NVDS_ENABLE_COMPONENT_LATENCY_MEASUREMENT'] = '1'
os.environ['CUDA_VER'] = "12.6"
# os.environ['OPENCV'] = "1" # These are for int8 calibration, not needed here
# os.environ['INT8_CALIB_IMG_PATH'] = "calibration.txt"
# os.environ['INT8_CALIB_BATCH_SIZE'] = "4"
# os.environ['GST_DEBUG'] = "3"

if (re.search("/home/ws", os.getcwd()) is not None):
    IS_DEV_CONTAINER = True
else:
    IS_DEV_CONTAINER = False

SHOULD_SAVE_IMAGES = True
# NUM_IMAGES_TO_SAVE = 10000

# These are constants. Don't change these. Needed for a workaround with DeepStream 7.1 and JetPack 6.2
COMPUTE_HW = 1
MEMORY_TYPE = 0

if (IS_DEV_CONTAINER):
    PATH_TO_SRC_DIR = "/home/ws/src"
else:
    PATH_TO_SRC_DIR = "/home/sailbot/autoboat_vt/src"

# YOLO_VER = 11
YOLO_VER = 26
if "YOLO_VER" in os.environ and os.environ["YOLO_VER"] in ["11", "26"]:
    YOLO_VER = int(os.environ["YOLO_VER"])

YOLO_CONFIG = {
    11: f"{PATH_TO_SRC_DIR}/object_detection/object_detection/deepstream_yolo/config_infer_primary_yolo11.txt",
    26: f"{PATH_TO_SRC_DIR}/object_detection/object_detection/deepstream_yolo/config_infer_primary_yolo26.txt"
}
INFERENCE = True
if "INFERENCE" in os.environ and os.environ["INFERENCE"] == "false":
    INFERENCE = False
# MUXER_BATCH_TIMEOUT_USEC = 40_000

# if SHOULD_SAVE_IMAGES and not os.path.exists(f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results"):
#     os.makedirs(f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results")


class ObjectDetection:
    def __init__(self, frame_number=-1, detector_confidence=0.0, tracker_confidence=0.0,
                 x_position=0.0, y_position=0.0, width=0.0, height=0.0, object_id=-1,
                 class_id=-1, obj_label=None, pose_matrix=None, camera_matrix_inv=None):
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

        R = pose_matrix[:3, :3]
        t = pose_matrix[:3, 3]

        ray_dir = R @ bearing_cam
        ray_dir = ray_dir / np.linalg.norm(ray_dir)
        self.triangle_position = (t.reshape(3, 1), ray_dir.reshape(3, 1))


class ObjectTrack:
    def __init__(self, object_id=-1, class_id=-1, obj_label=None):
        self.detection_results = []
        self.last_updated_frame_number = -1
        self.object_id = object_id
        self.class_id = class_id
        self.obj_label = obj_label
        self.last_world_pos = None
    
    def add_detection(self, detection: ObjectDetection, buffer_window):
        self.detection_results.append(detection)
        while len(self.detection_results) > buffer_window:
            self.detection_results.pop(0)


class ObjectTriangulator:
    def __init__(self, camera_matrix, frame_size):
        self.K = camera_matrix
        self.K_inv = np.linalg.inv(camera_matrix)
        self.observations = {}
        self.frame_size = frame_size
        self.buffer_window = 300
        """
        {obj_id: ObjectTrack}
        """

    def add_observation(self, detection: ObjectDetection):
        obj_id = detection.object_id
        if (detection.x_position < 0 or detection.y_position < 0 or detection.x_position >= self.frame_size[0] or detection.y_position >= self.frame_size[1] - 1):
            return
        if obj_id not in self.observations:
            self.observations[obj_id] = ObjectTrack(object_id=detection.object_id, class_id=detection.class_id, obj_label=detection.obj_label)
        self.observations[obj_id].add_detection(detection, self.buffer_window)
    
    def triangulate(self, obj_id):
        track = self.observations[obj_id]
        obs = track.detection_results
        if len(obs) < 2:
            return None # Need at least 2 observations to triangulate
        
        if (obs[-1].frame_number == track.last_updated_frame_number):
            return track.last_world_pos # If we already triangulated this frame, return the last result

        first_dir = obs[0].triangle_position[1]
        last_dir = obs[-1].triangle_position[1]
        dot_prod = float(np.dot(first_dir.T, last_dir))
        if abs(dot_prod) > 0.99: # If the rays are almost parallel, we can't triangulate accurately
            return None
        
        A = np.zeros((3, 3))
        b = np.zeros((3, 1))

        for detection in obs:
            p, d = detection.triangle_position
            identity_minus_ddt = np.eye(3) - (d @ d.T)
            A += identity_minus_ddt
            b += identity_minus_ddt @ p
        
        world_pos = np.linalg.lstsq(A, b, rcond=None)[0].flatten()
        track.last_world_pos = world_pos
        track.last_updated_frame_number = obs[-1].frame_number
        return world_pos


class BuoyDetectionNode(Node):
    """
    Handles the Object Detection using DeepStream and YOLO models.

    Credit:<br>
    marcoslucianops (https://github.com/marcoslucianops/DeepStream-Yolo)<br>
    DeepStream SDK Documentation (https://docs.nvidia.com/metropolis/deepstream/7.1/text/DS_Overview.html)<br>
    DeepStream Python Examples (https://github.com/NVIDIA-AI-IOT/deepstream_python_apps)<br>
    Ultralytics https://www.ultralytics.com/
    """
    def __init__(self):
        super().__init__("buoy_detection")
        self.CAM_LIST = {
            0: {
                "name": self._find_camera("YUYV"),
                "framerate": "30/1",
                "format": "YUY2",
                "input_width": 1280,
                "input_height": 800
            }
        }

        self.camera_baseline = 59 # Distance between cameras in mm
        self.camera_HFOV = 90 # Horizontal Field of View in degrees (from datasheet)
        
        # Focal length of the camera is 1.93 mm or 640 px.
        self.camera_focal_px = (self.CAM_LIST[0]["input_width"] * 0.5) / tan(self.camera_HFOV * 0.5 * pi / 180) # 640 px
        self.camera_K = np.array([[self.camera_focal_px, 0, self.CAM_LIST[0]["input_width"] * 0.5],
                             [0, self.camera_focal_px, self.CAM_LIST[0]["input_height"] * 0.5],
                             [0, 0, 1]])
        self.camera_K_inv = np.linalg.inv(self.camera_K)
        self.triangulator = ObjectTriangulator(camera_matrix=self.camera_K, frame_size=(self.CAM_LIST[0]["input_width"], self.CAM_LIST[0]["input_height"]))
        self.triangulation_lock = threading.Lock()
        self.iou_threshold = 10.0 # If two detections are less than this distance apart, they are considered the same object and the older one is deleted. This is to prevent duplicate detections from multiple cameras or noisy detections.
        self.update_frequency = 0.5 # seconds. How often to publish detection results. We can only publish the most recent detection for each object, so we don't need to publish every frame.

        # ROS2 Initialization
        sensor_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.object_detection_results_publisher = self.create_publisher(msg_type=ObjectDetectionResultsList, topic="/object_detection_results_list", qos_profile=sensor_qos_profile)
        self.triangulation_results_publisher = self.create_publisher(msg_type=TriangulationResultsList, topic="/triangulation_results_list", qos_profile=sensor_qos_profile)
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile) # heading is counterclockwise of true east
        self.model_listener = self.create_subscription(msg_type=String, topic="/model", callback=self.model_callback, qos_profile=sensor_qos_profile)
        self.threshold_listener = self.create_subscription(msg_type=Float32, topic="/threshold", callback=self.threshold_callback, qos_profile=sensor_qos_profile)
        self.iou_threshold_listener = self.create_subscription(msg_type=Float32, topic="/iou_threshold", callback=self.iou_threshold_callback, qos_profile=sensor_qos_profile)
        self.update_frequency_listener = self.create_subscription(msg_type=Float32, topic="/update_frequency", callback=self.update_frequency_callback, qos_profile=sensor_qos_profile)
        self.buffer_window_listener = self.create_subscription(msg_type=Int32, topic="/buffer_window", callback=self.buffer_window_callback, qos_profile=sensor_qos_profile)

        self.current_position = {
            "latitude": 0,
            "longitude": 0
        }
        self.origin_position = {
            "latitude": 0,
            "longitude": 0
        }
        self.current_heading = 0 # default to true east

        # TODO: add subscriber for image topics
        # TODO: add localization

        # DeepStream Initialization
        Gst.init(None)
        self.pipeline = None
        self.loop = None

        # Config file parameters
        self.threshold = 0
        self.model = "" # model name without .pt.onnx. Ex. yolo11m.pt.onnx -> yolo11m
        self.config_file_split = []
        self.last_time = time.time()
        self.file_lock = threading.Lock()
        self.last_published_frame_number = -1
        with self.file_lock:
            self.config_file_split, self.model, self.threshold = self._read_file()

        self._init_pipeline()
        vs = threading.Thread(target=self.run, daemon=True)
        vs.start()

        if INFERENCE:
            self.timer = self.create_timer(timer_period_sec=self.update_frequency, callback=self._iterate_results)

    def _init_pipeline(self):
        self.pipeline = Gst.Pipeline()

        streammux = Gst.ElementFactory.make("nvstreammux", "muxer")
        streammux.set_property('batch-size', 1)
        
        # v4l2-ctl --list-devices
        # v4l2-ctl --device /dev/video0 --list-formats-ext
        source0 = Gst.ElementFactory.make("v4l2src", "usb-cam-0")
        source0.set_property('device', self.CAM_LIST[0]["name"])
        source0.set_property('brightness', 0)
        self.get_logger().info(f"Opening camera device: {self.CAM_LIST[0]['name']}")

        """
        v4l2 camera settings
        brightness: [-64, 64]
        hue: [-180, 180]
        contrast: [0, 100]
        saturation: [0, 100]
        """

        caps_source0 = Gst.ElementFactory.make('capsfilter', 'source0-caps')
        caps_source0.set_property('caps', Gst.Caps.from_string(f'video/x-raw, width={self.CAM_LIST[0]["input_width"]}, height={self.CAM_LIST[0]["input_height"]}, format={self.CAM_LIST[0]["format"]}, framerate={self.CAM_LIST[0]["framerate"]}'))

        # This is a workaround.
        # Issue with deepstream7.1 and jetpack6.2 requires compute-hw to be 1 instead of 0.
        # When compute-hw is 1, nvvidconv fails to convert from YUY2 to NV12 directly.
        # So we convert from YUY2 to RGB first, then from RGB to NV12
        videoconvert0 = Gst.ElementFactory.make('videoconvert', 'convertor-0')

        caps_videoconvert0 = Gst.ElementFactory.make('capsfilter', 'convertor-caps-0')
        caps_videoconvert0.set_property('caps', Gst.Caps.from_string(f'video/x-raw, format=RGB'))

        nvvidconvsrc0 = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter-src-0')
        nvvidconvsrc0.set_property('nvbuf-memory-type', MEMORY_TYPE)
        nvvidconvsrc0.set_property('compute-hw', COMPUTE_HW)
        nvvidconvsrc0.set_property('flip-method', 2)

        caps_nvvidconvsrc0 = Gst.ElementFactory.make('capsfilter', 'nvmm-caps-0')
        caps_nvvidconvsrc0.set_property('caps', Gst.Caps.from_string(f'video/x-raw(memory:NVMM), format=NV12, width={self.CAM_LIST[0]["input_width"]}, height={self.CAM_LIST[0]["input_height"]}'))

        if INFERENCE:
            pgie = Gst.ElementFactory.make('nvinfer', 'pgie')
            pgie.set_property('config-file-path', YOLO_CONFIG[YOLO_VER])
            self.get_logger().info(f"Running Inference with Yolo{YOLO_VER}")

            tracker = Gst.ElementFactory.make('nvtracker', 'tracker')
            # docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvtracker.html#nvidia-tao-reidentificationnet
            tracker.set_property('ll-lib-file', '/opt/nvidia/deepstream/deepstream-7.1/lib/libnvds_nvmultiobjecttracker.so')
            # tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_IOU.yml')
            # tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_NvSORT.yml')
            # tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_NvDeepSORT.yml')
            # tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_NvDCF_max_perf.yml')
            # tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_NvDCF_perf.yml')
            # tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_NvDCF_accuracy.yml')
            tracker.set_property('ll-config-file', f'{PATH_TO_SRC_DIR}/object_detection/object_detection/deepstream_yolo/config_tracker_NvDCF_perf.yml')
            # tracker.set_property('compute-hw', COMPUTE_HW)
            tracker.set_property('tracking-id-reset-mode', 0)
        
        queue_multifilesink_valve = Gst.ElementFactory.make('queue', 'queue-valve')

        multifilesink_valve = Gst.ElementFactory.make('valve', 'multifilesink-valve')
        multifilesink_valve.set_property('drop', not SHOULD_SAVE_IMAGES)

        osd = Gst.ElementFactory.make('nvdsosd', 'nvosd')

        # sink_tee = Gst.ElementFactory.make('tee', 'sink-tee')

        # nvvidconv_jpeg = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter-jpeg')
        # nvvidconv_jpeg.set_property('nvbuf-memory-type', MEMORY_TYPE)
        # nvvidconv_jpeg.set_property('compute-hw', COMPUTE_HW)
        
        # caps_nvvidconv_jpeg = Gst.ElementFactory.make('capsfilter', 'nvconverter-jpeg-caps')
        # if (IS_DEV_CONTAINER):
        #     caps_nvvidconv_jpeg.set_property('caps', Gst.Caps.from_string('video/x-raw(memory:NVMM), format=I420')) # Dev container needs I420
        # else:
        #     caps_nvvidconv_jpeg.set_property('caps', Gst.Caps.from_string('video/x-raw(memory:NVMM), format=NV12')) # Jetson needs NV12

        # jpegenc = Gst.ElementFactory.make('nvjpegenc', 'jpegenc')

        # multifilesink = Gst.ElementFactory.make('multifilesink', 'multifilesink')
        # multifilesink.set_property('location', f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results/frame%06d.jpg")
        # multifilesink.set_property('index', 0)
        # multifilesink.set_property('max-files', NUM_IMAGES_TO_SAVE)

        sink = Gst.ElementFactory.make('nveglglessink', 'sink')
        sink.set_property('sync', False)

        self.pipeline.add(source0)
        self.pipeline.add(caps_source0)
        self.pipeline.add(videoconvert0)
        self.pipeline.add(caps_videoconvert0)
        self.pipeline.add(nvvidconvsrc0)
        self.pipeline.add(caps_nvvidconvsrc0)
        self.pipeline.add(streammux)
        if INFERENCE:
            self.pipeline.add(pgie)
            self.pipeline.add(tracker)
        self.pipeline.add(queue_multifilesink_valve)
        self.pipeline.add(multifilesink_valve)
        self.pipeline.add(osd)
        # self.pipeline.add(sink_tee)
        # self.pipeline.add(nvvidconv_jpeg)
        # self.pipeline.add(caps_nvvidconv_jpeg)
        # self.pipeline.add(jpegenc)
        # self.pipeline.add(multifilesink)
        self.pipeline.add(sink)

        source0.link(caps_source0)
        caps_source0.link(videoconvert0)
        videoconvert0.link(caps_videoconvert0)
        caps_videoconvert0.link(nvvidconvsrc0)
        nvvidconvsrc0.link(caps_nvvidconvsrc0)

        sinkpad0 = streammux.request_pad_simple('sink_0')
        srcpad0 = caps_nvvidconvsrc0.get_static_pad('src')
        srcpad0.link(sinkpad0)

        if INFERENCE:
            streammux.link(pgie)
            pgie.link(tracker)
            tracker.link(queue_multifilesink_valve)
        else:
            streammux.link(queue_multifilesink_valve)
        queue_multifilesink_valve.link(multifilesink_valve)
        multifilesink_valve.link(osd)
        osd.link(sink)

        # osd.link(sink_tee)
        # sink_tee.link(sink)

        # sink_tee.link(nvvidconv_jpeg)
        # nvvidconv_jpeg.link(caps_nvvidconv_jpeg)
        # caps_nvvidconv_jpeg.link(jpegenc)
        # jpegenc.link(multifilesink)

        # TODO: Make file saving a toggleable option

        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._bus_call, self.loop)

        infer_probe_pad = queue_multifilesink_valve.get_static_pad('sink')
        infer_probe_pad.add_probe(Gst.PadProbeType.BUFFER, self._infer_probe, 0)

    def _bus_call(self, bus, message, loop):
        t = message.type
        if t == Gst.MessageType.EOS:
            # sys.stdout.write("End-of-stream\n")
            self.get_logger().info("End-of-stream\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            # sys.stderr.write("Warning: %s: %s\n" % (err, debug))
            self.get_logger().warn("Warning: %s: %s\n" % (err, debug))
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            # sys.stderr.write("Error: %s: %s\n" % (err, debug))
            self.get_logger().error("Error: %s: %s\n" % (err, debug))
            self.close_pipeline()
        return True
    
    def run(self):
        self.get_logger().info("Starting pipeline\n")
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop.run()
        except Exception as e:
            self.get_logger().error(e)
        self.get_logger().info("Closing pipeline\n")
        self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info("Closed pipeline\n")
    
    def close_pipeline(self):
        self.get_logger().info("Quitting pipeline\n")
        self.pipeline.set_state(Gst.State.NULL)
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.loop.quit()
        # Trigger ROS2 node shutdown to exit the process
        self.get_logger().info("Shutting down ROS2 node\n")
        rclpy.shutdown()

    def _infer_probe(self, pad, info, u_data):
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            self.get_logger().info("Unable to get GstBuffer")
            return

        msg = ObjectDetectionResultsList()
        msg.detection_results = []
        # object_list = ObjectTracker()

        # Retrieve batch metadata from the gst_buffer
        # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
        # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
        l_frame = batch_meta.frame_meta_list
        while l_frame is not None:
            # When batch size is only 1, this only iterates once
            try:
                # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
                # The casting is done by pyds.NvDsFrameMeta.cast()
                # The casting also keeps ownership of the underlying memory
                # in the C code, so the Python garbage collector will leave
                # it alone.
                frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
            except StopIteration:
                break
            
            msg.ntp_timestamp = frame_meta.ntp_timestamp

            if (frame_meta.source_id == 0):
                # object_list.update_frame_number(frame_meta.frame_num)
                # object_list.update_timestamp(frame_meta.ntp_timestamp)
                if (frame_meta.frame_num % 60 == 0):
                    current_time = time.time()
                    fps = 60 / (current_time - self.last_time)
                    self.last_time = current_time
                    self.get_logger().info(f"Frame: {frame_meta.frame_num}, avg FPS: {fps:.2f}")
            l_obj = frame_meta.obj_meta_list

            # if (self.origin_position["latitude"] == 0 and self.origin_position["longitude"] == 0):
            #     return Gst.PadProbeReturn.OK # Don't process frames until we have a valid position
            pose_matrix = self._get_current_pose(self.current_position["latitude"], self.current_position["longitude"], self.current_heading)

            # Iterate through each object in frame
            while l_obj is not None:
                try:
                    # Casting l_obj.data to pyds.NvDsObjectMeta
                    obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
                except StopIteration:
                    break
                
                obj_results = ObjectDetectionResult()
                obj_results.detector_confidence = obj_meta.confidence
                obj_results.tracker_confidence = obj_meta.tracker_confidence
                mid_x = obj_meta.tracker_bbox_info.org_bbox_coords.left + (obj_meta.tracker_bbox_info.org_bbox_coords.width / 2)
                mid_y = obj_meta.tracker_bbox_info.org_bbox_coords.top + (obj_meta.tracker_bbox_info.org_bbox_coords.height / 2)
                obj_results.x_position = mid_x
                obj_results.y_position = mid_y
                obj_results.width = obj_meta.tracker_bbox_info.org_bbox_coords.width
                obj_results.height = obj_meta.tracker_bbox_info.org_bbox_coords.height
                obj_results.object_id = obj_meta.object_id
                obj_results.class_id = obj_meta.class_id
                obj_results.angle_to_object = np.arctan((mid_x - self.CAM_LIST[0]["input_width"] * 0.5) / self.camera_focal_px) * 180 / pi
                msg.detection_results.append(obj_results)
                with self.triangulation_lock:
                    if (self.origin_position["latitude"] != 0 and self.origin_position["longitude"] != 0):
                        self.triangulator.add_observation(ObjectDetection(
                                                            frame_number = frame_meta.frame_num,
                                                            detector_confidence = obj_meta.confidence,
                                                            tracker_confidence = obj_meta.tracker_confidence,
                                                            x_position = mid_x,
                                                            y_position = mid_y,
                                                            width = obj_meta.tracker_bbox_info.org_bbox_coords.width,
                                                            height = obj_meta.tracker_bbox_info.org_bbox_coords.height,
                                                            object_id = obj_meta.object_id,
                                                            class_id = obj_meta.class_id,
                                                            obj_label = obj_meta.obj_label,
                                                            pose_matrix = pose_matrix,
                                                            camera_matrix_inv = self.camera_K_inv
                        ))
                    if obj_meta.object_id in self.triangulator.observations:
                        track = self.triangulator.observations[obj_meta.object_id]
                        if track.last_world_pos is not None:
                            x_world, y_world, z_world = track.last_world_pos
                            obj_meta.text_params.display_text = f"{obj_meta.obj_label} {obj_meta.object_id}. Pos: [ {x_world:.01f}, {y_world:.01f}, {z_world:.01f} ]"
                        else:
                            obj_meta.text_params.display_text = f"{obj_meta.obj_label} {obj_meta.object_id}. Pos: N/A"

                try:
                    l_obj = l_obj.next
                except StopIteration:
                    break
            
            try:
                l_frame = l_frame.next
            except StopIteration:
                break
        
        display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1
        py_nvosd_text_params = display_meta.text_params[0]
        py_nvosd_text_params.display_text = f"Yolo Version: {YOLO_VER}\nCurrent Model: {self.model}\nThreshold: {self.threshold}"
   
        # Set the offsets where the string should appear
        py_nvosd_text_params.x_offset = 10
        py_nvosd_text_params.y_offset = 12

        # Font , font-color and font-size
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 12
        # set(red, green, blue, alpha); set to White
        py_nvosd_text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)

        # Text background color
        py_nvosd_text_params.set_bg_clr = 1
        # set(red, green, blue, alpha); set to Black
        py_nvosd_text_params.text_bg_clr.set(0.0, 0.0, 0.0, 0.25)

        self.object_detection_results_publisher.publish(msg)
        
        return Gst.PadProbeReturn.OK

    def _get_current_pose(self, lat, long, heading):
        R_earth = 6378137.0  # Radius of Earth in meters
    
        # Delta lat/lon in radians
        d_lat = np.radians(lat - self.origin_position["latitude"])
        d_lon = np.radians(long - self.origin_position["longitude"])
        
        # Calculate Easting (x) and Northing (y)
        t_y = d_lat * R_earth
        t_x = d_lon * R_earth * np.cos(np.radians(self.origin_position["latitude"]))
        t_z = 0  # Assume sea level
        # if (self.origin_position["latitude"] != 0 and self.origin_position["longitude"] != 0):
        #     self.get_logger().info(f"Pose translation: {t_x} {t_y}")
        
        # 2. Calculate Rotation (R) from Heading
        psi = np.radians(heading)
        cos_p = np.cos(psi)
        sin_p = np.sin(psi)
        R_yaw = np.array([
            [ np.cos(psi), -np.sin(psi), 0],
            [ np.sin(psi),  np.cos(psi), 0],
            [ 0,            0,           1]
        ])
        
        # Columns are world-space representations of Camera X, Y, and Z
        # R = np.array([
        #     [ cos_p,  0,  sin_p],
        #     [-sin_p,  0,  cos_p],
        #     [     0, -1,      0]
        # ])
        R_cam_to_enu = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])

        R = R_yaw @ R_cam_to_enu
        
        # 3. Assemble 4x4 Matrix
        pose = np.eye(4)
        pose[:3, :3] = R
        pose[:3, 3] = [t_x, t_y, t_z]
        
        return pose

    def _iterate_results(self):
        # if (self.current_object_list.frame_number == self.last_published_frame_number):
        #     return
        with self.triangulation_lock:
            observations_snapshot = {
                obj_id: track 
                for obj_id, track in self.triangulator.observations.items()
            }

        detections = {}
        for obj_id, obs_track in observations_snapshot.items():
            # obs_list = obs_track.detection_results
            world_pos = self.triangulator.triangulate(obj_id)
            if world_pos is not None:
                self.get_logger().info(f"{obs_track.obj_label} {obj_id} triangulated position: {world_pos}")
                lat = self.origin_position["latitude"] + (world_pos[1] / 6378137.0) * (180 / pi)
                lon = self.origin_position["longitude"] + (world_pos[0] / (6378137.0 * np.cos(np.radians(self.origin_position["latitude"])))) * (180 / pi)
                self.get_logger().info(f"Object {obj_id} triangulated GPS position: ({lat}, {lon})")
                detections[obj_id] = {
                    "label": obs_track.obj_label,
                    "class_id": obs_track.class_id,
                    "world_pos": world_pos,
                    "lat": lat,
                    "lon": lon,
                    "last_updated_frame_number": obs_track.last_updated_frame_number
                }
            else:
                self.get_logger().info(f"Object {obj_id} does not have enough observations to triangulate")
        
        self._filter_results(detections)
        self._publish_results(detections)

    def _filter_results(self, detections):
        """
        Define some nms/iou filtering.
        If 2 objects are very close together, we can assume they are the same object and only publish one of them.
        If overlapping detections have a recent detection and one with an old detection, we can delete the old detection and keep the new one.

        If the distance between 2 detections is less than 1 meter, we can assume they are the same object and only publish the most recent one.
        If an overlapping detection is old enough, just delete the track
        """
        ids_to_delete = []
        for obj_id_1, det1 in detections.items():
            for obj_id_2, det2 in detections.items():
                if obj_id_1 >= obj_id_2 or det1["class_id"] != det2["class_id"]: # Don't compare the same pair twice or with different classes
                    continue
                dist = np.linalg.norm(det1["world_pos"] - det2["world_pos"])
                if dist < self.iou_threshold: # If detections are less than this many meters apart, keep the most recent one
                    if det1["last_updated_frame_number"] > det2["last_updated_frame_number"]:
                        ids_to_delete.append(obj_id_2)
                        #self.get_logger().info(f"Deleted detection {obj_id_2} because it was too close to {obj_id_1} and older")
                    else:
                        ids_to_delete.append(obj_id_1)
                        #self.get_logger().info(f"Deleted detection {obj_id_1} because it was too close to {obj_id_2} and older")

        for obj_id in ids_to_delete:
            del detections[obj_id]

    def _publish_results(self, detections):
        results_list = TriangulationResultsList()
        for obj_id, det in detections.items():
            result = TriangulationResult()
            result.object_id = obj_id
            result.label = det["label"]
            result.class_id = det["class_id"]
            result.latitude = det["lat"]
            result.longitude = det["lon"]
            results_list.results.append(result)
        self.triangulation_results_publisher.publish(results_list)

    def _probe(self, pad, info, u_data):
        self.get_logger().info(u_data)
        return Gst.PadProbeReturn.OK

    def _find_camera(self, format):
        """
        This is just a way to figure out which /dev/video* is the camera\n
        The camera outputs on 3 devices.\n
        Each device is a different format, but the order can change or extra cameras can cause the number to increase,\n
        While this finds the device with the specified format, it does not guarantee that the correct resolution and framerate are available.
        Returns:
            str: The /dev/video* device path
        """

        camera_devices_output = subprocess.run(['ls', '/sys/class/video4linux/'], capture_output=True, text=True).stdout
        for device in camera_devices_output.splitlines():
            if (re.search("RealSense", subprocess.run(['cat', f'/sys/class/video4linux/{device}/name'], capture_output=True, text=True).stdout) is not None):
                if (re.search(format, subprocess.run(['v4l2-ctl', '--device', f'/dev/{device}', '--list-formats'], capture_output=True, text=True).stdout) is not None):
                    return f"/dev/{device}"
        self.get_logger().error(f"Could not find RealSense camera device with {format} format")
        sys.exit(1)

    def _read_file(self, file_name=YOLO_CONFIG[YOLO_VER]):
        # Open file
        # Don't need the file lock becuase the caller has it.
        with open(file_name, 'r') as file:
            content = file.read()
            config_file_split = content.split('\n\n')

        # Read model
        onnx_section = config_file_split[1]
        onnx_lines = onnx_section.split('\n')
        for line in onnx_lines:
            if line.startswith('onnx-file='):
                model = line.split('=')[-1][13:-8] # remove .pt.onnx
                break
        
        # Read threshold
        attributes_lines = config_file_split[5].split('\n')
        threshold = float(attributes_lines[1].split('=')[-1])
        return (config_file_split, model, threshold)

    def position_callback(self, msg):
        self.current_position["latitude"] = msg.latitude
        self.current_position["longitude"] = msg.longitude
        if self.origin_position["latitude"] == 0 and self.origin_position["longitude"] == 0:
            self.origin_position["latitude"] = msg.latitude
            self.origin_position["longitude"] = msg.longitude

    def heading_callback(self, msg):
        self.current_heading = msg.data
    
    def model_callback(self, msg):
        global YOLO_VER
        # TODO: check if config file was modified externally and reload if so
        self.file_lock.acquire() # Don't want multiple threads writing to the file at once
        new_model = msg.data
        if new_model != self.model and INFERENCE:
            onnx_lines, engine_lines, labels_lines, found_model_entry = self._modify_config_lines(self.config_file_split, new_model)
            
            # Should we add the new model if not found?
            self.get_logger().info(f"Model entry found in current config: {found_model_entry}")
            if not found_model_entry:
                if YOLO_VER == 11:
                    split_lines = self._read_file(YOLO_CONFIG[26])[0]
                else:
                    split_lines = self._read_file(YOLO_CONFIG[11])[0]
                onnx_lines, engine_lines, labels_lines, found_model_entry = self._modify_config_lines(split_lines, new_model)
                if found_model_entry:
                    YOLO_VER = 26 if YOLO_VER == 11 else 11
                    attributes_lines = split_lines[5].split('\n')
                    attributes_lines[1] = f"pre-cluster-threshold={self.threshold}"
                    split_lines[5] = "\n".join(attributes_lines)
                    self.config_file_split = split_lines
                    self.get_logger().info(f"Model entry found in alternate config, switching to Yolo{YOLO_VER}")
                else:
                    self.get_logger().info(f"Model {new_model}.pt.onnx not found, not updating model")
                    self.file_lock.release()
                    return
            
            onnx_content = "\n".join(onnx_lines)
            engine_content = "\n".join(engine_lines)
            labels_content = "\n".join(labels_lines)
            self.config_file_split[1] = onnx_content
            self.config_file_split[2] = engine_content
            self.config_file_split[3] = labels_content
            self.model = new_model
            self.update_config_file(YOLO_CONFIG[YOLO_VER])
            self.get_logger().info(f"Updated model to {new_model}")
        else:
            self.get_logger().info(f"Model is already {new_model}, not updating")
        self.file_lock.release()

    def _modify_config_lines(self, lines_split, new_model):
        onnx_lines = lines_split[1].split('\n')
        engine_lines = lines_split[2].split('\n')
        labels_lines = lines_split[3].split('\n')
        # TODO: Add support for switching between yolo11 and yolo26

        found_model_entry = False
        for i in range(len(onnx_lines)):
            if onnx_lines[i].startswith('onnx-file='):
                onnx_lines[i] = "#" + onnx_lines[i]
            if onnx_lines[i] == f"#onnx-file=./onnx_files/{new_model}.pt.onnx":
                onnx_lines[i] = onnx_lines[i][1:] # uncomment line so the model can be used
                found_model_entry = True
        
        for i in range(len(engine_lines)):
            if engine_lines[i].startswith('model-engine-file='):
                engine_lines[i] = "#" + engine_lines[i]
            if engine_lines[i] == f"#model-engine-file=./engine_files/{new_model}_model_b1_gpu0_fp16.engine":
                engine_lines[i] = engine_lines[i][1:] # uncomment line so the model can be used

        for i in range(len(labels_lines)):
            if labels_lines[i].startswith('labelfile-path='):
                labels_lines[i] = "#" + labels_lines[i]
            if labels_lines[i] == f"#labelfile-path=./label_files/{new_model}_labels.txt":
                labels_lines[i] = labels_lines[i][1:] # uncomment line so the model can be used
        
        return (onnx_lines, engine_lines, labels_lines, found_model_entry)

    def threshold_callback(self, msg):
        self.file_lock.acquire() # Don't want multiple threads writing to the file at once
        new_threshold = float(msg.data)
        if new_threshold != self.threshold:
            if new_threshold >= 0.0 and new_threshold <= 1.0:
                attributes_lines = self.config_file_split[5].split('\n')
                attributes_lines[1] = f"pre-cluster-threshold={new_threshold}"
                self.config_file_split[4] = "\n".join(attributes_lines)
                self.threshold = new_threshold
                self.update_config_file()
                self.get_logger().info(f"Updated threshold to {new_threshold}")
            else:
                self.get_logger().info(f"Threshold {new_threshold} is out of range [0.0, 1.0], not updating")
        else:
            self.get_logger().info(f"Threshold is already {new_threshold}, not updating")
        self.file_lock.release()
    
    def iou_threshold_callback(self, msg):
        new_iou_threshold = float(msg.data)
        if new_iou_threshold != self.iou_threshold:
            self.iou_threshold = new_iou_threshold
            self.get_logger().info(f"Updated iou threshold to {new_iou_threshold}")
        else:
            self.get_logger().info(f"IoU threshold is already {new_iou_threshold}, not updating")

    def update_frequency_callback(self, msg):
        new_update_frequency = float(msg.data)
        if hasattr(self, 'timer'):
            if new_update_frequency != self.update_frequency:
                self.update_frequency = new_update_frequency
                self.timer.cancel()
                self.timer = self.create_timer(timer_period_sec=self.update_frequency, callback=self._iterate_results)
                self.get_logger().info(f"Updated update frequency to {new_update_frequency}")
            else:
                self.get_logger().info(f"Update frequency is already {new_update_frequency}, not updating")
        else:
            self.get_logger().info("Inference is disabled, not updating update frequency")
    
    def buffer_window_callback(self, msg):
        new_buffer_window = int(msg.data)
        if new_buffer_window != self.triangulator.buffer_window:
            self.triangulator.buffer_window = new_buffer_window
            self.get_logger().info(f"Updated buffer window to {new_buffer_window}")
        else:
            self.get_logger().info(f"Buffer window is already {new_buffer_window}, not updating")

    def update_config_file(self, file_name=YOLO_CONFIG[YOLO_VER]):
        # This doesn't need the file_lock because the caller already has it
        with open(file_name, 'w') as file:
            file.write("\n\n".join(self.config_file_split))
        if INFERENCE: # Should this check be here? Is it necessary?
            self.pipeline.get_by_name('pgie').set_property('config-file-path', file_name)
            self.get_logger().info("Reloaded config file in nvinfer")
        else:
            self.get_logger().info("Not reloading config file in nvinfer since INFERENCE is disabled")


def main():
    rclpy.init()
    buoy_detection_node = BuoyDetectionNode()
    rclpy.spin(buoy_detection_node)

if __name__ == '__main__':
    sys.exit(main())
