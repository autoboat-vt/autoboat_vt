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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import Float32, String
from sensor_msgs.msg import NavSatFix, Image
from autoboat_msgs.msg import ObjectDetectionResultsList, ObjectDetectionResult

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
NUM_IMAGES_TO_SAVE = 500

# These are constants. Don't change these. Needed for a workaround with DeepStream 7.1 and JetPack 6.2
COMPUTE_HW = 1
MEMORY_TYPE = 0

if (IS_DEV_CONTAINER):
    PATH_TO_SRC_DIR = "/home/ws/src"
else:
    PATH_TO_SRC_DIR = "/home/sailbot/autoboat_vt/src"

# YOLO_VER = 11
YOLO_VER = 26

PATH_TO_YOLO_CONFIG = f"{PATH_TO_SRC_DIR}/object_detection/object_detection/deepstream_yolo/config_infer_primary_yolo{YOLO_VER}.txt"
INFERENCE = True
if "INFERENCE" in os.environ and os.environ["INFERENCE"] == "false":
    INFERENCE = False
# MUXER_BATCH_TIMEOUT_USEC = 40_000

if SHOULD_SAVE_IMAGES and not os.path.exists(f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results"):
    os.makedirs(f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results")

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
                "name": self._find_camera(),
                "framerate": "30/1",
                "format": "YUY2",
                "input_width": 1280,
                "input_height": 800
            }
        }

        # ROS2 Initialization
        sensor_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.object_detection_results_publisher = self.create_publisher(msg_type=ObjectDetectionResultsList, topic="/object_detection_results_list", qos_profile=sensor_qos_profile)
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile) # heading is counterclockwise of true east
        self.model_listener = self.create_subscription(msg_type=String, topic="/model", callback=self.model_callback, qos_profile=sensor_qos_profile)
        self.threshold_listener = self.create_subscription(msg_type=Float32, topic="/threshold", callback=self.threshold_callback, qos_profile=sensor_qos_profile)

        self.current_position = {
            "latitude": 0,
            "longitude": 0
        }
        self.current_heading = 0 # default to true east

        # TODO: add subscriber for image topics
        # TODO: add dynamic reconfigure for parameters from telemetry node
        #   Added model and threshold switching.
        # TODO: add localization

        # DeepStream Initialization
        Gst.init(None)
        self.pipeline = None
        self.loop = None
        self.threshold = 0
        self.model = "" # model name without .pt.onnx. Ex. yolo11m.pt.onnx -> yolo11m
        self.config_file_split = []
        self.last_time = time.time()
        self.file_lock = threading.Lock()

        self._init_pipeline()
        self._read_file()
        vs = threading.Thread(target=self.run, daemon=True)
        vs.start()

    def _init_pipeline(self):
        self.pipeline = Gst.Pipeline()

        streammux = Gst.ElementFactory.make("nvstreammux", "muxer")
        streammux.set_property('batch-size', 1)
        
        # v4l2-ctl --list-devices
        # v4l2-ctl --device /dev/video0 --list-formats-ext
        source0 = Gst.ElementFactory.make("v4l2src", "usb-cam-0")
        source0.set_property('device', self.CAM_LIST[0]["name"])
        self.get_logger().info(f"Opening camera device: {self.CAM_LIST[0]['name']}")

        caps_source0 = Gst.ElementFactory.make('capsfilter', 'source0-caps')
        caps_source0.set_property('caps', Gst.Caps.from_string(f'video/x-raw, width={self.CAM_LIST[0]["input_width"]}, height={self.CAM_LIST[0]["input_height"]}, format={self.CAM_LIST[0]["format"]}, framerate={self.CAM_LIST[0]["framerate"]}'))

        # This is a workaround.
        # Issue with deepstream7.1 and jetpack6.2 requires compute-hw to be 1 instead of 0.
        # When compute-hw is 1, nvvidconv fails to convert from YUY2 to NV12 directly.
        # So we convert from YUY2 to RGB first, then from RGB to NV12
        videoconvert = Gst.ElementFactory.make('videoconvert', 'convertor-0')

        caps_videoconvert = Gst.ElementFactory.make('capsfilter', 'convertor-caps-0')
        caps_videoconvert.set_property('caps', Gst.Caps.from_string(f'video/x-raw, format=RGB'))

        nvvidconvsrc0 = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter-src-0')
        nvvidconvsrc0.set_property('nvbuf-memory-type', MEMORY_TYPE)
        nvvidconvsrc0.set_property('compute-hw', COMPUTE_HW)

        caps_nvvidconvsrc0 = Gst.ElementFactory.make('capsfilter', 'nvmm-caps-0')
        caps_nvvidconvsrc0.set_property('caps', Gst.Caps.from_string(f'video/x-raw(memory:NVMM), format=NV12, width={self.CAM_LIST[0]["input_width"]}, height={self.CAM_LIST[0]["input_height"]}'))

        if INFERENCE:
            pgie = Gst.ElementFactory.make('nvinfer', 'pgie')
            pgie.set_property('config-file-path', PATH_TO_YOLO_CONFIG)

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

        nvvidconv_jpeg = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter-jpeg')
        nvvidconv_jpeg.set_property('nvbuf-memory-type', MEMORY_TYPE)
        nvvidconv_jpeg.set_property('compute-hw', COMPUTE_HW)
        
        caps_nvvidconv_jpeg = Gst.ElementFactory.make('capsfilter', 'nvconverter-jpeg-caps')
        if (IS_DEV_CONTAINER):
            caps_nvvidconv_jpeg.set_property('caps', Gst.Caps.from_string('video/x-raw(memory:NVMM), format=I420')) # Dev container needs I420
        else:
            caps_nvvidconv_jpeg.set_property('caps', Gst.Caps.from_string('video/x-raw(memory:NVMM), format=NV12')) # Jetson needs NV12

        jpegenc = Gst.ElementFactory.make('nvjpegenc', 'jpegenc')

        multifilesink = Gst.ElementFactory.make('multifilesink', 'multifilesink')
        multifilesink.set_property('location', f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results/frame%06d.jpg")
        multifilesink.set_property('index', 0)
        multifilesink.set_property('max-files', NUM_IMAGES_TO_SAVE)

        self.pipeline.add(source0)
        self.pipeline.add(caps_source0)
        self.pipeline.add(videoconvert)
        self.pipeline.add(caps_videoconvert)
        self.pipeline.add(nvvidconvsrc0)
        self.pipeline.add(caps_nvvidconvsrc0)
        self.pipeline.add(streammux)
        if INFERENCE:
            self.pipeline.add(pgie)
            self.pipeline.add(tracker)
        self.pipeline.add(queue_multifilesink_valve)
        self.pipeline.add(multifilesink_valve)
        self.pipeline.add(osd)
        self.pipeline.add(nvvidconv_jpeg)
        self.pipeline.add(caps_nvvidconv_jpeg)
        self.pipeline.add(jpegenc)
        self.pipeline.add(multifilesink)

        source0.link(caps_source0)
        caps_source0.link(videoconvert)
        videoconvert.link(caps_videoconvert)
        caps_videoconvert.link(nvvidconvsrc0)
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
        osd.link(nvvidconv_jpeg)
        nvvidconv_jpeg.link(caps_nvvidconv_jpeg)
        caps_nvvidconv_jpeg.link(jpegenc)
        jpegenc.link(multifilesink)

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
    
    def _infer_probe(self, pad, info, u_data):
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            self.get_logger().info("Unable to get GstBuffer")
            return

        msg = ObjectDetectionResultsList()
        msg.detection_results = []
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

            if (frame_meta.frame_num % 60 == 0):
                current_time = time.time()
                fps = 60 / (current_time - self.last_time)
                self.last_time = current_time
                self.get_logger().info(f"Frame: {frame_meta.frame_num}, avg FPS: {fps:.2f}")
            l_obj = frame_meta.obj_meta_list

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
                obj_results.x_position = obj_meta.tracker_bbox_info.org_bbox_coords.left + (obj_meta.tracker_bbox_info.org_bbox_coords.width / 2)
                obj_results.y_position = obj_meta.tracker_bbox_info.org_bbox_coords.top + (obj_meta.tracker_bbox_info.org_bbox_coords.height / 2)
                obj_results.width = obj_meta.tracker_bbox_info.org_bbox_coords.width
                obj_results.height = obj_meta.tracker_bbox_info.org_bbox_coords.height
                obj_results.object_id = obj_meta.object_id
                obj_results.class_id = obj_meta.class_id
                msg.detection_results.append(obj_results)

                try:
                    l_obj = l_obj.next
                except StopIteration:
                    break
            
            try:
                l_frame = l_frame.next
            except StopIteration:
                break
        
        self.object_detection_results_publisher.publish(msg)
        
        return Gst.PadProbeReturn.OK

    def _probe(self, pad, info, u_data):
        self.get_logger().info(u_data)
        return Gst.PadProbeReturn.OK

    def _find_camera(self):
        """
        This is just a way to figure out which /dev/video* is the camera\n
        The camera outputs on 3 devices.\n
        Each device is a different format, but the order can change or extra cameras can cause the number to increase,\n
        We want specifically the YUYV (color) format of the RealSense camera\n
        While this finds the device with YUYV format, it does not guarantee that the correct resolution and framerate are available.
        Returns:
            str: The /dev/video* device path
        """

        camera_devices_output = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True).stdout
        matches = re.findall(r"/dev/video[0-9]*", camera_devices_output)
        for match in matches:
            device_id = match.split("/dev/video")[-1]
            if (re.search("RealSense", subprocess.run(['cat', f'/sys/class/video4linux/video{device_id}/name'], capture_output=True, text=True).stdout) is not None):
                if (re.search("YUYV", subprocess.run(['v4l2-ctl', '--device', f'/dev/video{device_id}', '--list-formats-ext'], capture_output=True, text=True).stdout) is not None):
                    return f"/dev/video{device_id}"
        self.get_logger().error("Could not find RealSense camera device with YUYV format")
        sys.exit(1)

    def _read_file(self):
        # Open file
        self.file_lock.acquire()
        with open(PATH_TO_YOLO_CONFIG, 'r') as file:
            content = file.read()
            self.config_file_split = content.split('\n\n')
        self.file_lock.release()

        # Read model
        onnx_section = self.config_file_split[1]
        onnx_lines = onnx_section.split('\n')
        for line in onnx_lines:
            if line.startswith('onnx-file='):
                self.model = line.split('=')[-1][:-8] # remove .pt.onnx
                break
        
        # Read threshold
        attributes_lines = self.config_file_split[5].split('\n')
        self.threshold = float(attributes_lines[1].split('=')[-1])

    def position_callback(self, msg):
        self.current_position["latitude"] = msg.latitude
        self.current_position["longitude"] = msg.longitude

    def heading_callback(self, msg):
        self.current_heading = msg.data
    
    def model_callback(self, msg):
        # TODO: check if config file was modified externally and reload if so
        self.file_lock.acquire() # Don't want multiple threads writing to the file at once
        new_model = msg.data
        if new_model != self.model and INFERENCE:
            onnx_lines = self.config_file_split[1].split('\n')
            engine_lines = self.config_file_split[2].split('\n')
            labels_lines = self.config_file_split[3].split('\n')

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
            
            # Should we add the new model if not found?
            if not found_model_entry:
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
            self.update_config_file()
            self.get_logger().info(f"Updated model to {new_model}")
        else:
            self.get_logger().info(f"Model is already {new_model}, not updating")
        self.file_lock.release()
    
    def threshold_callback(self, msg):
        self.file_lock.acquire() # Don't want multiple threads writing to the file at once
        new_threshold = float(msg.data)
        if new_threshold != self.threshold:
            if new_threshold >= 0.0 and new_threshold <= 1.0:
                attributes_lines = self.config_file_split[4].split('\n')
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
    
    def update_config_file(self):
        # This doesn't need the file_lock because the caller already has it
        with open(PATH_TO_YOLO_CONFIG, 'w') as file:
            file.write("\n\n".join(self.config_file_split))
        if INFERENCE: # Should this check be here? Is it necessary?
            self.pipeline.get_by_name('pgie').set_property('config-file-path', PATH_TO_YOLO_CONFIG)
            self.get_logger().info("Reloaded config file in nvinfer")
        else:
            self.get_logger().info("Not reloading config file in nvinfer since INFERENCE is disabled")

def main():
    rclpy.init()
    buoy_detection_node = BuoyDetectionNode()
    rclpy.spin(buoy_detection_node)

if __name__ == '__main__':
    sys.exit(main())
