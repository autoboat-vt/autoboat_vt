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
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Int32
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
NUM_IMAGES_TO_SAVE = 1000

COMPUTE_HW = 1
MEMORY_TYPE = 0
LATENCY = False

if (IS_DEV_CONTAINER):
    PATH_TO_SRC_DIR = "/home/ws/src"
else:
    PATH_TO_SRC_DIR = "/home/sailbot/autoboat_vt/src"

PATH_TO_YOLO_CONFIG = f"{PATH_TO_SRC_DIR}/object_detection/object_detection/deepstream_yolo/config_infer_primary_yolo11.txt"
INFERENCE = True
# MUXER_BATCH_TIMEOUT_USEC = 40_000

if SHOULD_SAVE_IMAGES and not os.path.exists(f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results"):
    os.makedirs(f"{PATH_TO_SRC_DIR}/object_detection/object_detection/frame_results")

class BuoyDetectionNode(Node):
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

        # TODO: add subscriber for image topics
        # TODO: add dynamic reconfigure for parameters from telemetry node
        # TODO: add localization

        # DeepStream Initialization
        Gst.init(None)
        self.pipeline = None
        self.loop = None
        self.last_time = time.process_time()
        self._init_pipeline()
        vs = threading.Thread(target=self.run, daemon=True)
        vs.start()

    def _init_pipeline(self):
        self.pipeline = Gst.Pipeline()

        streammux = Gst.ElementFactory.make("nvstreammux", "muxer")
        streammux.set_property('batch-size', 1)
        # streammux.set_property('batched-push-timeout', MUXER_BATCH_TIMEOUT_USEC)
        
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
            tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_NvDCF_perf.yml')
            # tracker.set_property('ll-config-file', '/opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app/config_tracker_NvDCF_accuracy.yml')
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
            # tracker.link(fakesink)
        else:
            streammux.link(queue_multifilesink_valve)
            # streammux.link(fakesink)
        queue_multifilesink_valve.link(multifilesink_valve)
        multifilesink_valve.link(osd)
        osd.link(nvvidconv_jpeg)
        # multifilesink_valve.link(nvvidconv_jpeg)
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
                current_time = time.process_time()
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
        The camera outputs on 3 devices: /dev/video0, /dev/video2, /dev/video4.\n
        Each device is a different format, but the order can change.\n
        We want specifically the YUYV (color) format\n
        While this finds the device with YUYV format, it does not guarantee that the correct resolution and framerate are available.
        Returns:
            str: The /dev/video* device path
        """
        if (re.search("YUYV", subprocess.run(['v4l2-ctl', '--list-devices', '--device', '/dev/video0'], capture_output=True, text=True).stdout) is not None):
            return "/dev/video0"
        elif (re.search("YUYV", subprocess.run(['v4l2-ctl', '--list-devices', '--device', '/dev/video2'], capture_output=True, text=True).stdout) is not None):
            return "/dev/video2"
        elif (re.search("YUYV", subprocess.run(['v4l2-ctl', '--list-devices', '--device', '/dev/video4'], capture_output=True, text=True).stdout) is not None):
            return "/dev/video4"
        else:
            self.get_logger().error("Could not find camera device with YUYV format")
            sys.exit(1)

def main():
    rclpy.init()
    buoy_detection_node = BuoyDetectionNode()
    rclpy.spin(buoy_detection_node)

if __name__ == '__main__':
    sys.exit(main())
