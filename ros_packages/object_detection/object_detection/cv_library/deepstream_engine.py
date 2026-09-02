# ruff: noqa:E402
import os

import gi

gi.require_version('Gst', '1.0')
import re
import shutil
import subprocess
import time
from collections.abc import Callable
from math import pi
from threading import Lock

import numpy as np
import pyds
import yaml
from gi.repository import GLib, Gst

os.environ["USE_NEW_NVSTREAMMUX"] = "yes"
# os.environ['GST_DEBUG'] = "3"

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None

SHOULD_DISPLAY = True
# NUM_IMAGES_TO_SAVE = 10000

# These are constants. Don't change these. Needed for a workaround with DeepStream 7.1 and JetPack 6.2
COMPUTE_HW = 1
MEMORY_TYPE = 0
EARTH_RADIUS = 6378137.0  # Radius of Earth in meters

PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"

YOLO_CONFIG = {
    11: f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/yolo11_config.yaml",
    26: f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/yolo26_config.yaml"
}
CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"
INFERENCE = True
if "INFERENCE" in os.environ and os.environ["INFERENCE"] == "false":
    INFERENCE = False

CAMERA = True
if "CAMERA" in os.environ and os.environ["CAMERA"] == "false":
    CAMERA = False

class DeepStreamEngine:
    """
    Handles the Object Detection using DeepStream and YOLO models.

    Credit:<br>
    marcoslucianops (https://github.com/marcoslucianops/DeepStream-Yolo)<br>
    DeepStream SDK Documentation (https://docs.nvidia.com/metropolis/deepstream/7.1/text/DS_Overview.html)<br>
    DeepStream Python Examples (https://github.com/NVIDIA-AI-IOT/deepstream_python_apps)<br>
    Ultralytics https://www.ultralytics.com/
    """
    
    def __init__(
      self, detection_callback:Callable[[dict], None],
      info_callback:Callable[[str], None],
      warn_callback:Callable[[str], None],
      error_callback:Callable[[str], None]
    ) -> None:
        self.parameters = {
            "model_name": "", # model name without .onnx. Ex. yolo11m.onnx -> yolo11m
            "threshold": "" # detection threshold
        }
        self.detection_callback = detection_callback
        self.info_callback = info_callback
        self.warn_callback = warn_callback
        self.error_callback = error_callback

        self.cam_list = self._read_camera_config()
        if CAMERA:
            self.cam_list[0]["device"] = self._find_camera()

        self.camera_focal_px = self.cam_list[0]["focal_px"]

        # DeepStream Initialization
        Gst.init(None)
        self.pipeline = None
        self.loop = None

        # Config file parameters
        self.config_file_split = []
        self.yolo_ver = 26
        if "YOLO_VER" in os.environ and os.environ["YOLO_VER"] in ["11", "26"]:
            self.yolo_ver = int(os.environ["YOLO_VER"])
        self.last_time = time.time() # used to calculate fps
        self.file_lock = Lock()
        self.last_published_frame_number = -1
        with self.file_lock:
            file_results = self._read_file(YOLO_CONFIG[self.yolo_ver])
            self.config_file_split = file_results[0]
            self.parameters["model_name"] = file_results[1]
            self.parameters["threshold"] = file_results[2]

        self._init_pipeline()
    
    def _init_pipeline(self) -> None:
        self.pipeline = Gst.Pipeline()

        streammux = Gst.ElementFactory.make("nvstreammux", "muxer")
        streammux.set_property('batch-size', 2)
        
        # v4l2-ctl --list-devices
        # v4l2-ctl --device /dev/video0 --list-formats-ext
        if CAMERA:
            source0 = Gst.ElementFactory.make("v4l2src", "usb-cam-0")
            source0.set_property('device', self.cam_list[0]["device"])
            self.info_callback(f"Opening camera device {self.cam_list[0]['name']} on {self.cam_list[0]['device']}")
        else:
            source0 = Gst.ElementFactory.make("videotestsrc", "usb-cam-0")
            self.info_callback("Opening videotestsrc")

        """
        v4l2 camera settings
        brightness: [-64, 64]
        hue: [-180, 180]
        contrast: [0, 100]
        saturation: [0, 100]
        """

        caps_source0 = Gst.ElementFactory.make('capsfilter', 'source0-caps')
        caps_source0.set_property('caps', Gst.Caps.from_string(f'video/x-raw,'
                                                               f'width={self.cam_list[0]["width"]},'
                                                               f'height={self.cam_list[0]["height"]},'
                                                               f'format={self.cam_list[0]["gst_format"]},'
                                                               f'framerate={self.cam_list[0]["framerate"]}'
                                                               ))

        # This is a workaround.
        # Issue with deepstream7.1 and jetpack6.2 requires compute-hw to be 1 instead of 0.
        # When compute-hw is 1, nvvidconv fails to convert from YUY2 to NV12 directly.
        # So we convert from YUY2 to RGB first, then from RGB to NV12
        videoconvert0 = Gst.ElementFactory.make('videoconvert', 'convertor-0')

        caps_videoconvert0 = Gst.ElementFactory.make('capsfilter', 'convertor-caps-0')
        caps_videoconvert0.set_property('caps', Gst.Caps.from_string('video/x-raw, format=RGB'))

        nvvidconvsrc0 = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter-src-0')
        nvvidconvsrc0.set_property('nvbuf-memory-type', MEMORY_TYPE)
        nvvidconvsrc0.set_property('compute-hw', COMPUTE_HW)

        caps_nvvidconvsrc0 = Gst.ElementFactory.make("capsfilter", "nvmm-caps-0")
        caps_nvvidconvsrc0.set_property(
            "caps",
            Gst.Caps.from_string(
                f"video/x-raw(memory:NVMM), format=NV12,"
                f"width={self.cam_list[0]['width']},"
                f"height={self.cam_list[0]['height']}"
            )
        )

        src_tee = Gst.ElementFactory.make("tee", "src-tee")

        nvvidconv_left = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter-left')
        nvvidconv_left.set_property('nvbuf-memory-type', MEMORY_TYPE)
        nvvidconv_left.set_property('compute-hw', COMPUTE_HW)
        nvvidconv_left.set_property('src-crop', f"0:0:{self.cam_list[0]['width'] // 2}:{self.cam_list[0]['height']}")

        caps_nvvidconv_left = Gst.ElementFactory.make("capsfilter", "nvmm-caps-left")
        caps_nvvidconv_left.set_property(
            "caps",
            Gst.Caps.from_string(
                f"video/x-raw(memory:NVMM), format=NV12,"
                f"width={self.cam_list[0]['width'] // 2},"
                f"height={self.cam_list[0]['height']}"
            )
        )

        nvvidconv_right = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter-right')
        nvvidconv_right.set_property('nvbuf-memory-type', MEMORY_TYPE)
        nvvidconv_right.set_property('compute-hw', COMPUTE_HW)
        nvvidconv_right.set_property('src-crop', f"{self.cam_list[0]['width'] // 2}:0:"
                                                 f"{self.cam_list[0]['width']}:{self.cam_list[0]['height']}")
        caps_nvvidconv_right = Gst.ElementFactory.make("capsfilter", "nvmm-caps-right")
        caps_nvvidconv_right.set_property(
            "caps",
            Gst.Caps.from_string(
                f"video/x-raw(memory:NVMM), format=NV12,"
                f"width={self.cam_list[0]['width'] // 2},"
                f"height={self.cam_list[0]['height']}"
            )
        )

        if INFERENCE:
            pgie = Gst.ElementFactory.make('nvinfer', 'pgie')
            pgie.set_property('config-file-path', YOLO_CONFIG[self.yolo_ver])
            self.info_callback(f"Running Inference with Yolo{self.yolo_ver}")

            tracker = Gst.ElementFactory.make("nvtracker", "tracker")
            # docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_gst-nvtracker.html#nvidia-tao-reidentificationnet
            tracker.set_property("ll-lib-file", "/opt/nvidia/deepstream/deepstream-7.1/lib/libnvds_nvmultiobjecttracker.so")
            tracker.set_property('ll-config-file',
                                 f'{PATH_TO_PKG_DIR}/object_detection/object_detection/config/config_tracker_NvDCF_perf.yml')
            tracker.set_property("tracking-id-reset-mode", 0)

        queue_display_valve = Gst.ElementFactory.make("queue", "queue-valve")

        display_valve = Gst.ElementFactory.make('valve', 'display-valve')
        display_valve.set_property('drop', not SHOULD_DISPLAY)

        tiler = Gst.ElementFactory.make('nvmultistreamtiler', 'nvtiler')
        tiler.set_property('width', 1792)
        tiler.set_property('height', 504)
        # tiler.set_property('show-source', 0)

        osd = Gst.ElementFactory.make("nvdsosd", "nvosd")

        sink = Gst.ElementFactory.make('nveglglessink', 'sink')
        sink.set_property('sync', False)

        self.pipeline.add(source0)
        self.pipeline.add(caps_source0)
        self.pipeline.add(videoconvert0)
        self.pipeline.add(caps_videoconvert0)
        self.pipeline.add(nvvidconvsrc0)
        self.pipeline.add(caps_nvvidconvsrc0)
        self.pipeline.add(src_tee)
        self.pipeline.add(nvvidconv_left)
        self.pipeline.add(caps_nvvidconv_left)
        self.pipeline.add(nvvidconv_right)
        self.pipeline.add(caps_nvvidconv_right)
        self.pipeline.add(streammux)
        if INFERENCE:
            self.pipeline.add(pgie)
            self.pipeline.add(tracker)
        self.pipeline.add(queue_display_valve)
        self.pipeline.add(display_valve)
        self.pipeline.add(tiler)
        self.pipeline.add(osd)
        self.pipeline.add(sink)

        source0.link(caps_source0)
        caps_source0.link(videoconvert0)
        videoconvert0.link(caps_videoconvert0)
        caps_videoconvert0.link(nvvidconvsrc0)
        nvvidconvsrc0.link(caps_nvvidconvsrc0)
        caps_nvvidconvsrc0.link(src_tee)

        srcpad_left = src_tee.request_pad_simple('src_0')
        sinkpad_left = nvvidconv_left.get_static_pad('sink')
        srcpad_left.link(sinkpad_left)
        nvvidconv_left.link(caps_nvvidconv_left)

        srcpad_right = src_tee.request_pad_simple('src_1')
        sinkpad_right = nvvidconv_right.get_static_pad('sink')
        srcpad_right.link(sinkpad_right)
        nvvidconv_right.link(caps_nvvidconv_right)

        sinkpad0 = streammux.request_pad_simple('sink_0')
        srcpad0 = caps_nvvidconv_left.get_static_pad('src')
        srcpad0.link(sinkpad0)

        sinkpad1 = streammux.request_pad_simple('sink_1')
        srcpad1 = caps_nvvidconv_right.get_static_pad('src')
        srcpad1.link(sinkpad1)

        if INFERENCE:
            streammux.link(pgie)
            pgie.link(tracker)
            tracker.link(queue_display_valve)
        else:
            streammux.link(queue_display_valve)
        queue_display_valve.link(display_valve)
        display_valve.link(tiler)
        tiler.link(osd)
        osd.link(sink)

        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._bus_call, self.loop)

        infer_probe_pad = queue_display_valve.get_static_pad('sink')
        infer_probe_pad.add_probe(Gst.PadProbeType.BUFFER, self._infer_probe, 0)

        osd_probe_pad = osd.get_static_pad('sink')
        osd_probe_pad.add_probe(Gst.PadProbeType.BUFFER, self._osd_probe, 0)
    
    def _bus_call(self, bus: Gst.Bus, message: Gst.Message, loop: GLib.MainLoop) -> bool: # noqa: ARG002
        t = message.type
        if t == Gst.MessageType.EOS:
            self.info_callback("End-of-stream\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            self.warn_callback(f"Warning: {err}: {debug}\n")
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.error_callback(f"Error: {err}: {debug}\n")
            self.close_pipeline()
        return True

    def run(self) -> None:
        """Turn on the pipeline and start the main loop. This will block until the pipeline is closed."""
        self.info_callback("Starting pipeline\n")
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop.run()
        except Exception as e:
            self.error_callback(f"Exception in main loop: {e}\n")
        finally:
            self.pipeline.set_state(Gst.State.NULL)
            self.info_callback("Pipeline stopped\n")

    def close_pipeline(self) -> None:
        """Safely closes the GStreamer pipeline and stops the main loop."""
        self.info_callback("Quitting pipeline\n")
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()
        self.info_callback("Pipeline stopped\n")
    
    def _infer_probe(self, pad: Gst.Pad, info: Gst.PadProbeInfo, u_data: any) -> Gst.PadProbeReturn: # noqa: ARG002
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            self.info_callback("Unable to get GstBuffer")
            return None

        msg = {}
        msg["detection_results"] = []
        if (self.parameters["model_name"] is not None):
            msg["model_name"] = self.parameters["model_name"]
        else:
            msg["model_name"] = "DISABLED"
        msg["yolo_version"] = self.yolo_ver
        msg["threshold"] = self.parameters["threshold"]

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

            msg["ntp_timestamp"] = frame_meta.ntp_timestamp
            msg["detection_results"].append([])

            if (frame_meta.source_id == 0 and frame_meta.frame_num % 60 == 0):
                current_time = time.time()
                fps = 60 / (current_time - self.last_time)
                self.last_time = current_time
                self.info_callback(f"Frame: {frame_meta.frame_num}, avg FPS: {fps:.2f}")
            l_obj = frame_meta.obj_meta_list

            # Iterate through each object in frame
            while l_obj is not None:
                try:
                    # Casting l_obj.data to pyds.NvDsObjectMeta
                    obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
                except StopIteration:
                    break

                obj_results = {}
                obj_results["detector_confidence"] = obj_meta.confidence
                obj_results["tracker_confidence"] = obj_meta.tracker_confidence
                mid_x = obj_meta.tracker_bbox_info.org_bbox_coords.left + (obj_meta.tracker_bbox_info.org_bbox_coords.width / 2)
                mid_y = obj_meta.tracker_bbox_info.org_bbox_coords.top + (obj_meta.tracker_bbox_info.org_bbox_coords.height / 2)
                obj_results["x_position"] = mid_x
                obj_results["y_position"] = mid_y
                obj_results["width"] = obj_meta.tracker_bbox_info.org_bbox_coords.width
                obj_results["height"] = obj_meta.tracker_bbox_info.org_bbox_coords.height
                obj_results["object_id"] = obj_meta.object_id
                obj_results["class_id"] = obj_meta.class_id
                obj_results["obj_label"] = obj_meta.obj_label
                obj_results["angle_to_object"] = np.arctan((mid_x - self.cam_list[0]["width"] / 4)
                                                           / self.cam_list[0]["focal_px"]) * 180 / pi
                msg["detection_results"][frame_meta.source_id].append(obj_results)

                obj_meta.text_params.display_text = f"{obj_meta.obj_label} {obj_meta.object_id} {obj_meta.confidence:.2f}."

                try:
                    l_obj = l_obj.next
                except StopIteration:
                    break

            try:
                l_frame = l_frame.next
            except StopIteration:
                break

        self.detection_callback(msg)

        return Gst.PadProbeReturn.OK

    def _osd_probe(self, pad: Gst.Pad, info: Gst.PadProbeInfo, user_data: int) -> Gst.PadProbeReturn: # noqa: ARG002
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            self.info_callback("Unable to get GstBuffer")
            return None
        
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))

        display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1
        py_nvosd_text_params = display_meta.text_params[0]
        py_nvosd_text_params.display_text = (f"Yolo Version: {self.yolo_ver}\n"
                                             f"Current Model: {self.parameters['model_name'] if INFERENCE else 'DISABLED'}\n"
                                             f"Threshold: {self.parameters['threshold']}")
   
        # Set the offsets where the string should appear
        py_nvosd_text_params.x_offset = 10
        py_nvosd_text_params.y_offset = 12

        # Font , font-color and font-size
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 10
        # set(red, green, blue, alpha); set to White
        py_nvosd_text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)

        # Text background color
        py_nvosd_text_params.set_bg_clr = 1
        # set(red, green, blue, alpha); set to Black
        py_nvosd_text_params.text_bg_clr.set(0.0, 0.0, 0.0, 0.25)

        return Gst.PadProbeReturn.OK

    def _read_camera_config(self) -> dict:
        with open(CAMERA_CONFIG, 'r') as file:
            return yaml.safe_load(file)

    def _find_camera(self, cam_id: int = 0) -> str:
        """
        This is just a way to figure out which /dev/video* is the camera<br>
        The camera outputs on 3 devices<br>
        Each device is a different format, but the order can change or extra cameras can cause the number to increase<br>
        While this finds the device with the specified format,
        it does not guarantee that the correct resolution and framerate are available.
        
        Returns
        -------
            str: The /dev/video* device path.
        """

        cam_format = self.cam_list[cam_id]["v4l2_format"]
        cam_name = self.cam_list[cam_id]["name"]
        ls = shutil.which("ls")
        cat = shutil.which("cat")
        v4l2_ctl = shutil.which("v4l2-ctl")
        if ls is None or cat is None or v4l2_ctl is None:
            self.error_callback("ls, cat, or v4l2-ctl command not found. Cannot find camera device.")
            raise OSError("Required command not found")
        try:
            camera_devices_output = subprocess.run([ls, '/sys/class/video4linux/'], # noqa: S603
                                                   capture_output=True, text=True, check=True).stdout
        except subprocess.CalledProcessError as err:
            self.error_callback("Failed to list camera devices in /sys/class/video4linux/.")
            raise OSError("Failed to list camera devices in /sys/class/video4linux/.") from err
        for device in camera_devices_output.splitlines():
            try:
                if ((re.search(cam_name, subprocess.run([cat, f'/sys/class/video4linux/{device}/name'], # noqa: S603
                                                        capture_output=True, text=True, check=True).stdout) is not None) and
                (re.search(cam_format, subprocess.run([v4l2_ctl, '--device', f'/dev/{device}', '--list-formats'], # noqa: S603
                                                            capture_output=True, text=True, check=True).stdout) is not None)):
                        return f"/dev/{device}"
            except subprocess.CalledProcessError as err:
                self.error_callback(f"Command v4l2-ctl failed for device {device}.")
                raise OSError(f"Command v4l2-ctl failed for device {device}.") from err
        self.error_callback(f"Could not find {cam_name} device with {cam_format} format")
        raise OSError("Camera device not found")

    def _read_file(self, file_name:str) -> tuple[list[str], str, float]:
        # Open file
        # Don't need the file lock becuase the caller has it.
        with open(file_name, 'r') as file:
            content = file.read()
            config_file_split = content.split('\n\n')

        model = None
        # Read model
        onnx_section = config_file_split[1]
        onnx_lines = onnx_section.split('\n')
        for line in onnx_lines:
            if line.strip().startswith('onnx-file: '):
                model = line.split(': ')[-1].split('/')[-1].split('.')[0] # isolate model name without path or extension
                break
        
        # Read threshold
        attributes_lines = config_file_split[5].split('\n')
        threshold = float(attributes_lines[1].split(': ')[-1])
        return (config_file_split, model, threshold)

    def update_model_or_threshold(self, new_model: str | None = None, new_threshold: float | None = None) -> bool:
        """Updates the model and/or detection threshold in the config file and reloads it in the pipeline."""
        with self.file_lock: # Don't want multiple threads writing to the file at once
            updated = False
            if new_model is not None:
                updated = self._update_model(new_model) or updated
            if new_threshold is not None:
                updated = self._update_threshold(new_threshold) or updated
            if updated:
                self._update_config_file(YOLO_CONFIG[self.yolo_ver])
        return updated

    def _update_threshold(self, new_threshold: float) -> bool:
        """Updates the detection threshold in the config file."""
        updated_value = False
        if new_threshold != self.parameters["threshold"]:
            if new_threshold >= 0.0 and new_threshold <= 1.0:
                attributes_lines = self.config_file_split[5].split('\n')
                attributes_lines[1] = f"    pre-cluster-threshold: {new_threshold}"
                self.config_file_split[5] = "\n".join(attributes_lines)
                self.parameters["threshold"] = new_threshold
                self.info_callback(f"Updated threshold to {new_threshold}")
                updated_value = True
            else:
                self.info_callback(f"Threshold {new_threshold} is out of range [0.0, 1.0], not updating")
        else:
            self.info_callback(f"Threshold is already {new_threshold}, not updating")
        return updated_value
    
    def _update_model(self, new_model: str) -> bool:
        """Updates the model in the config file."""
        updated_value = False
        # Should we check if the config file was externally updated?
        if new_model != self.parameters["model_name"] and INFERENCE:
            onnx_lines, engine_lines, labels_lines, found_model_entry = self._modify_config_lines(self.config_file_split,
                                                                                                  new_model)
            
            # Should we add the new model if not found?
            self.info_callback(f"Model entry found in current config: {found_model_entry}")
            if not found_model_entry:
                split_lines = self._read_file(YOLO_CONFIG[26 if self.yolo_ver == 11 else 11])[0]
                onnx_lines, engine_lines, labels_lines, found_model_entry = self._modify_config_lines(split_lines, new_model)
                if found_model_entry: # we found the model in the alternate config, so we can switch to that config
                    self.yolo_ver = 26 if self.yolo_ver == 11 else 11
                    attributes_lines = split_lines[5].split('\n')
                    attributes_lines[1] = f"    pre-cluster-threshold: {self.parameters['threshold']}"
                    split_lines[5] = "\n".join(attributes_lines)
                    self.config_file_split = split_lines
                    self.info_callback(f"Model entry found in alternate config, switching to Yolo{self.yolo_ver}")
                else:
                    self.info_callback(f"Model {new_model}.onnx not found, not updating model")
                    self.file_lock.release()
            
            if found_model_entry:
                onnx_content = "\n".join(onnx_lines)
                engine_content = "\n".join(engine_lines)
                labels_content = "\n".join(labels_lines)
                self.config_file_split[1] = onnx_content
                self.config_file_split[2] = engine_content
                self.config_file_split[3] = labels_content
                self.parameters["model_name"] = new_model
                self.info_callback(f"Updated model to {new_model}")
                updated_value = True
        else:
            self.info_callback(f"Model is already {new_model}, not updating")
        return updated_value
    
    def _modify_config_lines(self, lines_split: list, new_model: str) -> tuple[list, list, list, bool]:
        onnx_lines = lines_split[1].split('\n')
        engine_lines = lines_split[2].split('\n')
        labels_lines = lines_split[3].split('\n')
        properties = lines_split[4].split('\n')
        batch_size = int(properties[1].split(': ')[-1].split(' ')[0])
        network_mode = int(properties[2].split(': ')[-1].split(' ')[0])
        match network_mode:
            case 0:
                quantize = "fp32"
            case 1:
                quantize = "int8"
            case 2:
                quantize = "fp16"
            case _:
                self.get_logger().warn(f"Unknown network mode {network_mode}, defaulting to fp16")
                quantize = "fp16"

        found_model_entry = False
        for i in range(len(onnx_lines)):
            if onnx_lines[i].strip().startswith('onnx-file: '):
                onnx_lines[i] = "    # " + onnx_lines[i].strip()
            if onnx_lines[i].split(': ')[-1].split('/')[-1] == f"{new_model}.onnx":
                onnx_lines[i] = f"    {onnx_lines[i].strip()[2:]}" # uncomment line so the model can be used
                found_model_entry = True

        if found_model_entry:
            for i in range(len(engine_lines)):
                if engine_lines[i].strip().startswith('model-engine-file: '):
                    engine_lines[i] = "    # " + engine_lines[i].strip()
                if engine_lines[i].split(': ')[-1].split('/')[-1] == f"{new_model}_model_b{batch_size}_gpu0_{quantize}.engine":
                    engine_lines[i] = f"    {engine_lines[i].strip()[2:]}" # uncomment line so the model can be used

            for i in range(len(labels_lines)):
                if labels_lines[i].strip().startswith('labelfile-path: '):
                    labels_lines[i] = "    # " + labels_lines[i].strip()
                if labels_lines[i].split(': ')[-1].split('/')[-1] == f"{new_model}_labels.txt":
                    labels_lines[i] = f"    {labels_lines[i].strip()[2:]}" # uncomment line so the model can be used
        
        return (onnx_lines, engine_lines, labels_lines, found_model_entry)

    def _update_config_file(self, file_name: str) -> None:
        # This doesn't need the file_lock because the caller already has it
        with open(file_name, 'w') as file:
            file.write("\n\n".join(self.config_file_split))
        if INFERENCE: # Should this check be here? Is it necessary?
            self.pipeline.get_by_name('pgie').set_property('config-file-path', file_name)
            self.info_callback("Reloaded config file in nvinfer")
        else:
            self.info_callback("Not reloading config file in nvinfer since INFERENCE is disabled")
