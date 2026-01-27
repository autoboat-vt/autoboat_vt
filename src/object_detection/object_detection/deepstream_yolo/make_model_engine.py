"""
Script to generate TensorRT engine file from ONNX model using DeepStream.
This script creates a minimal GStreamer pipeline that triggers engine generation
without requiring a physical camera.

This was created with AI
"""

import os
import sys
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst
import re

if len(sys.argv) < 2:
    print("Usage: python make_model_engine.py <yolo_version>")
    sys.exit(1)

if (re.search("/home/ws", os.getcwd()) is not None):
    IS_DEV_CONTAINER = True
else:
    IS_DEV_CONTAINER = False

# Determine paths based on environment
if (IS_DEV_CONTAINER):
    PATH_TO_SRC_DIR = "/home/ws/src"
else:
    PATH_TO_SRC_DIR = "/home/sailbot/autoboat_vt/src"

PATH_TO_YOLO_CONFIG = f"{PATH_TO_SRC_DIR}/object_detection/object_detection/deepstream_yolo/config_infer_primary_yolo{sys.argv[1]}.txt"

# Configuration
COMPUTE_HW = 1
MEMORY_TYPE = 0
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
FRAMERATE = "30/1"

class EngineGenerator:
    def __init__(self):
        Gst.init(None)
        self.pipeline = None
        self.loop = None
        self.engine_created = False
        self.start_time = time.time()
        self.max_wait_time = 1800  # 30 minutes max wait for engine creation
        
    def bus_call(self, bus, message, loop):
        """Handle GStreamer bus messages"""
        t = message.type
        
        if t == Gst.MessageType.EOS:
            print("End-of-stream")
            loop.quit()
            
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}: {debug}")
            loop.quit()
            
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"Warning: {err}: {debug}")
            
        elif t == Gst.MessageType.INFO:
            err, debug = message.parse_info()
            info_str = str(err)
            print(f"Info: {info_str}")
            
            # Check if engine was created
            if "Deserialize engine successful" in info_str or "Serialize engine" in info_str:
                print("\n✓ Engine file created successfully!")
                self.engine_created = True
                # Give it a moment to finish writing
                GLib.timeout_add_seconds(2, self.stop_pipeline)
                
        return True
    
    def stop_pipeline(self):
        """Stop the pipeline gracefully"""
        print("Stopping pipeline...")
        if self.pipeline:
            self.pipeline.send_event(Gst.Event.new_eos())
        return False
    
    def probe_callback(self, pad, info, user_data):
        """Monitor the pipeline - after a few frames, we know the engine should be ready"""
        elapsed = time.time() - self.start_time
        
        # If we've been running for more than 5 seconds and haven't seen confirmation,
        # the engine is probably created (older DeepStream versions don't always log it)
        if elapsed > 5 and not self.engine_created:
            print("\n✓ Pipeline has been processing frames - engine should be ready")
            print(f"   (Ran for {elapsed:.1f} seconds)")
            self.engine_created = True
            GLib.timeout_add_seconds(1, self.stop_pipeline)
            
        return Gst.PadProbeReturn.OK
    
    def timeout_check(self):
        """Periodic check for timeout"""
        elapsed = time.time() - self.start_time
        
        if elapsed > self.max_wait_time:
            print(f"\n✗ Timeout after {self.max_wait_time} seconds")
            self.loop.quit()
            return False
            
        return True
    
    def create_pipeline(self):
        """Create a minimal pipeline to trigger engine generation"""
        print("Creating minimal pipeline for engine generation...")
        print(f"Using config: {PATH_TO_YOLO_CONFIG}")
        
        self.pipeline = Gst.Pipeline()
        
        # Fake video source - no camera needed!
        videotestsrc = Gst.ElementFactory.make("videotestsrc", "test-source")
        videotestsrc.set_property('pattern', 0)  # 0 = SMPTE color bars
        videotestsrc.set_property('num-buffers', 100)  # Generate 100 frames then EOS
        
        # Caps for test source
        caps_videotestsrc = Gst.ElementFactory.make('capsfilter', 'test-caps')
        caps_videotestsrc.set_property('caps', 
            Gst.Caps.from_string(f'video/x-raw, width={INPUT_WIDTH}, height={INPUT_HEIGHT}, framerate={FRAMERATE}'))
        
        # Convert to NVMM memory
        nvvidconv = Gst.ElementFactory.make('nvvideoconvert', 'nvconverter')
        nvvidconv.set_property('nvbuf-memory-type', MEMORY_TYPE)
        nvvidconv.set_property('compute-hw', COMPUTE_HW)
        
        caps_nvvidconv = Gst.ElementFactory.make('capsfilter', 'nvmm-caps')
        caps_nvvidconv.set_property('caps', 
            Gst.Caps.from_string(f'video/x-raw(memory:NVMM), format=NV12, width={INPUT_WIDTH}, height={INPUT_HEIGHT}'))
        
        # Stream muxer
        streammux = Gst.ElementFactory.make("nvstreammux", "muxer")
        streammux.set_property('batch-size', 1)
        
        # Primary inference - this is what creates the engine!
        pgie = Gst.ElementFactory.make('nvinfer', 'pgie')
        pgie.set_property('config-file-path', PATH_TO_YOLO_CONFIG)
        
        # Fake sink - just discard the output
        fakesink = Gst.ElementFactory.make('fakesink', 'fake-sink')
        
        # Add all elements to pipeline
        self.pipeline.add(videotestsrc)
        self.pipeline.add(caps_videotestsrc)
        self.pipeline.add(nvvidconv)
        self.pipeline.add(caps_nvvidconv)
        self.pipeline.add(streammux)
        self.pipeline.add(pgie)
        self.pipeline.add(fakesink)
        
        # Link elements
        videotestsrc.link(caps_videotestsrc)
        caps_videotestsrc.link(nvvidconv)
        nvvidconv.link(caps_nvvidconv)
        
        # Link to muxer
        sinkpad = streammux.request_pad_simple('sink_0')
        srcpad = caps_nvvidconv.get_static_pad('src')
        if not srcpad.link(sinkpad) == Gst.PadLinkReturn.OK:
            print("Failed to link converter to streammux")
            return False
        
        # Link rest of pipeline
        streammux.link(pgie)
        pgie.link(fakesink)
        
        # Add probe to monitor processing
        pgie_sink_pad = pgie.get_static_pad("sink")
        pgie_sink_pad.add_probe(Gst.PadProbeType.BUFFER, self.probe_callback, None)
        
        return True
    
    def run(self):
        """Run the pipeline"""
        if not self.create_pipeline():
            print("Failed to create pipeline")
            return False
        
        # Create event loop
        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.bus_call, self.loop)
        
        # Add timeout checker
        GLib.timeout_add_seconds(5, self.timeout_check)
        
        print("\nStarting pipeline to generate engine file...")
        print("This may take several minutes on first run...")
        print("=" * 60)
        
        # Start pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("Unable to set the pipeline to PLAYING state")
            return False
        
        try:
            # Run event loop
            self.loop.run()
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        # Cleanup
        print("\nCleaning up...")
        self.pipeline.set_state(Gst.State.NULL)
        
        return self.engine_created


def main():
    print("=" * 60)
    print("DeepStream YOLO Engine Generator")
    print("=" * 60)
    print()
    
    # Check if config exists
    if not os.path.exists(PATH_TO_YOLO_CONFIG):
        print(f"✗ Error: Config file not found: {PATH_TO_YOLO_CONFIG}")
        return 1
    
    print(f"✓ Config file found: {PATH_TO_YOLO_CONFIG}")
    
    # Parse config to find the ONNX and engine file names
    onnx_file = None
    engine_file = None
    config_dir = os.path.dirname(PATH_TO_YOLO_CONFIG)
    
    with open(PATH_TO_YOLO_CONFIG, 'r') as f:
        for line in f:
            if line.strip().startswith('onnx-file=') and not line.strip().startswith('#'):
                onnx_file = line.split('=')[1].strip()
            if line.strip().startswith('model-engine-file=') and not line.strip().startswith('#'):
                engine_file = line.split('=')[1].strip()
    
    if onnx_file:
        onnx_path = os.path.join(config_dir, onnx_file)
        if os.path.exists(onnx_path):
            print(f"✓ ONNX file found: {onnx_file}")
        else:
            print(f"✗ Error: ONNX file not found: {onnx_path}")
            return 1
    else:
        print("✗ Error: No onnx-file specified in config")
        return 1
    
    if engine_file:
        target_engine_path = os.path.join(config_dir, engine_file)
        # Default engine filename that DeepStream creates
        default_engine_file = "model_b1_gpu0_fp16.engine"
        default_engine_path = os.path.join(config_dir, default_engine_file)
        
        if os.path.exists(target_engine_path):
            print(f"⚠ Warning: Target engine file already exists: {engine_file}")
            response = input("Regenerate? (y/n): ")
            if response.lower() != 'y':
                print("Exiting.")
                return 0
            # Delete existing engine
            os.remove(target_engine_path)
            print(f"✓ Deleted existing engine file")
        
        # Also check if default engine exists and clean it up
        if os.path.exists(default_engine_path):
            os.remove(default_engine_path)
            print(f"✓ Cleaned up existing default engine file")
            
        print(f"Target engine file: {engine_file}")
        print(f"Will rename from default: {default_engine_file}")
    else:
        print("✗ Error: No model-engine-file specified in config")
        return 1
    
    print()
    
    # Set environment variables
    os.environ["USE_NEW_NVSTREAMMUX"] = "yes"
    os.environ['CUDA_VER'] = "12.6"
    
    # Run the generator
    generator = EngineGenerator()
    success = generator.run()
    
    print()
    print("=" * 60)
    
    if success:
        # Rename the default engine file to the target name
        if os.path.exists(default_engine_path):
            os.rename(default_engine_path, target_engine_path)
            print(f"✓ Renamed {default_engine_file} -> {engine_file}")
        
        # Verify engine was created
        if os.path.exists(target_engine_path):
            file_size = os.path.getsize(target_engine_path) / (1024 * 1024)  # MB
            print(f"✓ SUCCESS! Engine file created: {engine_file}")
            print(f"  Size: {file_size:.1f} MB")
            print(f"  Location: {target_engine_path}")
        else:
            print(f"⚠ Pipeline ran but engine file not found at: {target_engine_path}")
            print("  Check the logs above for errors")
            return 1
    else:
        print("✗ Failed to create engine file")
        print("  Check the logs above for errors")
        return 1
    
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
