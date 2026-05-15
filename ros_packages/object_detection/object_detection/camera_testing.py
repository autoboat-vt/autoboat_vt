import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# Create output directory
output_dir = "realsense_captures"
os.makedirs(output_dir, exist_ok=True)

# Create pipeline - let it auto-detect device
print("Creating RealSense pipeline...")
pipeline = rs.pipeline()

# Start with default configuration first to see if device works
print("Starting with default config...")
try:
    profile = pipeline.start()
    print("Success! Stopping to reconfigure...")
    pipeline.stop()
    import time
    time.sleep(1)
except Exception as e:
    print(f"Error with default config: {e}")
    print("Trying to set up udev rules...")
    print("Run: sudo cp /usr/local/etc/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/")
    print("Then: sudo udevadm control --reload-rules && sudo udevadm trigger")
    exit(1)

# Now configure specific streams
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  # Left
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)  # Right

# Start pipeline with config
print("Starting RealSense pipeline with full config...")
pipeline.start(config)
print("Pipeline started successfully!")

try:
    print("Press 's' to save frames, 'q' to quit")
    frame_count = 0
    
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        
        # Get all frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        ir_left = frames.get_infrared_frame(1)
        ir_right = frames.get_infrared_frame(2)
        
        if not color_frame or not depth_frame or not ir_left or not ir_right:
            continue
        
        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        left_image = np.asanyarray(ir_left.get_data())
        right_image = np.asanyarray(ir_right.get_data())
        
        # Display color image for preview
        cv2.imshow('Color Preview', color_image)
        cv2.imshow('Left IR', left_image)
        cv2.imshow('Right IR', right_image)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s'):
            # Save all frames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            
            cv2.imwrite(f"{output_dir}/color_{timestamp}.png", color_image)
            cv2.imwrite(f"{output_dir}/left_ir_{timestamp}.png", left_image)
            cv2.imwrite(f"{output_dir}/right_ir_{timestamp}.png", right_image)
            
            # Save depth as 16-bit PNG
            cv2.imwrite(f"{output_dir}/depth_{timestamp}.png", depth_image)
            
            # Also save colorized depth for visualization
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            cv2.imwrite(f"{output_dir}/depth_colorized_{timestamp}.png", depth_colormap)
            
            frame_count += 1
            print(f"Saved frame set {frame_count} at {timestamp}")
        
        elif key == ord('q'):
            break
            
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print(f"Total frames saved: {frame_count}")