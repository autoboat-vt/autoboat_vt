import subprocess
import re
import os
import sys
import cv2

def get_device(format):
    camera_devices_output = subprocess.run(['ls', '/sys/class/video4linux/'], capture_output=True, text=True).stdout
    for device in camera_devices_output.splitlines():
        if (re.search("RealSense", subprocess.run(['cat', f'/sys/class/video4linux/{device}/name'], capture_output=True, text=True).stdout) is not None):
            if (re.search(format, subprocess.run(['v4l2-ctl', '--device', f'/dev/{device}', '--list-formats'], capture_output=True, text=True).stdout) is not None):
                return f"/dev/{device}"
    print(f"Could not find RealSense camera device with {format} format")
    sys.exit(1)

def main():
    format = "YUYV"
    device = get_device(format)
    width = 1280
    height = 800
    fps = 15
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Could not open video device {device}")
        sys.exit(1)

    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*format))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"Camera opened with resolution {actual_width}x{actual_height} at {actual_fps:.1f} FPS")

    try:
        count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
            if count % 120 == 0:
                print("Current frame count:", count)
            cv2.imwrite(f'./frames/frame{count}.png', frame)
            count += 1
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
