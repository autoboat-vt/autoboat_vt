import pyrealsense2 as rs
import numpy as np
from scipy.spatial.transform import Rotation as R

# -----------------------------
# PARAMETERS
# -----------------------------
ACCEL_MOTION_THRESH = 0.15   # m/s²
GYRO_MOTION_THRESH  = 0.03   # rad/s
CALIB_SAMPLES = 600

# -----------------------------
# START REALSENSE
# -----------------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)
pipeline.start(config)

# -----------------------------
# CALIBRATION
# -----------------------------
def calibrate_imu():
    print("Calibrating... Keep device perfectly still.")
    acc_samples = []
    gyro_samples = []

    for _ in range(CALIB_SAMPLES):
        frames = pipeline.wait_for_frames()
        accel = frames.first_or_default(rs.stream.accel)
        gyro  = frames.first_or_default(rs.stream.gyro)

        acc_data = accel.as_motion_frame().get_motion_data()
        gyro_data = gyro.as_motion_frame().get_motion_data()

        acc_samples.append([acc_data.x, acc_data.y, acc_data.z])
        gyro_samples.append([gyro_data.x, gyro_data.y, gyro_data.z])

    acc_bias = np.mean(acc_samples, axis=0)
    gyro_bias = np.mean(gyro_samples, axis=0)

    # Gravity vector = measured accel average
    gravity_world = acc_bias.copy()

    print("Calibration complete.")
    return acc_bias, gyro_bias, gravity_world

bias_acc, bias_gyro, gravity = calibrate_imu()

# -----------------------------
# STATE
# -----------------------------
# Use gravity direction from calibration
gravity_dir = gravity / np.linalg.norm(gravity)

q = R.identity()
v = np.zeros(3)
p = np.zeros(3)

last_ts = None

while True:
    frames = pipeline.wait_for_frames()

    accel_frame = frames.first_or_default(rs.stream.accel)
    gyro_frame  = frames.first_or_default(rs.stream.gyro)

    ts = accel_frame.get_timestamp() / 1000.0
    if last_ts is None:
        last_ts = ts
        continue

    dt = ts - last_ts
    last_ts = ts

    acc_data = accel_frame.as_motion_frame().get_motion_data()
    gyro_data = gyro_frame.as_motion_frame().get_motion_data()

    accel_raw = np.array([acc_data.x, acc_data.y, acc_data.z])
    gyro = np.array([gyro_data.x, gyro_data.y, gyro_data.z]) - bias_gyro

    # --- Orientation from gyro only (small correction)
    q = q * R.from_rotvec(gyro * dt)

    # --- Remove gravity using calibrated direction
    accel_corrected = accel_raw - gravity

    # Rotate into world
    accel_world = q.apply(accel_corrected)

    # Ignore vertical axis (desk testing only)
    accel_world[2] = 0

    # Motion detection
    if np.linalg.norm(accel_world[:2]) > 0.1:
        v[:2] += accel_world[:2] * dt
        p[:2] += v[:2] * dt
    else:
        v[:] = 0

    print(f"\rPosition (m): {p}", end="")