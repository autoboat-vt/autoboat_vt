import numpy as np
from math import tan, pi
from deepstream_buoy_detection_node import ObjectDetection, ObjectTrack, ObjectTriangulator

# ------------------------------------------------------------
# 1. PASTE YOUR CLASS DEFINITIONS HERE (With the fixes above!)
# ------------------------------------------------------------
# Copy ObjectDetection, ObjectTrack, ObjectTriangulator here.
# Ensure ObjectDetection.__init__ uses .reshape(3,1) on the vectors.

# ------------------------------------------------------------
# 2. HELPER: The "Ground Truth" Engine
# ------------------------------------------------------------
def get_camera_pose(x, y, heading):
    # Simplified pose matrix builder for the test
    # Heading 0 = North (Y+), 90 = East (X+)
    psi = np.radians(heading)
    c, s = np.cos(psi), np.sin(psi)
    
    # World Frame: X=East, Y=North, Z=Up
    # Cam Frame:   Z=Forward, X=Right, Y=Down
    R = np.array([
        [ c,  0,  s], # Cam X in World
        [-s,  0,  c], # Cam Y in World (Wait, Cam Y is DOWN)
        [ 0, -1,  0]  # Cam Z in World
    ])
    # Note: Your code had Cam Y as [0, -1, 0] which is correct for "Down"
    
    t = np.array([x, y, 0])
    
    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = t
    return pose

def project_point_to_pixel(K, pose, world_point):
    # Inverse of what ObjectDetection does
    R = pose[:3, :3]
    t = pose[:3, 3]
    
    # World to Cam: P_cam = R.T @ (P_world - t)
    p_cam = R.T @ (world_point - t)
    
    # Avoid division by zero if behind camera
    if p_cam[2] <= 0: return None
    
    # Project: u = fx * (x/z) + cx
    u = (p_cam[0] * K[0,0] / p_cam[2]) + K[0,2]
    v = (p_cam[1] * K[1,1] / p_cam[2]) + K[1,2]
    return u, v

# ------------------------------------------------------------
# 3. THE TEST RUNNER
# ------------------------------------------------------------
def run_test():
    # Setup Camera (Same as your node)
    width, height = 1280, 800
    hfov = 90
    focal_px = (width * 0.5) / tan(hfov * 0.5 * pi / 180)
    K = np.array([[focal_px, 0, width/2], [0, focal_px, height/2], [0, 0, 1]])
    
    triangulator = ObjectTriangulator(K, (width, height))
    
    # REALITY: Buoy is at (10, 20)
    buoy_truth = np.array([10, 20, 0])
    
    # SIMULATION: Boat drives past the buoy
    # Path: Starts at (0,0), drives North-East
    path = [
        {'x': 0,  'y': 0,  'head': 0},   # Looking North
        {'x': 5,  'y': 5,  'head': 10},  # Turning slightly
        {'x': 10, 'y': 10, 'head': 45},  # Looking NE
        {'x': 15, 'y': 10, 'head': 90},  # Looking East
    ]
    path_strafe = [
        {'x': 0,  'y': 0, 'head': 0}, # Buoy is 20m fwd, 10m right
        {'x': 5,  'y': 0, 'head': 0}, # Buoy is 20m fwd, 5m right
        {'x': 10, 'y': 0, 'head': 0}, # Buoy is 20m fwd, directly ahead
    ]
    path_orbit = [
        {'x': 20, 'y': 0,  'head': 0},   # South of buoy, facing North
        {'x': 0,  'y': 20, 'head': 90},  # West of buoy, facing East
        {'x': 20, 'y': 40, 'head': 180}, # North of buoy, facing South
        {'x': 40, 'y': 20, 'head': 270}, # East of buoy, facing West
    ]
    path_collision = [
        {'x': 0, 'y': 0,  'head': 0}, # Directly ahead
        {'x': 0, 'y': 10, 'head': 0}, # Still directly ahead
        {'x': 0, 'y': 20, 'head': 0}, # Still directly ahead
    ]
    
    print(f"--- Starting Simulation. Target Buoy at {buoy_truth} ---")
    
    for i, step in enumerate(path_collision):
        # 1. Get Boat Pose
        pose = get_camera_pose(step['x'], step['y'], step['head'])
        
        # 2. Generate "Fake" Detection (Perfect Projection)
        uv = project_point_to_pixel(K, pose, buoy_truth)
        
        if uv is None:
            print(f"Step {i}: Buoy not visible (behind camera)")
            continue
            
        u, v = uv
        print(f"Step {i}: Boat at ({step['x']},{step['y']}) see buoy at pixels ({u:.1f}, {v:.1f})")
        
        # 3. Feed to Triangulator
        # We assume Object ID 1 is always our buoy
        det = ObjectDetection(
            frame_number=i,
            x_position=u, y_position=v,
            width=50, height=50, # Dummy size
            object_id=1,
            pose_matrix=pose,
            camera_matrix=K,
            camera_matrix_inv=np.linalg.inv(K)
        )
        triangulator.add_observation(det)
        
        # 4. Attempt Solution
        estimated_pos = triangulator.triangulate(1)
        if estimated_pos is not None:
            err = np.linalg.norm(estimated_pos - buoy_truth)
            print(f"    -> Current Estimate: {estimated_pos}")
            print(f"    -> Error: {err:.4f} meters")
        else:
            print("    -> Not enough data to triangulate yet.")

if __name__ == "__main__":
    run_test()