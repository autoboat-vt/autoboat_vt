import numpy as np
from enum import Enum

import geopy
import geopy.distance
import pyproj
import math

from .position import Position



class SailboatAutopilotMode(Enum):
    Disabled = 0
    Full_RC = 1
    Hold_Best_Sail = 2
    Hold_Heading = 3
    Hold_Heading_And_Best_Sail = 4
    Waypoint_Mission = 5
    
class SailboatStates(Enum):
    NORMAL = 0
    CW_TACKING = 1
    CCW_TACKING = 2
    STALL = 3
    # JIBE = 4

class SailboatManeuvers(Enum):
    AUTOPILOT_DISABLED = 0
    STANDARD = 1
    TACK = 2
    JIBE = 3






class MotorboatAutopilotMode(Enum):
    Disabled = 0
    Full_RC = 1
    Hold_Heading = 2
    Waypoint_Mission = 3

class MotorboatControls(Enum):
    RPM = 0
    DUTY_CYCLE = 1
    CURRENT = 2
  


  
    
def check_float_equivalence(float1, float2):
    return abs(float1 - float2) <= 0.001


def cartesian_vector_to_polar(x, y):
    """
        Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).
        Output direction is in degrees from 0 to 360 degrees
        Outputs a tuple of magnitude and direction of the inputted vector
    """
    # arctan2 doesn't like when we pass 2 zeros into it so we should cover that case
    if x == 0. and y == 0.:
        return 0., 0.
    magnitude = np.sqrt(x**2 + y**2)
    direction = np.arctan2(y, x) # radians
    direction = direction * (180/np.pi)  # angle from -180 to 180 degrees
    direction = direction % 360  # angle from 0 to 360 degrees
    return magnitude, direction
    
    
def get_distance_between_angles(angle1: float, angle2: float):
    # https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point
    return -1 * ((float(angle1) - float(angle2) + 180) % 360 - 180)
    

def get_bearing(current_pos: Position, destination_pos: Position):
    """
    utility function to get the bearing towards a specific destination point, from our current location.
    This returns the bearing as an angle between 0 to 360, counter clockwise, measured from east
    """
    
    cur_lat, cur_lon = current_pos.get_lat_lon()
    des_lat, des_lon = destination_pos.get_lat_lon()
    azimuth_heading, _, _ = pyproj.Geod(ellps='WGS84').inv(cur_lon, cur_lat, des_lon, des_lat)
    return (-azimuth_heading + 90) % 360      # azimuth is cw from true north while we want ccw from true east



def get_distance_between_positions(position1: Position, position2: Position):
    return geopy.distance.geodesic(position1.get_lat_lon(), position2.get_lat_lon()).m



def does_line_intersect_circle(
        start_point: list[float, float], 
        end_point: list[float, float], 
        obstacle_position: list[float, float], 
        obstacle_sizes: float
    ):
    """
    Adapted from top answer of this stack overflow post: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm.
    It is assumed that all obstacles are spheres with a size of the obstacle_size constant.
    Sorry if the variables don't have very creative names and if its not well documented, if you would like to understand how this algorithm works, I encourage you to read the page linked above
    """
    # initializing
    e = np.array(start_point)
    l = np.array(end_point)
    c = np.array(obstacle_position)
    r = obstacle_sizes
    
    d = l - e
    f = e - c
    
    # using the algorithm from the stack overflow post (basically solving a quadratic equation)
    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - r*r

    discriminant = b*b-4*a*c
    
    return discriminant >= 0


   
        
def does_line_violate_no_sail_zone(start_point: list[float, float], end_point: list[float, float], true_wind_angle: float, no_sail_zone_size: float):
    """wind amgle is measured ccw from true north"""
    
    displacement = np.array(end_point) - np.array(start_point)
    displacement_magnitude = np.sqrt(displacement[0]**2 + displacement[1]**2 + displacement[2]**2)
    normalized_displacement = displacement/ displacement_magnitude
    
    true_wind_angle = (true_wind_angle + 90) % 360    # transform to ccw from true east
    up_wind_angle = (true_wind_angle + 180) % 360
    
    up_wind_vector = np.array(np.cos(up_wind_angle), np.sin(up_wind_angle))
    
    # find the angle between these two vectors
    angle_between = np.arccos(np.dot(up_wind_vector, normalized_displacement))
    
    if -no_sail_zone_size < angle_between < no_sail_zone_size:
        return True
    
    return False




def angle_between_vectors(v1: np.ndarray, v2: np.ndarray):
    
    v1_normalized = v1/ np.linalg.norm(v1)
    v2_normalized = v2/ np.linalg.norm(v2)
    
    return np.rad2deg(np.arccos(np.clip(np.dot(v1_normalized, v2_normalized), -1, 1)))
    
    
    
    
def is_angle_between_boundaries(angle: float, boundary1, boundary2):
    # TODO make the names of these a little bit less cringeworthy
    angle = np.deg2rad(angle)
    boundary1 = np.deg2rad(boundary1)
    boundary2 = np.deg2rad(boundary2)
    
    angle_vector = np.array([np.cos(angle), np.sin(angle)])
    boundary1_vector = np.array([np.cos(boundary1), np.sin(boundary1)])
    boundary2_vector = np.array([np.cos(boundary2), np.sin(boundary2)])
  
    return check_float_equivalence(
        angle_between_vectors(boundary1_vector, angle_vector) + angle_between_vectors(angle_vector, boundary2_vector), 
        angle_between_vectors(boundary1_vector, boundary2_vector)
    )
    
    
    
def get_maneuver_from_desired_heading(heading, desired_heading, true_wind_angle):
    
    # wind angle ccw from true east
    global_true_wind_angle = (true_wind_angle + heading) % 360
    
    #Calculate upwind angle (nominal angle at which course is heading straight into the wind)
    # Global True Wind Angle                    0     45    90   135  179 | 180  225  270  315  360
    # Global True Upwind Angle                  180   225   270  315  359 | 0    45   90   135  180
    global_true_upwind_angle = (global_true_wind_angle + 180) % 360

    if is_angle_between_boundaries(global_true_upwind_angle, heading, desired_heading): 
        return SailboatManeuvers.JIBE
    
    elif is_angle_between_boundaries(global_true_upwind_angle, heading, desired_heading): 
        return SailboatManeuvers.TACK
        
    else: 
        return SailboatManeuvers.STANDARD