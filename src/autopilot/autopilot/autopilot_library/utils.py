import numpy as np
from enum import Enum


import geopy
import geopy.distance
import pyproj

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
  


  
    
def check_float_equivalence(float1: float, float2: float) -> bool:
    """
    Args:
        float1 (float) 
        float2 (float)

    Returns:
        bool: True if the two floats are within 0.001 of each other and False if these floats are not within 0.001 of each other.
    """
    return abs(float1 - float2) <= 0.001



def cartesian_vector_to_polar(x: float, y: float) -> tuple:
    """
    Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).

    Args:
        x (float): x coordinate of the vector
        y (float): y coordinate of the vector

    Returns:
        tuple: Outputs a tuple of magnitude and direction (counter-clockwise from the x axis) of the inputted vector.
            Output direction is between 0 and 360 degrees
    """
    
    # arctan2 doesn't like when we pass 2 zeros into it so we should cover that case
    if x == 0. and y == 0.:
        return 0., 0.
    magnitude = np.sqrt(x**2 + y**2)
    direction = np.arctan2(y, x) # radians
    direction = direction * (180/np.pi)  # angle from -180 to 180 degrees
    direction = direction % 360  # angle from 0 to 360 degrees
    return magnitude, direction



def get_angle_between_vectors(vector1: np.ndarray, vector2: np.ndarray) -> float:
    """
    Args:
        vector1 (np.ndarray)
        vector2 (np.ndarray)

    Returns:
        float: the smallest angle between vector1 and vector2
    """
    
    vector1_normalized = vector1/ np.linalg.norm(vector1)
    vector2_normalized = vector2/ np.linalg.norm(vector2)
    
    return np.rad2deg(np.arccos(np.clip(np.dot(vector1_normalized, vector2_normalized), -1, 1)))

    
def get_distance_between_angles(angle1: float, angle2: float) -> float:
    """
    Takes two angles in degrees and computes the shortest angular distance between them.
    For example if angle1 = 30 degrees and angle2 = 50 degrees, then the output of this function would be 20 degrees. 
    
    https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point

    Args:
        angle1 (float): any angle measured in degrees
        angle2 (float): any angle measured in degrees

    Returns:
        float: the shortest angular distance between the two angles (in degrees) 
    """
    # https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point
    return -1 * ((float(angle1) - float(angle2) + 180) % 360 - 180)
    


def get_bearing(current_position: Position, destination_position: Position) -> float:
    """
    Gets the bearing towards a specific destination point, from our current location. 
    The bearing is just the angle between two points on earth 
    (AKA which direction to travel in to get to the destination position from the current position)

    Args:
        current_position (Position): a Position object that represents the current position
        destination_position (Position): a Position object that represents the position that you would like to travel towards

    Returns:
        float: the bearing as an angle between 0 to 360, counter clockwise, measured from east. 
        This value tells you which direction you need to travel in to get to your destination.
    """
    
    current_latitude, current_longitude = current_position.get_longitude_latitude()
    destination_latitude, destination_longitude = destination_position.get_longitude_latitude()
    azimuth_heading, _, _ = pyproj.Geod(ellps='WGS84').inv(current_longitude, current_latitude, destination_longitude, destination_latitude)
    
    
    # azimuth is measured clockwise from true north but we want counter-clockwise from true east
    return (-azimuth_heading + 90) % 360
    


def get_distance_between_positions(position1: Position, position2: Position):
    """
    Args:
        position1 (Position)
        position2 (Position)

    Returns:
        float: distance between position1 and position2 in meters 
    """
    return geopy.distance.geodesic(position1.get_longitude_latitude(), position2.get_longitude_latitude()).m


    

    

             


def is_angle_between_boundaries(angle: float, boundary1: float, boundary2: float) -> bool:
    """
    TODO Better documentation
    
    Args:
        angle (float): an angle measured in degrees counter-clockwise from the x axis
        boundary1 (float): an angle measured in degrees counter-clockwise from the x axis 
        boundary2 (float): an angle measured in degrees counter-clockwise from the x axis

    Returns:
        bool: If "angle" is between "boundary1" and "boundary2", then return True and if not, then return False
    """
    angle = np.deg2rad(angle)
    boundary1 = np.deg2rad(boundary1)
    boundary2 = np.deg2rad(boundary2)
    
    angle_vector = np.array([np.cos(angle), np.sin(angle)])
    boundary1_vector = np.array([np.cos(boundary1), np.sin(boundary1)])
    boundary2_vector = np.array([np.cos(boundary2), np.sin(boundary2)])
  
    return check_float_equivalence(
        get_angle_between_vectors(boundary1_vector, angle_vector) + get_angle_between_vectors(angle_vector, boundary2_vector), 
        get_angle_between_vectors(boundary1_vector, boundary2_vector)
    )




    
    
    
    
    
    
    
    
    
    
    
# ==============================================================================
# UTIL FUNCTIONS THAT ARE NOT FULLY TESTED
# ==============================================================================
    
    
def does_line_violate_no_sail_zone(
        current_position: list[float, float], 
        destination_position: list[float, float], 
        global_true_wind_angle: float, 
        no_sail_zone_size: float
    ) -> bool:
    """
    TODO: NOT FULLY TESTED
    
    This function takes 
    wind angle is measured counter-clockwise from true east


    Args:
        current_position (list[float, float]): A list that represents the current position in cartesian coordinates. 
            For example: [x_coordinate, y_coordinate] is the form that you should use
        destination_position (list[float, float]): A list that represents the position you want to directly travel to in cartesian coordinates. 
            For example: [x_coordinate, y_coordinate] is the form that you should use
        global_true_wind_angle (float): The global true wind angle measured counter-clockwise from true east as an angle in degrees
        no_sail_zone_size (float): 

    Returns:
        bool: _description_
    """
    
    displacement = np.array(destination_position) - np.array(current_position)
    displacement_magnitude = np.sqrt(displacement[0]**2 + displacement[1]**2 + displacement[2]**2)
    normalized_displacement = displacement/ displacement_magnitude
    
    # the upwind angle is the angle that is opposite to the wind angle. For example, if the wind is blowing straight east, 
    # then the wind angle would be 0 degrees and the upwind angle would be 180 degrees
    global_true_upwind_angle = (global_true_wind_angle + 180) % 360
    
    up_wind_vector = np.array(np.cos(global_true_upwind_angle), np.sin(global_true_upwind_angle))
    
    # find the angle between these two vectors
    angle_between = np.arccos(np.dot(up_wind_vector, normalized_displacement))
    
    # TODO I AM PRETTY SURE THAT WE NEED TO USE THE IS ANGLE BETWEEN BOUNDARIES FUNCTION FOR THIS?
    if -no_sail_zone_size < angle_between < no_sail_zone_size:
        return True
    
    return False


def does_line_segment_intersect_circle(
        line_segment_start_position: list[float, float], 
        line_segment_end_position: list[float, float], 
        circle_position: list[float, float], 
        circle_radius: float
    ) -> bool:
    """
    TODO: NOT FULLY TESTED

    Adapted from top answer of this stack overflow post: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm.
    Sorry if the variables don't have very creative names and if its not well documented, 
    but if you would like to understand how this algorithm works, I encourage you to read the page linked above


    Args:
        line_segment_start_position (list[float, float]): the starting point of the line segment as an array of the [x_coordinate, y_coordinate]
        line_segment_end_position (list[float, float]): the end point of the line segment as an array of the [x_coordinate, y_coordinate]
        circle_position (list[float, float]): the position of the circle as an array of the [x_coordinate, y_coordinate]
        circle_radius (float): the radius of the circle

    Returns:
        bool: whether or not the 
    """
    
    
    # initializing
    e = np.array(line_segment_start_position)
    l = np.array(line_segment_end_position)
    c = np.array(circle_position)
    r = circle_radius
    
    d = l - e
    f = e - c
    
    # using the algorithm from the stack overflow post (basically solving a quadratic equation)
    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - r*r

    discriminant = b*b-4*a*c
    
    return discriminant >= 0
