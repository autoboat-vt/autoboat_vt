import geopy.distance
import numpy as np
import numpy.typing as npt
import pyproj

from .position import Position


def check_float_equivalence(float1: float, float2: float) -> bool:
    """
    Checks if two floats are equivalent to within 0.001.

    Parameters
    ----------
    float1
        The first float to compare.
    float2
        The second float to compare.

    Returns
    -------
    bool
        `True` if the two floats are equivalent to within 0.001, `False` otherwise.
    """

    return abs(float1 - float2) <= 0.001


def cartesian_vector_to_polar(x: float, y: float) -> tuple[float, float]:
    """
    Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).

    Parameters
    ----------
    x
        The x coordinate of the vector.
    y
        The y coordinate of the vector.

    Returns
    -------
    tuple
        Outputs a tuple of magnitude and direction (counter-clockwise from the x axis) of the inputted vector.
        Output direction is between 0 and 360 degrees.
    """

    if x == 0.0 and y == 0.0:
        return 0.0, 0.0

    magnitude = np.hypot(x, y)
    direction = np.degrees(np.arctan2(y, x)) % 360
    return magnitude, direction


def get_angle_between_vectors(vector1: npt.NDArray[np.float64], vector2: npt.NDArray[np.float64]) -> float:
    """
    Takes two vectors and computes the smallest angle between them in degrees.

    Parameters
    ----------
    vector1
        The first vector.
    vector2
        The second vector.

    Returns
    -------
    float
        The smallest angle between the two vectors in degrees.
    """

    vector1_normalized = vector1 / np.linalg.norm(vector1)
    vector2_normalized = vector2 / np.linalg.norm(vector2)

    return np.rad2deg(np.arccos(np.clip(np.dot(vector1_normalized, vector2_normalized), -1, 1)))


def get_distance_between_angles(angle1: float, angle2: float) -> float:
    """
    Takes two angles in degrees and computes the shortest angular distance between them.

    Note:
    -----
    Adapted from this stack overflow post:
    https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point

    For example:
        >>> get_distance_between_angles(30, 50)
        20

    Parameters
    ----------
    angle1
        The first angle in degrees.
    angle2
        The second angle in degrees.

    Returns
    -------
    float
        The shortest angular distance between the two angles (in degrees).
    """

    return -1 * ((float(angle1) - float(angle2) + 180) % 360 - 180)


def get_bearing(current_position: Position, destination_position: Position) -> float:
    """
    Gets the bearing towards a specific destination point, from our current location.

    The bearing is just the angle between two points on earth
    (AKA which direction to travel in to get to the destination position from the current position).

    Parameters
    ----------
    current_position
        The current position.
    destination_position
        The destination position.

    Returns
    -------
    float
        The bearing as an angle between ``0`` to ``360``, counter clockwise, measured from east.
        This value tells you which direction you need to travel in to get to your destination.
    """

    current_latitude, current_longitude = current_position.get_latitude_longitude()
    destination_latitude, destination_longitude = destination_position.get_latitude_longitude()
    azimuth_heading, _, _ = pyproj.Geod(ellps='WGS84').inv(
        current_longitude, current_latitude, destination_longitude, destination_latitude
    )

    # azimuth is measured clockwise from true north but we want counter-clockwise from true east
    return (-azimuth_heading + 90) % 360


def get_distance_between_positions(position1: Position, position2: Position) -> float:
    """
    Gets the distance between two ``Position`` objects in meters.

    Parameters
    ----------
    position1
        The first position.
    position2
        The second position.

    Returns
    -------
    float
        The distance between the two positions in meters.
    """

    return geopy.distance.geodesic(position1.get_latitude_longitude(), position2.get_latitude_longitude()).m


def is_angle_between_boundaries(angle: float, boundary1: float, boundary2: float) -> bool:
    """
    TODO Better documentation.

    Parameters
    ----------
    angle
        An angle measured in degrees counter-clockwise from the x axis.
    boundary1
        An angle measured in degrees counter-clockwise from the x axis.
    boundary2
        An angle measured in degrees counter-clockwise from the x axis.

    Returns
    -------
    bool
        If "angle" is between ``boundary1`` and ``boundary2``, then return ``True`` and if not, return ``False``.
    """

    angle_rad, b1_rad, b2_rad = np.deg2rad([angle, boundary1, boundary2])

    angle_vector = np.array([np.cos(angle_rad), np.sin(angle_rad)])
    boundary1_vector = np.array([np.cos(b1_rad), np.sin(b1_rad)])
    boundary2_vector = np.array([np.cos(b2_rad), np.sin(b2_rad)])

    # Angle is between boundaries if the sum of angles from b1->angle and angle->b2 equals b1->b2
    angle_b1_to_angle = get_angle_between_vectors(boundary1_vector, angle_vector)
    angle_angle_to_b2 = get_angle_between_vectors(angle_vector, boundary2_vector)
    angle_b1_to_b2 = get_angle_between_vectors(boundary1_vector, boundary2_vector)

    return check_float_equivalence(angle_b1_to_angle + angle_angle_to_b2, angle_b1_to_b2)


# ==============================================================================
# UTIL FUNCTIONS THAT ARE NOT FULLY TESTED
# ==============================================================================


def does_line_violate_no_sail_zone(
    current_position: tuple[float, float],
    destination_position: tuple[float, float],
    global_true_wind_angle: float,
    no_sail_zone_size: float,
) -> bool:
    """
    This function takes wind angle as measured counter-clockwise from true east.

    TODO: NOT FULLY TESTED.

    Parameters
    ----------
    current_position
        A tuple that represents the current position in cartesian coordinates.
        For example: ``[x_coordinate, y_coordinate]`` is the form that you should use.

    destination_position
        A tuple that represents the position you want to directly travel to in cartesian coordinates.
        For example: ``[x_coordinate, y_coordinate]`` is the form that you should use.

    global_true_wind_angle
        The global true wind angle measured counter-clockwise from true east as an angle in degrees.

    no_sail_zone_size
        The size of the no sail zone in degrees.

    Returns
    -------
    bool
        Whether or not the line between the current position and the destination position violates the no sail zone.
    """

    displacement = np.asarray(destination_position, dtype=np.float64) - np.asarray(current_position, dtype=np.float64)
    normalized_displacement = displacement / np.linalg.norm(displacement)

    # The upwind angle is opposite to the wind angle (e.g., wind at 0° means upwind at 180°)
    global_true_upwind_rad: float = np.deg2rad((global_true_wind_angle + 180) % 360)
    up_wind_vector = np.array(
        [np.cos(global_true_upwind_rad), np.sin(global_true_upwind_rad)],
        dtype=np.float64,
    )

    # Find the angle between the upwind vector and displacement vector
    angle_between: float = np.degrees(np.arccos(np.clip(np.dot(up_wind_vector, normalized_displacement), -1, 1)))

    return angle_between < no_sail_zone_size


def does_line_segment_intersect_circle(
    line_segment_start_position: tuple[float, float],
    line_segment_end_position: tuple[float, float],
    circle_position: tuple[float, float],
    circle_radius: float,
) -> bool:
    """
    Checks if a line segment intersects a circle.

    TODO: NOT FULLY TESTED

    Note
    ----
    Adapted from top answer of this stack overflow post:
    https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm.

    Parameters
    ----------
    line_segment_start_position
        The starting point of the line segment as a tuple of the form ``[x_coordinate, y_coordinate]``.
    line_segment_end_position
        The end point of the line segment as a tuple of the form ``[x_coordinate, y_coordinate]``.
    circle_position
        The position of the circle as a tuple of the form ``[x_coordinate, y_coordinate]``.
    circle_radius
        The radius of the circle.

    Returns
    -------
    bool
        Whether or not the line segment intersects the circle.

    Raises
    ------
    ValueError
        If the circle radius is not positive.
    """

    if circle_radius <= 0:
        raise ValueError("Circle radius must be positive.")

    dx = line_segment_end_position[0] - line_segment_start_position[0]
    dy = line_segment_end_position[1] - line_segment_start_position[1]

    fx = line_segment_start_position[0] - circle_position[0]
    fy = line_segment_start_position[1] - circle_position[1]

    a = (dx * dx) + (dy * dy)
    ff = (fx * fx) + (fy * fy)
    r2 = circle_radius * circle_radius

    if ff <= r2:
        return True

    if a == 0.0:
        return ff <= r2

    b = 2 * (fx * dx + fy * dy)
    c = ff - r2

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    sqrt_disc: float = np.sqrt(discriminant)
    inv_2a = 1 / (2 * a)
    t1 = (-b - sqrt_disc) * inv_2a
    t2 = (-b + sqrt_disc) * inv_2a

    return (0 <= t1 <= 1) or (0 <= t2 <= 1)
