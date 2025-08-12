import numpy as np

def cartesian_vector_to_polar(x, y):
    """
    Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).
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