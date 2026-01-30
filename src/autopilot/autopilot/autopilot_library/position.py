import navpy
import utm
import numpy as np
from pygeodesy.ellipsoidalKarney import LatLon
import pygeodesy


class Position:
    """
    A position that describes a point on the earth which is stored internally as its longitude and latitude.
    Whenever a function such as get_local_coordinates is called, this class has to convert to the proper position measurement system such as NED or UTM.
    
    This class mainly just calls on python libraries such as navpy, utm, and pygeodesy to convert to other position measurement systems.
    """
    
    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude
    
    def get_longitude_latitude(self):
        return np.array([self.longitude, self.latitude])
    
    def get_latitude_longitude(self) -> np.ndarray[float]:
        return np.array([self.latitude, self.longitude])
    
    def set_longitude_latitude(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude




    def get_utm(self):
        latlong = LatLon(self.latitude, -1 * self.longitude)
        utm_coord = pygeodesy.utm.toUtm8(latlong)
        return utm_coord.easting, utm_coord.northing

    def set_utm(self, easting: int, northing: int, zone_number: int, hemisphere: str) -> None:
        """
        If hemisphere is "N" then we are talking about the northern hemisphere and if the hemisphere is "S" we are talking about the southern hemisphere.  
        """
        if hemisphere != "N" and hemisphere != "S": 
            raise Exception("Incorrect Arguments Passed for set_utm")
        
        is_northern = True if "N" else False  
        self.latitude, self.longitude = utm.to_latlon(easting, northing, zone_number, northern=is_northern)
    
    
    
    
    def set_local_coordinates(self, local_x, local_y, reference_longitude, reference_latitude):
        self.latitude, self.longitude, _ = navpy.ned2lla([local_y, local_x, 0], reference_latitude, reference_longitude, 0)
        
    def get_local_coordinates(self, reference_longitude_latitude: np.ndarray):
        reference_latitude = reference_longitude_latitude[1]
        reference_longitude = reference_longitude_latitude[0]
        local_y, local_x, _ = navpy.lla2ned(self.latitude, self.longitude, 0, reference_latitude, reference_longitude, 0)
        return np.array([local_x, local_y])