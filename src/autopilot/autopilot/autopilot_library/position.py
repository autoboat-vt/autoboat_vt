import navpy
import numpy as np
import numpy.typing as npt
import pygeodesy
import utm
from pygeodesy.ellipsoidalKarney import LatLon


class Position:
    """
    A position that describes a point on the earth which is stored internally as its longitude and latitude.

    Whenever a function such as `get_local_coordinates` is called, this class has to convert to the proper position measurement system such as NED or UTM.

    This class mainly just calls on python libraries such as `navpy`, `utm`, and `pygeodesy` to convert to other position measurement systems.
    """

    def __init__(self, longitude: float, latitude: float) -> None:
        self.longitude = longitude
        self.latitude = latitude

    def set_longitude_latitude(self, longitude: float, latitude: float) -> None:
        self.longitude = longitude
        self.latitude = latitude

    def get_longitude_latitude(self) -> npt.NDArray[np.float64]:
        return np.array([self.longitude, self.latitude])

    def get_latitude_longitude(self) -> npt.NDArray[np.float64]:
        return np.array([self.latitude, self.longitude])

    def set_utm(self, easting: int, northing: int, zone_number: int, hemisphere: str) -> None:
        """
        Sets the position using UTM coordinates.

        Args:
            easting (int): The easting value in UTM coordinates.
            northing (int): The northing value in UTM coordinates.
            zone_number (int): The UTM zone number.
            hemisphere (str): "N" for northern hemisphere, "S" for southern hemisphere.
        """

        if hemisphere not in ["N", "S"]:
            raise Exception("Incorrect Arguments Passed for set_utm")

        is_northern = hemisphere == "N"
        self.latitude, self.longitude = utm.to_latlon(easting, northing, zone_number, northern=is_northern)

    def get_utm(self) -> tuple[float]:
        latlong = LatLon(self.latitude, -1 * self.longitude)
        utm_coord = pygeodesy.utm.toUtm8(latlong)
        return utm_coord.easting, utm_coord.northing

    def set_local_coordinates(
        self, local_x: float, local_y: float, reference_longitude: float, reference_latitude: float
    ) -> None:
        """
        Sets the position using local NED coordinates.

        Args:
            local_x (float): The local x coordinate (north).
            local_y (float): The local y coordinate (east).
            reference_longitude (float): The reference longitude for the NED frame.
            reference_latitude (float): The reference latitude for the NED frame.
        """

        lat, lon, _ = navpy.ned2lla(local_y, local_x, 0, reference_latitude, reference_longitude, 0)

        self.latitude = lat
        self.longitude = lon

    def get_local_coordinates(self, reference_longitude_latitude: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Gets the local NED coordinates of this position with respect to a reference position.

        Args:
            reference_longitude_latitude (np.array): Array where the first element is the reference longitude and the second element is the reference latitude.

        Returns:
            np.array: Array where the first element is the local x coordinate (north) and the second element is the local y coordinate (east).
        """

        reference_latitude = reference_longitude_latitude[1]
        reference_longitude = reference_longitude_latitude[0]

        local_y, local_x, _ = navpy.lla2ned(
            self.latitude, self.longitude, 0, reference_latitude, reference_longitude, 0
        )

        return np.array([local_x, local_y])
