import navpy
import numpy as np
import numpy.typing as npt
import pygeodesy
import utm
from pygeodesy.ellipsoidalKarney import LatLon


class Position:
    """
    A position that describes a point on the earth which is stored internally as its longitude and latitude.

    Whenever a function such as ``get_local_coordinates`` is called, this class has to convert to the proper
    position measurement system such as NED or UTM.

    This class mainly just calls on python libraries such as ``navpy``, ``utm``, and ``pygeodesy`` to convert to
    other position measurement systems.
    """

    def __init__(self, longitude: float, latitude: float) -> None:
        self.longitude = longitude
        self.latitude = latitude



    def set_longitude_latitude(self, longitude: float, latitude: float) -> None:
        """
        Sets the position using longitude and latitude.

        Parameters
        ----------
        longitude
            The longitude to set the position to.
        latitude
            The latitude to set the position to.
        """

        self.longitude = longitude
        self.latitude = latitude

    def get_longitude_latitude(self) -> npt.NDArray[np.float64]:
        """
        Gets the longitude and latitude of this position.

        Returns
        -------
        array
            An array where the first element is the longitude and the second element is the latitude.
        """

        return np.array([self.longitude, self.latitude], dtype=np.float64)

    def get_latitude_longitude(self) -> npt.NDArray[np.float64]:
        """
        Gets the latitude and longitude of this position.

        Returns
        -------
        array
            An array where the first element is the latitude and the second element is the longitude.
        """

        return np.array([self.latitude, self.longitude], dtype=np.float64)



    def set_utm(self, easting: int, northing: int, zone_number: int, hemisphere: str) -> None:
        """
        Sets the position using UTM coordinates.

        Parameters
        ----------
        easting
            The easting value in UTM coordinates.
        northing
            The northing value in UTM coordinates.
        zone_number
            The UTM zone number.
        hemisphere
            "N" for northern hemisphere, "S" for southern hemisphere.

        Raises
        ------
        Exception
            If an invalid hemisphere is provided.
        """

        if hemisphere not in {"N", "S"}:
            raise Exception("Incorrect Arguments Passed for set_utm")

        is_northern = hemisphere == "N"
        latitude, longitude = utm.to_latlon(easting, northing, zone_number, northern=is_northern)

        self.latitude = float(latitude)
        self.longitude = float(longitude)

    def get_utm(self) -> tuple[float, float]:
        """
        Gets the UTM coordinates of this position.

        Returns
        -------
        tuple
            A tuple where the first element is the easting and the second element is the northing.
        """

        latlong = LatLon(self.latitude, -1 * self.longitude)
        utm_coord = pygeodesy.utm.toUtm8(latlong)

        return utm_coord.easting, utm_coord.northing



    def set_local_coordinates(
        self, local_x: float, local_y: float, reference_longitude: float, reference_latitude: float
    ) -> None:
        """
        Sets the position using local NED coordinates.

        Parameters
        ----------
        local_x
            The local x coordinate (north).
        local_y
            The local y coordinate (east).
        reference_longitude
            The reference longitude for the NED frame.
        reference_latitude
            The reference latitude for the NED frame.
        """

        self.latitude, self.longitude, _ = navpy.ned2lla(local_y, local_x, 0, reference_latitude, reference_longitude, 0)

    def get_local_coordinates(self, reference_longitude_latitude: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Gets the local NED coordinates of this position with respect to a reference position.

        Parameters
        ----------
        reference_longitude_latitude
            Array where the first element is the reference longitude and the second element is the reference latitude.

        Returns
        -------
        array
            Array where the first element is the local x coordinate (north)
            and the second element is the local y coordinate (east).
        """

        reference_latitude: float = reference_longitude_latitude[1]
        reference_longitude: float = reference_longitude_latitude[0]

        local_y, local_x, _ = navpy.lla2ned(self.latitude, self.longitude, 0, reference_latitude, reference_longitude, 0)

        return np.array([local_x, local_y], dtype=np.float64)
