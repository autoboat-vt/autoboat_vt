//
// This is just a library of functions taken from geopy, navpy, and pyproj
// This implements basic distance, azimuth calculations as well as coordinate transformations 
// between NED (North, East, Down), LLA (Latitude, Longitude, Altitude), and ECEF (Earth-centered, Earth-fixed) coordinate systems
//


// Here are some good resources to look at if you want to learn more about this function library:
// https://www.codeguru.com/cplusplus/geographic-distance-and-azimuth-calculations/
// https://en.wikipedia.org/wiki/Haversine_formula
// https://en.wikipedia.org/wiki/Vincenty%27s_formulae 
// https://github.com/NavPy/NavPy/blob/master/navpy/core/navpy.py
// https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
// https://en.wikipedia.org/wiki/Geographic_coordinate_system
// https://en.wikipedia.org/wiki/World_Geodetic_System




#pragma once

#include <tuple>
#include <cmath>
#include <array>
#include <vector>
#include <stdexcept>
#include <string>



// choose vincenty for more precise but slower calculations
// choose haversine for less precise but faster calculations
extern const std::string DISTANCE_FUNCTION_TO_USE;


// Constants taken from here: https://en.wikipedia.org/wiki/World_Geodetic_System
constexpr double WGS84_A = 6378137.0;              // semi-major axis
constexpr double WGS84_E2 = 6.69437999014e-3;      // eccentricity^2
constexpr double WGS84_F = 1 / 298.257223563;     // flattening




/**
 * @brief Calculate the great-circle distance between two points using the Haversine formula.
 * @param latitude1 Latitude of the first point in degrees.
 * @param longitude1 Longitude of the first point in degrees.
 * @param latitude2 Latitude of the second point in degrees.
 * @param longitude2 Longitude of the second point in degrees.
 * @return double Distance in meters.
 */
double get_distance_haversine(double latitude1, double longitude1, double latitude2, double longitude2);

/**
 * @brief Calculate the distance between two points using Vincenty's formulae (more accurate than Haversine).
 * @param latitude1 Latitude of the first point in degrees.
 * @param longitude1 Longitude of the first point in degrees.
 * @param latitude2 Latitude of the second point in degrees.
 * @param longitude2 Longitude of the second point in degrees.
 * @return double Distance in meters.
 */
double get_distance_vincenty(double latitude1, double longitude1, double latitude2, double longitude2);

/**
 * @brief Calculate the distance between two points using the configured distance function.
 * @param latitude1 Latitude of the first point in degrees.
 * @param longitude1 Longitude of the first point in degrees.
 * @param latitude2 Latitude of the second point in degrees.
 * @param longitude2 Longitude of the second point in degrees.
 * @return double Distance in meters.
 */
double get_distance(double latitude1, double longitude1, double latitude2, double longitude2);

/**
 * @brief Calculate the bearing from one point to another.
 * @param latitude1 Latitude of the starting point in degrees.
 * @param longitude1 Longitude of the starting point in degrees.
 * @param latitude2 Latitude of the destination point in degrees.
 * @param longitude2 Longitude of the destination point in degrees.
 * @return double Bearing in degrees (counter-clockwise from true East).
 */
double calculate_bearing(double latitude1, double longitude1, double latitude2, double longitude2);


/**
 * @brief Convert Geodetic coordinates (LLA) to Earth-Centered, Earth-Fixed (ECEF) coordinates.
 * @param lat_deg Latitude in degrees.
 * @param lon_deg Longitude in degrees.
 * @param alt_m Altitude in meters.
 * @return std::array<double, 3> ECEF coordinates {x, y, z}.
 */
std::array<double, 3> lla2ecef(double lat_deg, double lon_deg, double alt_m);

/**
 * @brief Convert ECEF coordinates to Geodetic coordinates (LLA).
 * @param x ECEF x-coordinate.
 * @param y ECEF y-coordinate.
 * @param z ECEF z-coordinate.
 * @return std::array<double, 3> LLA coordinates {lat_deg, lon_deg, alt_m}.
 */
std::array<double, 3> ecef2lla(double x, double y, double z);

/**
 * @brief Build a rotation matrix from ECEF to North-East-Down (NED).
 * @param lat_deg Latitude in degrees.
 * @param lon_deg Longitude in degrees.
 * @return std::array<std::array<double, 3>, 3> 3x3 rotation matrix.
 */
std::array<std::array<double, 3>, 3> nedRotation(double lat_deg, double lon_deg);

/**
 * @brief Multiply a 3x3 matrix by a 3x1 vector.
 * @param C 3x3 matrix.
 * @param v 3x1 vector.
 * @return std::array<double, 3> Resultant vector.
 */
std::array<double, 3> matmul(const std::array<std::array<double,3>,3> &C, const std::array<double,3> &v);

/**
 * @brief Convert ECEF coordinates to NED coordinates relative to a reference point.
 * @param ecef_vector ECEF {x, y, z}.
 * @param reference_latitude Reference latitude in degrees.
 * @param reference_longitude Reference longitude in degrees.
 * @return std::array<float, 3> NED coordinates {North, East, Down}.
 */
std::array<float, 3> ecef2ned(const std::array<double,3> &ecef_vector, double reference_latitude, double reference_longitude);

/**
 * @brief Convert NED coordinates to ECEF coordinates relative to a reference point.
 * @param ned_vector NED {North, East, Down}.
 * @param reference_latitude Reference latitude in degrees.
 * @param reference_longitude Reference longitude in degrees.
 * @return std::array<double, 3> ECEF coordinates {x, y, z}.
 */
std::array<double, 3> ned2ecef(const std::array<double,3> &ned_vector, double reference_latitude, double reference_longitude);

/**
 * @brief Convert Geodetic (LLA) to NED coordinates relative to a reference point.
 * @param latitude Target latitude.
 * @param longitude Target longitude.
 * @param altitude Target altitude.
 * @param reference_latitude Reference latitude.
 * @param reference_longitude Reference longitude.
 * @param reference_altitude Reference altitude.
 * @return std::array<float, 3> NED coordinates {North, East, Down}.
 */
std::array<float, 3> lla2ned(double latitude, double longitude, double altitude, double reference_latitude, double reference_longitude, double reference_altitude);

/**
 * @brief Convert NED to Geodetic (LLA) coordinates relative to a reference point.
 * @param ned_vector NED {North, East, Down}.
 * @param reference_latitude Reference latitude.
 * @param reference_longitude Reference longitude.
 * @param reference_altitude Reference altitude.
 * @return std::array<double, 3> LLA coordinates {lat_deg, lon_deg, alt_m}.
 */
std::array<double, 3> ned2lla(const std::array<double,3> &ned_vector, double reference_latitude, double reference_longitude, double reference_altitude);