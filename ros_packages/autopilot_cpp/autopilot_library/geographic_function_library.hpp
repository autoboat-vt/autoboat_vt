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
 * @return float Distance in meters.
 */
float get_distance_haversine(double latitude1, double longitude1, double latitude2, double longitude2);

/**
 * @brief Calculate the distance between two points using Vincenty's formulae (more accurate than Haversine).
 * @param latitude1 Latitude of the first point in degrees.
 * @param longitude1 Longitude of the first point in degrees.
 * @param latitude2 Latitude of the second point in degrees.
 * @param longitude2 Longitude of the second point in degrees.
 * @return float Distance in meters.
 */
float get_distance_vincenty(double latitude1, double longitude1, double latitude2, double longitude2);

/**
 * @brief Calculate the distance between two points using the configured distance function.
 * @param latitude1 Latitude of the first point in degrees.
 * @param longitude1 Longitude of the first point in degrees.
 * @param latitude2 Latitude of the second point in degrees.
 * @param longitude2 Longitude of the second point in degrees.
 * @return float Distance in meters.
 */
float get_distance(double latitude1, double longitude1, double latitude2, double longitude2);

/**
 * @brief Calculate the bearing from one point to another.
 * @param latitude1 Latitude of the starting point in degrees.
 * @param longitude1 Longitude of the starting point in degrees.
 * @param latitude2 Latitude of the destination point in degrees.
 * @param longitude2 Longitude of the destination point in degrees.
 * @return float Bearing in degrees (counter-clockwise from true East).
 */
float calculate_bearing(double latitude1, double longitude1, double latitude2, double longitude2);


/**
 * @brief Convert Geodetic coordinates (LLA) to Earth-Centered, Earth-Fixed (ECEF) coordinates.
 * @param latitude Latitude in degrees.
 * @param longitude Longitude in degrees.
 * @param altitude Altitude in meters.
 * @return std::array<double, 3> ECEF coordinates {x, y, z}.
 */
std::array<double, 3> lla2ecef(double latitude, double longitude, double altitude);

/**
 * @brief Convert ECEF coordinates to Geodetic coordinates (LLA).
 * @param x_coordinate ECEF x-coordinate.
 * @param y_coordinate ECEF y-coordinate.
 * @param z_coordinate ECEF z-coordinate.
 * @return std::array<double, 3> LLA coordinates {latitude, longitude, altitude}.
 */
std::array<double, 3> ecef2lla(double x_coordinate, double y_coordinate, double z_coordinate);

/**
 * @brief Build a rotation matrix from ECEF to North-East-Down (NED).
 * @param latitude Latitude in degrees.
 * @param longitude Longitude in degrees.
 * @return std::array<std::array<double, 3>, 3> 3x3 rotation matrix.
 */
std::array<std::array<double, 3>, 3> nedRotation(double latitude, double longitude);

/**
 * @brief Multiply a 3x3 matrix by a 3x1 vector (double version).
 * @param rotation_matrix 3x3 matrix.
 * @param input_vector 3x1 vector.
 * @return std::array<double, 3> Resultant vector.
 */
std::array<double, 3> matmul(const std::array<std::array<double,3>,3> &rotation_matrix, const std::array<double,3> &input_vector);

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
std::array<double, 3> ned2ecef(const std::array<float,3> &ned_vector, double reference_latitude, double reference_longitude);

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
 * @return std::array<double, 3> LLA coordinates {latitude, longitude, altitude}.
 */
std::array<double, 3> ned2lla(const std::array<float,3> &ned_vector, double reference_latitude, double reference_longitude, double reference_altitude);