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




// --- Distance and bearing functions ---
double get_distance_haversine(double latitude1, double longitude1, double latitude2, double longitude2);
double get_distance_vincenty(double latitude1, double longitude1, double latitude2, double longitude2);
double get_distance(double latitude1, double longitude1, double latitude2, double longitude2);
double calculate_bearing(double latitude1, double longitude1, double latitude2, double longitude2);


// --- Coordinate transformation functions ---

// LLA to ECEF
std::array<double, 3> lla2ecef(double lat_deg, double lon_deg, double alt_m);

// ECEF to LLA (iterative)
std::array<double, 3> ecef2lla(double x, double y, double z);

// Build ECEF to NED rotation matrix
std::array<std::array<double, 3>, 3> nedRotation(double lat_deg, double lon_deg);

// multiply matrix * vector
std::array<double, 3> matmul(const std::array<std::array<double,3>,3> &C, const std::array<double,3> &v);

// ECEF to NED
std::array<float, 3> ecef2ned(const std::array<double,3> &ecef_vector, double reference_latitude, double reference_longitude);

// NED to ECEF
std::array<double, 3> ned2ecef(const std::array<double,3> &ned_vector, double reference_latitude, double reference_longitude);

// LLA to NED
std::array<float, 3> lla2ned(double latitude, double longitude, double altitude, double reference_latitude, double reference_longitude, double reference_altitude);

// NED to LLA
std::array<double, 3> ned2lla(const std::array<double,3> &ned_vector, double reference_latitude, double reference_longitude, double reference_altitude);