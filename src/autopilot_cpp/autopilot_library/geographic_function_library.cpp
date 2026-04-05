#include "geographic_function_library.hpp"


const std::string DISTANCE_FUNCTION_TO_USE = "haversine";


// haversine distance for distance between 2 points on a spheroid (approximation to an elipsoid)
// this is a less accurate distance function than vincenty since it assumes that the earth is a perfect sphere
// https://en.wikipedia.org/wiki/Haversine_formula
float get_distance_haversine(double latitude1, double longitude1, double latitude2, double longitude2) {
    // Haversine in meters
    const double R = WGS84_A;
    double dlat = (latitude2 - latitude1) * M_PI / 180.0;
    double dlon = (longitude2 - longitude1) * M_PI / 180.0;
    double a = sin(dlat/2)*sin(dlat/2) + cos(latitude1*M_PI/180.0)*cos(latitude2*M_PI/180.0)*sin(dlon/2)*sin(dlon/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return static_cast<float>(R * c);
}



// Using vincentys formula to calculate the distance between 2 points on an elipsoid
// this is a more accurate distance function than haversine since it doesn't make the assumption that the earth is a perfect sphere
// https://github.com/dariusarnold/vincentys-formula
// https://en.wikipedia.org/wiki/Vincenty%27s_formulae 
float get_distance_vincenty(double latitude1, double longitude1, double latitude2, double longitude2) {
    using namespace std;

    constexpr double req = WGS84_A;             // Radius at equator
    constexpr double flat = WGS84_F;            // Flattening of earth
    constexpr double rpol = (1 - flat) * req;

    double sin_sigma, cos_sigma, sigma, sin_alpha, cos_sq_alpha, cos2sigma;
    double C, lam_pre;

    // convert to radians
    latitude1 = M_PI * latitude1 / 180.0;
    latitude2 = M_PI * latitude2 / 180.0;
    longitude1 = M_PI * longitude1 / 180.0;
    longitude2 = M_PI * longitude2 / 180.0;

    const double u1 = atan((1 - flat) * tan(latitude2));
    const double u2 = atan((1 - flat) * tan(latitude1));

    double lon = longitude1 - longitude2;
    double lam = lon;
    constexpr double tol = 10.e-12; // iteration tolerance
    double diff = 1.;

    while (abs(diff) > tol) {
        sin_sigma = sqrt(pow((cos(u2) * sin(lam)), 2.) + pow(cos(u1)*sin(u2) - sin(u1)*cos(u2)*cos(lam), 2.));
        
        if (sin_sigma == 0.) {
            // Coincident points, prevent division by zero resulting in NaN.
            return 0.0f;
        }
        
        cos_sigma = sin(u1) * sin(u2) + cos(u1) * cos(u2) * cos(lam);
        sigma = atan(sin_sigma / cos_sigma);
        
        if (sigma <= 0) sigma = M_PI + sigma;
        
        sin_alpha = (cos(u1) * cos(u2) * sin(lam)) / sin_sigma;
        cos_sq_alpha = 1 - pow(sin_alpha, 2.);
        
        if (cos_sq_alpha == 0.) {
            cos2sigma = 0.;
        } 
        else {
            cos2sigma = cos_sigma - ((2 * sin(u1) * sin(u2)) / cos_sq_alpha);
        }
        
        C = (flat / 16) * cos_sq_alpha * (4 + flat * (4 - 3 * cos_sq_alpha));
        lam_pre = lam;
        lam = lon + (1 - C) * flat * sin_alpha * (sigma + C * sin_sigma * (cos2sigma + C * cos_sigma * (2 * pow(cos2sigma, 2.) - 1)));
        diff = abs(lam_pre - lam);
    }

    const double usq = cos_sq_alpha * ((pow(req, 2.) - pow(rpol, 2.)) / pow(rpol ,2.));
    const double A = 1 + (usq / 16384) * (4096 + usq * (-768 + usq * (320 - 175 * usq)));
    const double B = (usq / 1024) * (256 + usq * (-128 + usq * (74 - 47 * usq)));
    const double delta_sig = B * sin_sigma * (cos2sigma + 0.25 * B * (cos_sigma * (-1 + 2 * pow(cos2sigma, 2.)) - (1. / 6) * B * cos2sigma * (-3 + 4 * pow(sin_sigma, 2.)) * (-3 + 4 * pow(cos2sigma, 2.))));
    const double dis = rpol * A * (sigma - delta_sig);

    return static_cast<float>(dis);
}




float get_distance(double latitude1, double longitude1, double latitude2, double longitude2) {
    if (DISTANCE_FUNCTION_TO_USE == "vincenty") 
        return get_distance_vincenty(latitude1, longitude1, latitude2, longitude2);

    else if (DISTANCE_FUNCTION_TO_USE == "haversine")
        return get_distance_haversine(latitude1, longitude1, latitude2, longitude2);

    else 
        throw std::runtime_error("DISTANCE_FUNCTION_TO_USE must be either vincenty or haversine");
}




/**
 * Calculates the initial azimuth (bearing) from point 1 to point 2.
 * 
 * @param latitude1 Latitude of point 1 in degrees.
 * @param longitude1 Longitude of point 1 in degrees.
 * @param latitude2 Latitude of point 2 in degrees.
 * @param longitude2 Longitude of point 2 in degrees.
 * @return Azimuth in degrees, clockwise from North (0 to 360).
 */
float calculate_bearing(double latitude1, double longitude1, double latitude2, double longitude2) {
   
    // Convert degrees to radians
    double lat1 = (latitude1 * M_PI / 180.0);
    double lon1 = (longitude1 * M_PI / 180.0);
    double lat2 = (latitude2 * M_PI / 180.0);
    double lon2 = (longitude2 * M_PI / 180.0);

    const double deltaLon = lon2 - lon1;

    // Calculate the components for atan2
    const double y = std::sin(deltaLon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(deltaLon);

    // Use atan2(y, x) to get the angle in radians (range -PI to +PI)
    double azimuthRad = std::atan2(y, x);

    // Convert result to degrees
    double azimuthDeg = (azimuthRad * 180.0 / M_PI);

    // Normalize to 0-360 degrees (azimuth is typically measured clockwise from North)
    if (azimuthDeg < 0) {
        azimuthDeg += 360.0;
    }

    return static_cast<float>(azimuthDeg);
}



// LLA, ECEF, and NED transformation functions are adapted from: https://github.com/NavPy/NavPy/blob/master/navpy/core/navpy.py

// -------------------------
// LLA to ECEF
// -------------------------
std::array<double, 3> lla2ecef(double lat_deg, double lon_deg, double alt_m)
{
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;

    double sinLat = std::sin(lat);
    double cosLat = std::cos(lat);
    double cosLon = std::cos(lon);
    double sinLon = std::sin(lon);

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

    double x = (N + alt_m) * cosLat * cosLon;
    double y = (N + alt_m) * cosLat * sinLon;
    double z = (N * (1 - WGS84_E2) + alt_m) * sinLat;

    return {x, y, z};
}



// -------------------------
// ECEF to LLA (iterative)
// -------------------------
std::array<double, 3> ecef2lla(double x, double y, double z)
{
    double lon = std::atan2(y, x);

    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1 - WGS84_E2));

    double h = 0;
    double lat_prev;

    // Iterate like Python code
    for(int i=0; i<100; i++)
    {
        lat_prev = lat;
        double sinLat = std::sin(lat);
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

        if (std::abs(M_PI/2 - std::abs(lat)) > 1e-3)
            h = p / std::cos(lat) - N;
        else
            h = z / sinLat - N * (1 - WGS84_E2);

        lat = std::atan2(z + WGS84_E2 * N * sinLat, p);

        if (std::abs(lat - lat_prev) < 1e-12)
            break;
    }

    return {
        lat * 180.0 / M_PI,
        lon * 180.0 / M_PI,
        h
    };
}



// Build ECEF to NED rotation matrix
std::array<std::array<double, 3>, 3> nedRotation(double lat_deg, double lon_deg)
{
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;

    double sinLat = std::sin(lat), cosLat = std::cos(lat);
    double sinLon = std::sin(lon), cosLon = std::cos(lon);

    return {{
        {-sinLat*cosLon,  -sinLat*sinLon, cosLat},
        {-sinLon,         cosLon,         0.0},
        {-cosLat*cosLon,  -cosLat*sinLon, -sinLat}
    }};
}



// multiply matrix * vector (double version)
std::array<double, 3> matmul(const std::array<std::array<double,3>,3> &C, const std::array<double,3> &v) {
    return {
        C[0][0]*v[0] + C[0][1]*v[1] + C[0][2]*v[2],
        C[1][0]*v[0] + C[1][1]*v[1] + C[1][2]*v[2],
        C[2][0]*v[0] + C[2][1]*v[1] + C[2][2]*v[2]
    };
}



// -------------------------
// ECEF to NED
// -------------------------
std::array<float, 3> ecef2ned(const std::array<double,3> &ecef_vector, double reference_latitude, double reference_longitude) {
    std::array<std::array<double, 3>, 3> C = nedRotation(reference_latitude, reference_longitude);
    std::array<double, 3> result_double_precision = matmul(C, ecef_vector);
    std::array<float, 3> result_single_precision;
    std::copy(std::begin(result_double_precision), std::end(result_double_precision), std::begin(result_single_precision));

    return result_single_precision;
}



// -------------------------
// NED to ECEF
// -------------------------
std::array<double, 3> ned2ecef(const std::array<float,3> &ned_vector, double reference_latitude, double reference_longitude) {
    std::array<std::array<double, 3>, 3> C = nedRotation(reference_latitude, reference_longitude);

    // Use transpose
    std::array<std::array<double, 3>, 3> Ct {{
        {C[0][0], C[1][0], C[2][0]},
        {C[0][1], C[1][1], C[2][1]},
        {C[0][2], C[1][2], C[2][2]}
    }};

    std::array<double, 3> ned_double = {
        static_cast<double>(ned_vector[0]),
        static_cast<double>(ned_vector[1]),
        static_cast<double>(ned_vector[2])
    };

    return matmul(Ct, ned_double);
}



// -------------------------
// LLA to NED
// -------------------------
std::array<float, 3> lla2ned(double latitude, double longitude, double altitude, double reference_latitude, double reference_longitude, double reference_altitude) {
    std::array<double, 3> e = lla2ecef(latitude, longitude, altitude);
    std::array<double, 3> e0 = lla2ecef(reference_latitude, reference_longitude, reference_altitude);

    return ecef2ned({e[0]-e0[0], e[1]-e0[1], e[2]-e0[2]}, reference_latitude, reference_longitude);
}



// -------------------------
// NED to LLA
// -------------------------
std::array<double, 3> ned2lla(const std::array<float,3> &ned_vector, double reference_latitude, double reference_longitude, double reference_altitude) {
    std::array<double, 3> e0 = lla2ecef(reference_latitude, reference_longitude, reference_altitude);
    std::array<double, 3> e_rel = ned2ecef(ned_vector, reference_latitude, reference_longitude);

    std::array<double, 3> ecef = {
        e0[0] + e_rel[0],
        e0[1] + e_rel[1],
        e0[2] + e_rel[2]
    };

    return ecef2lla(ecef[0], ecef[1], ecef[2]);
}
