#include "geographic_function_library.hpp"


const std::string DISTANCE_FUNCTION_TO_USE = "haversine";


// haversine distance for distance between 2 points on a spheroid (approximation to an elipsoid)
// this is a less accurate distance function than vincenty since it assumes that the earth is a perfect sphere
// https://en.wikipedia.org/wiki/Haversine_formula
float get_distance_haversine(double latitude1, double longitude1, double latitude2, double longitude2) {
    // Haversine in meters
    const double earth_radius_meters = WGS84_A;
    double delta_latitude_radians = (latitude2 - latitude1) * M_PI / 180.0;
    double delta_longitude_radians = (longitude2 - longitude1) * M_PI / 180.0;
    double haversine_a = sin(delta_latitude_radians/2)*sin(delta_latitude_radians/2) + cos(latitude1*M_PI/180.0)*cos(latitude2*M_PI/180.0)*sin(delta_longitude_radians/2)*sin(delta_longitude_radians/2);
    double haversine_c = 2*atan2(sqrt(haversine_a), sqrt(1-haversine_a));
    return static_cast<float>(earth_radius_meters * haversine_c);
}



// Using vincentys formula to calculate the distance between 2 points on an elipsoid
// this is a more accurate distance function than haversine since it doesn't make the assumption that the earth is a perfect sphere
// https://github.com/dariusarnold/vincentys-formula
// https://en.wikipedia.org/wiki/Vincenty%27s_formulae 
float get_distance_vincenty(double latitude1, double longitude1, double latitude2, double longitude2) {
    using namespace std;

    constexpr double equatorial_radius_meters = WGS84_A;             // Radius at equator
    constexpr double flattening = WGS84_F;            // Flattening of earth
    constexpr double polar_radius_meters = (1 - flattening) * equatorial_radius_meters;

    double sine_of_sigma, cosine_of_sigma, sigma_radians, sine_of_alpha, cosine_squared_of_alpha, cosine_of_twice_sigma;
    double vincenty_c, previous_lambda_radians;

    // convert to radians
    double latitude_1_radians = M_PI * latitude1 / 180.0;
    double latitude_2_radians = M_PI * latitude2 / 180.0;
    double longitude_1_radians = M_PI * longitude1 / 180.0;
    double longitude_2_radians = M_PI * longitude2 / 180.0;

    const double reduced_latitude_1_radians = atan((1 - flattening) * tan(latitude_2_radians));
    const double reduced_latitude_2_radians = atan((1 - flattening) * tan(latitude_1_radians));

    double longitude_difference_radians = longitude_1_radians - longitude_2_radians;
    double lambda_radians = longitude_difference_radians;
    constexpr double iteration_tolerance = 10.e-12; // iteration tolerance
    double lambda_difference = 1.;

    while (abs(lambda_difference) > iteration_tolerance) {
        sine_of_sigma = sqrt(pow((cos(reduced_latitude_2_radians) * sin(lambda_radians)), 2.) + pow(cos(reduced_latitude_1_radians)*sin(reduced_latitude_2_radians) - sin(reduced_latitude_1_radians)*cos(reduced_latitude_2_radians)*cos(lambda_radians), 2.));
        
        if (sine_of_sigma == 0.) {
            // Coincident points, prevent division by zero resulting in NaN.
            return 0.0f;
        }
        
        cosine_of_sigma = sin(reduced_latitude_1_radians) * sin(reduced_latitude_2_radians) + cos(reduced_latitude_1_radians) * cos(reduced_latitude_2_radians) * cos(lambda_radians);
        sigma_radians = atan(sine_of_sigma / cosine_of_sigma);
        
        if (sigma_radians <= 0) sigma_radians = M_PI + sigma_radians;
        
        sine_of_alpha = (cos(reduced_latitude_1_radians) * cos(reduced_latitude_2_radians) * sin(lambda_radians)) / sine_of_sigma;
        cosine_squared_of_alpha = 1 - pow(sine_of_alpha, 2.);
        
        if (cosine_squared_of_alpha == 0.) {
            cosine_of_twice_sigma = 0.;
        } 
        else {
            cosine_of_twice_sigma = cosine_of_sigma - ((2 * sin(reduced_latitude_1_radians) * sin(reduced_latitude_2_radians)) / cosine_squared_of_alpha);
        }
        
        vincenty_c = (flattening / 16) * cosine_squared_of_alpha * (4 + flattening * (4 - 3 * cosine_squared_of_alpha));
        previous_lambda_radians = lambda_radians;
        lambda_radians = longitude_difference_radians + (1 - vincenty_c) * flattening * sine_of_alpha * (sigma_radians + vincenty_c * sine_of_sigma * (cosine_of_twice_sigma + vincenty_c * cosine_of_sigma * (2 * pow(cosine_of_twice_sigma, 2.) - 1)));
        lambda_difference = abs(previous_lambda_radians - lambda_radians);
    }

    const double u_squared = cosine_squared_of_alpha * ((pow(equatorial_radius_meters, 2.) - pow(polar_radius_meters, 2.)) / pow(polar_radius_meters ,2.));
    const double vincenty_a = 1 + (u_squared / 16384) * (4096 + u_squared * (-768 + u_squared * (320 - 175 * u_squared)));
    const double vincenty_b = (u_squared / 1024) * (256 + u_squared * (-128 + u_squared * (74 - 47 * u_squared)));
    const double delta_sigma = vincenty_b * sine_of_sigma * (cosine_of_twice_sigma + 0.25 * vincenty_b * (cosine_of_sigma * (-1 + 2 * pow(cosine_of_twice_sigma, 2.)) - (1. / 6) * vincenty_b * cosine_of_twice_sigma * (-3 + 4 * pow(sine_of_sigma, 2.)) * (-3 + 4 * pow(cosine_of_twice_sigma, 2.))));
    const double distance_meters = polar_radius_meters * vincenty_a * (sigma_radians - delta_sigma);

    return static_cast<float>(distance_meters);
}




float get_distance(double latitude1, double longitude1, double latitude2, double longitude2) {
    if (DISTANCE_FUNCTION_TO_USE == "vincenty") 
        return get_distance_vincenty(latitude1, longitude1, latitude2, longitude2);

    else if (DISTANCE_FUNCTION_TO_USE == "haversine")
        return get_distance_haversine(latitude1, longitude1, latitude2, longitude2);

    else 
        throw std::runtime_error("DISTANCE_FUNCTION_TO_USE must be either vincenty or haversine");
}





float calculate_bearing(double latitude1, double longitude1, double latitude2, double longitude2) {
   
    // Convert degrees to radians
    double latitude_1_radians = (latitude1 * M_PI / 180.0);
    double longitude_1_radians = (longitude1 * M_PI / 180.0);
    double latitude_2_radians = (latitude2 * M_PI / 180.0);
    double longitude_2_radians = (longitude2 * M_PI / 180.0);

    const double delta_longitude_radians = longitude_2_radians - longitude_1_radians;

    // Calculate the components for atan2
    const double y_component = std::sin(delta_longitude_radians) * std::cos(latitude_2_radians);
    const double x_component = std::cos(latitude_1_radians) * std::sin(latitude_2_radians) - std::sin(latitude_1_radians) * std::cos(latitude_2_radians) * std::cos(delta_longitude_radians);

    // Use atan2(y_component, x_component) to get the angle in radians (range -PI to +PI)
    double azimuth_in_radians = std::atan2(y_component, x_component);

    // Convert result to degrees
    double azimuth_in_degrees = (azimuth_in_radians * 180.0 / M_PI);

    // Normalize to 0-360 degrees (azimuth is typically measured clockwise from North)
    if (azimuth_in_degrees < 0) {
        azimuth_in_degrees += 360.0;
    }

    return static_cast<float>(azimuth_in_degrees);
}



// LLA, ECEF, and NED transformation functions are adapted from: https://github.com/NavPy/NavPy/blob/master/navpy/core/navpy.py

// -------------------------
// LLA to ECEF
// -------------------------
std::array<double, 3> lla2ecef(double latitude, double longitude, double altitude)
{
    double latitude_radians = latitude * M_PI / 180.0;
    double longitude_radians = longitude * M_PI / 180.0;

    double sine_of_latitude = std::sin(latitude_radians);
    double cosine_of_latitude = std::cos(latitude_radians);
    double cosine_of_longitude = std::cos(longitude_radians);
    double sine_of_longitude = std::sin(longitude_radians);

    double prime_vertical_radius_of_curvature = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sine_of_latitude * sine_of_latitude);

    double x_coordinate = (prime_vertical_radius_of_curvature + altitude) * cosine_of_latitude * cosine_of_longitude;
    double y_coordinate = (prime_vertical_radius_of_curvature + altitude) * cosine_of_latitude * sine_of_longitude;
    double z_coordinate = (prime_vertical_radius_of_curvature * (1 - WGS84_E2) + altitude) * sine_of_latitude;

    return {x_coordinate, y_coordinate, z_coordinate};
}



// -------------------------
// ECEF to LLA (iterative)
// -------------------------
std::array<double, 3> ecef2lla(double x_coordinate, double y_coordinate, double z_coordinate)
{
    double longitude_radians = std::atan2(y_coordinate, x_coordinate);

    double perpendicular_distance_from_z_axis = std::sqrt(x_coordinate*x_coordinate + y_coordinate*y_coordinate);
    double latitude_radians = std::atan2(z_coordinate, perpendicular_distance_from_z_axis * (1 - WGS84_E2));

    double altitude = 0;
    double previous_latitude_radians;

    // Iterate like Python code
    for(int i=0; i<100; i++)
    {
        previous_latitude_radians = latitude_radians;
        double sine_of_latitude = std::sin(latitude_radians);
        double prime_vertical_radius_of_curvature = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sine_of_latitude * sine_of_latitude);

        if (std::abs(M_PI/2 - std::abs(latitude_radians)) > 1e-3)
            altitude = perpendicular_distance_from_z_axis / std::cos(latitude_radians) - prime_vertical_radius_of_curvature;
        else
            altitude = z_coordinate / sine_of_latitude - prime_vertical_radius_of_curvature * (1 - WGS84_E2);

        latitude_radians = std::atan2(z_coordinate + WGS84_E2 * prime_vertical_radius_of_curvature * sine_of_latitude, perpendicular_distance_from_z_axis);

        if (std::abs(latitude_radians - previous_latitude_radians) < 1e-12)
            break;
    }

    return {
        latitude_radians * 180.0 / M_PI,
        longitude_radians * 180.0 / M_PI,
        altitude
    };
}



// Build ECEF to NED rotation matrix
std::array<std::array<double, 3>, 3> nedRotation(double latitude, double longitude)
{
    double latitude_radians = latitude * M_PI / 180.0;
    double longitude_radians = longitude * M_PI / 180.0;

    double sine_of_latitude = std::sin(latitude_radians), cosine_of_latitude = std::cos(latitude_radians);
    double sine_of_longitude = std::sin(longitude_radians), cosine_of_longitude = std::cos(longitude_radians);

    return {{
        {-sine_of_latitude*cosine_of_longitude,  -sine_of_latitude*sine_of_longitude, cosine_of_latitude},
        {-sine_of_longitude,         cosine_of_longitude,         0.0},
        {-cosine_of_latitude*cosine_of_longitude,  -cosine_of_latitude*sine_of_longitude, -sine_of_latitude}
    }};
}



// multiply matrix * vector (double version)
std::array<double, 3> matmul(const std::array<std::array<double,3>,3> &rotation_matrix, const std::array<double,3> &input_vector) {
    return {
        rotation_matrix[0][0]*input_vector[0] + rotation_matrix[0][1]*input_vector[1] + rotation_matrix[0][2]*input_vector[2],
        rotation_matrix[1][0]*input_vector[0] + rotation_matrix[1][1]*input_vector[1] + rotation_matrix[1][2]*input_vector[2],
        rotation_matrix[2][0]*input_vector[0] + rotation_matrix[2][1]*input_vector[1] + rotation_matrix[2][2]*input_vector[2]
    };
}



// -------------------------
// ECEF to NED
// -------------------------
std::array<float, 3> ecef2ned(const std::array<double,3> &ecef_vector, double reference_latitude, double reference_longitude) {
    std::array<std::array<double, 3>, 3> rotation_matrix = nedRotation(reference_latitude, reference_longitude);
    std::array<double, 3> result_double_precision = matmul(rotation_matrix, ecef_vector);
    std::array<float, 3> result_single_precision;
    std::copy(std::begin(result_double_precision), std::end(result_double_precision), std::begin(result_single_precision));

    return result_single_precision;
}



// -------------------------
// NED to ECEF
// -------------------------
std::array<double, 3> ned2ecef(const std::array<float,3> &ned_vector, double reference_latitude, double reference_longitude) {
    std::array<std::array<double, 3>, 3> rotation_matrix = nedRotation(reference_latitude, reference_longitude);

    // Use transpose
    std::array<std::array<double, 3>, 3> rotation_matrix_transpose {{
        {rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0]},
        {rotation_matrix[0][1], rotation_matrix[1][1], rotation_matrix[2][1]},
        {rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2]}
    }};

    std::array<double, 3> ned_vector_double = {
        static_cast<double>(ned_vector[0]),
        static_cast<double>(ned_vector[1]),
        static_cast<double>(ned_vector[2])
    };

    return matmul(rotation_matrix_transpose, ned_vector_double);
}



// -------------------------
// LLA to NED
// -------------------------
std::array<float, 3> lla2ned(double latitude, double longitude, double altitude, double reference_latitude, double reference_longitude, double reference_altitude) {
    std::array<double, 3> ecef_target = lla2ecef(latitude, longitude, altitude);
    std::array<double, 3> ecef_reference = lla2ecef(reference_latitude, reference_longitude, reference_altitude);

    return ecef2ned({ecef_target[0]-ecef_reference[0], ecef_target[1]-ecef_reference[1], ecef_target[2]-ecef_reference[2]}, reference_latitude, reference_longitude);
}



// -------------------------
// NED to LLA
// -------------------------
std::array<double, 3> ned2lla(const std::array<float,3> &ned_vector, double reference_latitude, double reference_longitude, double reference_altitude) {
    std::array<double, 3> ecef_reference = lla2ecef(reference_latitude, reference_longitude, reference_altitude);
    std::array<double, 3> ecef_relative = ned2ecef(ned_vector, reference_latitude, reference_longitude);

    std::array<double, 3> ecef_target = {
        ecef_reference[0] + ecef_relative[0],
        ecef_reference[1] + ecef_relative[1],
        ecef_reference[2] + ecef_relative[2]
    };

    return ecef2lla(ecef_target[0], ecef_target[1], ecef_target[2]);
}
