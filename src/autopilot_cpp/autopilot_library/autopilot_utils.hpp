#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <algorithm>

// Include your previously defined geographic library
#include "geographic_function_library.hpp"

// --- Enums ---

enum class SailboatAutopilotMode {
    Disabled = 0, Full_RC = 1, Hold_Best_Sail = 2, Hold_Heading = 3,
    Hold_Heading_And_Best_Sail = 4, Waypoint_Mission = 5
};

enum class SailboatStates {
    NORMAL = 0, CW_TACKING = 1, CCW_TACKING = 2, STALL = 3
};

enum class SailboatManeuvers {
    AUTOPILOT_DISABLED = 0, STANDARD = 1, TACK = 2, JIBE = 3
};

enum class MotorboatAutopilotMode {
    Disabled = 0, Full_RC = 1, Hold_Heading = 2, Waypoint_Mission = 3
};

enum class MotorboatControls {
    RPM = 0, DUTY_CYCLE = 1, CURRENT = 2
};

// --- Position Class Wrapper (to match your Python usage) ---

struct Position {
    double longitude;
    double latitude;

    Position(double lon = 0.0, double lat = 0.0) : longitude(lon), latitude(lat) {}
    
    std::pair<double, double> get_longitude_latitude() const {
        return {longitude, latitude};
    }
};

// --- Math & Utility Functions ---

/**
 * Checks if two floats are within 0.001 of each other.
 */
inline bool check_float_equivalence(double f1, double f2) {
    return std::abs(f1 - f2) <= 0.001;
}

/**
 * Converts cartesian (x, y) to polar (magnitude, direction in degrees CCW from East).
 */
inline std::pair<double, double> cartesian_vector_to_polar(double x, double y) {
    if (x == 0.0 && y == 0.0) return {0.0, 0.0};
    
    double magnitude = std::sqrt(x * x + y * y);
    double direction = std::atan2(y, x) * (180.0 / M_PI);
    
    // Normalize to [0, 360)
    direction = std::fmod(direction, 360.0);
    if (direction < 0) direction += 360.0;
    
    return {magnitude, direction};
}

/**
 * Returns the smallest angle (degrees) between two 2D vectors.
 */
inline double get_angle_between_vectors(const std::array<double, 2>& v1, const std::array<double, 2>& v2) {
    double mag1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
    double mag2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);
    
    double dot = v1[0]*v2[0] + v1[1]*v2[1];
    double cos_theta = std::clamp(dot / (mag1 * mag2), -1.0, 1.0);
    
    return std::acos(cos_theta) * (180.0 / M_PI);
}

/**
 * Computes the shortest angular distance between two angles in degrees.
 */
inline double get_distance_between_angles(double angle1, double angle2) {
    double diff = std::fmod(angle1 - angle2 + 180.0, 360.0);
    if (diff < 0) diff += 360.0;
    return -1.0 * (diff - 180.0);
}

/**
 * Gets bearing from current position to destination.
 * Returns angle 0-360 CCW from East to match Python logic.
 */
inline double get_bearing(const Position& current, const Position& dest) {
    // calculate_bearing returns clockwise from North (Azimuth)
    double azimuth = calculate_bearing(current.latitude, current.longitude, dest.latitude, dest.longitude);
    
    // Convert: North (0) -> 90, East (90) -> 0, etc.
    double bearing = std::fmod(-azimuth + 90.0, 360.0);
    if (bearing < 0) bearing += 360.0;
    return bearing;
}

/**
 * Distance between positions in meters using Haversine or Vincenty (based on library config).
 */
inline double get_distance_between_positions(const Position& p1, const Position& p2) {
    return get_distance(p1.latitude, p1.longitude, p2.latitude, p2.longitude);
}

/**
 * Checks if 'angle' is between boundary1 and boundary2 using vector summation logic.
 */
inline bool is_angle_between_boundaries(double angle, double b1, double b2) {
    double a_rad = angle * M_PI / 180.0;
    double b1_rad = b1 * M_PI / 180.0;
    double b2_rad = b2 * M_PI / 180.0;

    std::array<double, 2> v_a = {std::cos(a_rad), std::sin(a_rad)};
    std::array<double, 2> v_b1 = {std::cos(b1_rad), std::sin(b1_rad)};
    std::array<double, 2> v_b2 = {std::cos(b2_rad), std::sin(b2_rad)};

    return check_float_equivalence(
        get_angle_between_vectors(v_b1, v_a) + get_angle_between_vectors(v_a, v_b2),
        get_angle_between_vectors(v_b1, v_b2)
    );
}

// --- Partially Tested Util Functions ---

/**
 * Checks if a direct line to destination enters the "No Sail Zone" (upwind).
 */
inline bool does_line_violate_no_sail_zone(
    const std::array<double, 2>& current,
    const std::array<double, 2>& dest,
    double global_true_wind_angle,
    double no_sail_zone_size
) {
    double dx = dest[0] - current[0];
    double dy = dest[1] - current[1];
    double dist = std::sqrt(dx*dx + dy*dy);
    
    if (dist < 1e-6) return false;

    // Upwind angle is opposite to wind
    double upwind_angle = std::fmod(global_true_wind_angle + 180.0, 360.0);
    double upwind_rad = upwind_angle * M_PI / 180.0;
    
    std::array<double, 2> upwind_vec = {std::cos(upwind_rad), std::sin(upwind_rad)};
    std::array<double, 2> dir_vec = {dx / dist, dy / dist};

    double angle_diff = get_angle_between_vectors(upwind_vec, dir_vec);
    
    return (angle_diff < no_sail_zone_size);
}

/**
 * Line segment vs Circle collision detection.
 */
inline bool does_line_segment_intersect_circle(
    const std::array<double, 2>& start,
    const std::array<double, 2>& end,
    const std::array<double, 2>& circle_pos,
    double radius
) {
    double dx = end[0] - start[0];
    double dy = end[1] - start[1];
    
    double fx = start[0] - circle_pos[0];
    double fy = start[1] - circle_pos[1];

    double a = dx*dx + dy*dy;
    double b = 2 * (fx*dx + fy*dy);
    double c = (fx*fx + fy*fy) - radius*radius;

    double discriminant = b*b - 4*a*c;
    
    if (discriminant < 0) return false;

    discriminant = std::sqrt(discriminant);
    double t1 = (-b - discriminant) / (2 * a);
    double t2 = (-b + discriminant) / (2 * a);

    // Check if the intersection points are within the line segment [0, 1]
    if (t1 >= 0 && t1 <= 1) return true;
    if (t2 >= 0 && t2 <= 1) return true;

    return false;
}