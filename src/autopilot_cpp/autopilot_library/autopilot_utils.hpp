#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "geographic_function_library.hpp"
#include "position.hpp"


using json = nlohmann::json;




enum class SailboatAutopilotModes {
    Disabled = 0, 
    Full_RC = 1, 
    Hold_Best_Sail = 2, 
    Hold_Heading = 3,
    Hold_Heading_And_Best_Sail = 4, 
    Waypoint_Mission = 5
};

enum class SailboatStates {
    NORMAL = 0, 
    CW_TACKING = 1, 
    CCW_TACKING = 2, 
    STALL = 3
};

enum class SailboatManeuvers {
    AUTOPILOT_DISABLED = 0, 
    STANDARD = 1, 
    TACK = 2, 
    JIBE = 3
};

enum class MotorboatAutopilotMode {
    Disabled = 0, 
    Full_RC = 1, 
    Hold_Heading = 2, 
    Waypoint_Mission = 3
};

enum class MotorboatControls {
    RPM = 0, 
    DUTY_CYCLE = 1, 
    CURRENT = 2
};


std::string to_string(MotorboatAutopilotMode mode);
std::string to_string(SailboatAutopilotModes mode);
std::string to_string(SailboatStates state);


/**
 * Checks if two floats are within 0.01 of each other.
 */
bool check_float_equivalence(float f1, float f2);

/**
 * Converts cartesian (x, y) to polar (magnitude, direction in degrees CCW from East).
 */
std::pair<float, float> cartesian_vector_to_polar(float x, float y);

/**
 * Returns the smallest angle (degrees) between two 2D vectors.
 */
float get_angle_between_vectors(const std::array<float, 2>& v1, const std::array<float, 2>& v2);

/**
 * Computes the shortest angular distance between two angles in degrees.
 */
float get_distance_between_angles(float angle1, float angle2);

/**
 * Gets bearing from current position to destination.
 * Returns angle 0-360 CCW from East to match Python logic.
 */
float get_bearing(const Position& current, const Position& dest);

/**
 * Distance between positions in meters using Haversine or Vincenty (based on library config).
 */
float get_distance_between_positions(const Position& p1, const Position& p2);

/**
 * Checks if 'angle' is between boundary1 and boundary2 using vector summation logic.
 */
bool is_angle_between_boundaries(float angle, float b1, float b2);




// --- Partially Tested Util Functions ---

/**
 * Checks if a direct line to destination enters the "No Sail Zone" (upwind).
 */
bool does_line_violate_no_sail_zone(
    const std::array<float, 2>& current,
    const std::array<float, 2>& dest,
    float global_true_wind_angle,
    float no_sail_zone_size
);

/**
 * Line segment vs Circle collision detection.
 */
bool does_line_segment_intersect_circle(
    const std::array<float, 2>& start,
    const std::array<float, 2>& end,
    const std::array<float, 2>& circle_pos,
    float radius
);



/** 
 * Helper function to convert YAML nodes to JSON objects
*/
json yaml_to_json(const YAML::Node& node);
