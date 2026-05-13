#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "geographic_function_library.hpp"
#include "position.hpp"


using json = nlohmann::json;




/**
 * @enum SailboatAutopilotModes
 * @brief Operational modes for a sailboat autopilot.
 */
enum class SailboatAutopilotModes {
    Disabled = 0,                     ///< Autopilot is inactive.
    Full_RC = 1,                      ///< Remote control overrides all autonomous functions.
    Hold_Best_Sail = 2,               ///< Automatically adjust sails for optimal trim given current wind.
    Hold_Heading = 3,                 ///< Maintain a constant compass heading using the rudder.
    Hold_Heading_And_Best_Sail = 4,   ///< Maintain heading while also optimizing sail trim.
    Waypoint_Mission = 5              ///< Navigate through a sequence of waypoints autonomously.
};

/**
 * @enum SailboatStates
 * @brief Internal state machine states for sailboat navigation logic.
 */
enum class SailboatStates {
    NORMAL = 0,       ///< Standard sailing state.
    CW_TACKING = 1,   ///< Clockwise tacking maneuver in progress.
    CCW_TACKING = 2,  ///< Counter-clockwise tacking maneuver in progress.
    STALL = 3         ///< Boat has lost speed or is in irons.
};

/**
 * @enum SailboatManeuvers
 * @brief Specific sailing maneuvers that the autopilot can execute.
 */
enum class SailboatManeuvers {
    AUTOPILOT_DISABLED = 0, ///< No maneuver active.
    STANDARD = 1,           ///< Standard course keeping.
    TACK = 2,               ///< Executing a tack (turning through the wind).
    JIBE = 3                ///< Executing a jibe (turning with the wind).
};

/**
 * @enum MotorboatAutopilotMode
 * @brief Operational modes for a motorboat autopilot.
 */
enum class MotorboatAutopilotMode {
    Disabled = 0,          ///< Autopilot is inactive.
    Full_RC = 1,           ///< Remote control mode.
    Hold_Heading = 2,      ///< Maintain a constant compass heading.
    Waypoint_Mission = 3   ///< Navigate through a sequence of waypoints.
};

/**
 * @enum MotorboatControls
 * @brief Control primitives for motorboat propulsion.
 */
enum class MotorboatControls {
    RPM = 0,           ///< Control based on motor revolutions per minute.
    DUTY_CYCLE = 1,    ///< Control based on Pulse Width Modulation duty cycle (0-1.0).
    CURRENT = 2        ///< Control based on motor current in Amperes.
};


/**
 * @brief Converts a MotorboatAutopilotMode to its string representation.
 * @param mode The mode to convert.
 * @return A string describing the mode.
 */
std::string to_string(MotorboatAutopilotMode mode);

/**
 * @brief Converts a SailboatAutopilotModes to its string representation.
 * @param mode The mode to convert.
 * @return A string describing the mode.
 */
std::string to_string(SailboatAutopilotModes mode);

/**
 * @brief Converts a SailboatStates to its string representation.
 * @param state The state to convert.
 * @return A string describing the state.
 */
std::string to_string(SailboatStates state);


/**
 * @brief Checks if two floats are approximately equal (within 0.01).
 * @param f1 First float.
 * @param f2 Second float.
 * @return True if the difference is less than 0.01.
 */
bool check_float_equivalence(float f1, float f2);

/**
 * @brief Converts cartesian (x, y) coordinates to polar (magnitude, direction).
 * @param x The x-component of the vector.
 * @param y The y-component of the vector.
 * @return A pair containing (magnitude, direction in degrees CCW from East).
 */
std::pair<float, float> cartesian_vector_to_polar(float x, float y);

/**
 * @brief Returns the smallest angle between two 2D vectors.
 * @param v1 First vector {x, y}.
 * @param v2 Second vector {x, y}.
 * @return The angle between the vectors in degrees.
 */
float get_angle_between_vectors(const std::array<float, 2>& v1, const std::array<float, 2>& v2);

/**
 * @brief Computes the shortest angular distance between two angles.
 * @param angle1 First angle in degrees.
 * @param angle2 Second angle in degrees.
 * @return The shortest distance between the two angles in degrees.
 */
float get_distance_between_angles(float angle1, float angle2);

/**
 * @brief Gets the bearing from current position to a destination.
 * @param current The current position.
 * @param dest The destination position.
 * @return The bearing angle in degrees (0-360 CCW from East).
 */
float get_bearing(const Position& current, const Position& dest);

/**
 * @brief Calculates the distance between two positions in meters.
 * @param position1 First position.
 * @param position2 Second position.
 * @return The distance in meters.
 * @note Uses Haversine or Vincenty based on underlying library configuration.
 */
float get_distance_between_positions(const Position& position1, const Position& position2);

/**
 * @brief Checks if a given angle is between two boundaries using vector summation logic.
 * @param angle The angle to check in degrees.
 * @param boundary1 The start boundary angle in degrees.
 * @param boundary2 The end boundary angle in degrees.
 * @return True if the angle is within the specified sector.
 */
bool is_angle_between_boundaries(float angle, float boundary1, float boundary2);




// --- Partially Tested Util Functions ---

/**
 * @brief (Experimental) Checks if a direct line to destination enters the "No Sail Zone" (upwind).
 * @param current_waypoint Current position as {x, y}.
 * @param destination_waypoint Destination position as {x, y}.
 * @param global_true_wind_angle The true wind angle relative to the global frame.
 * @param no_sail_zone_size The angular width of the No Sail Zone in degrees.
 * @return True if the direct path is deemed un-sailable.
 */
bool does_line_violate_no_sail_zone(
    const std::array<float, 2>& current_waypoint,
    const std::array<float, 2>& destination_waypoint,
    float global_true_wind_angle,
    float no_sail_zone_size
);

/**
 * @brief (Experimental) Performs line segment vs Circle collision detection.
 * @param line_start Start point of the line segment {x, y}.
 * @param line_end End point of the line segment {x, y}.
 * @param circle_position Center position of the circle {x, y}.
 * @param circle_radius Radius of the circle.
 * @return True if the line segment intersects with the circle.
 */
bool does_line_segment_intersect_circle(
    const std::array<float, 2>& line_start,
    const std::array<float, 2>& line_end,
    const std::array<float, 2>& circle_position,
    float circle_radius
);