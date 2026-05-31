#pragma once


#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <chrono>

#include <nlohmann/json.hpp>

#include "discrete_pid.hpp"
#include "autopilot_utils.hpp"
#include "position.hpp"



using json = nlohmann::json;



/**
 * @class SailboatAutopilot
 * @brief Logic for controlling a sailboat based on sensor data.
 * 
 * Provides methods for waypoint following, PID-based heading control, 
 * sail angle optimization, and automated maneuvers like tacking and jibing.
 */
class SailboatAutopilot {


private:

    std::map<std::string, json> *autopilot_parameters;
    std::vector<Position> waypoints;
    int current_waypoint_index;
    
    float desired_tacking_angle = 0.0f;
    
    SailboatAutopilotStates current_state = SailboatAutopilotStates::DOWNWIND_SAILING;
    Position last_tacking_position = Position(0.0, 0.0);
    std::chrono::steady_clock::time_point last_time_out_of_no_sail_zone;

    float get_decision_zone_size(float distance_to_waypoint);
    SailboatManeuvers get_maneuver_from_desired_heading(float heading, float desired_heading, float true_wind_angle);
    std::pair<float, bool> apply_decision_zone_tacking_logic(
        float current_heading, float desired_heading, 
        float true_wind_angle, float apparent_wind_angle, float distance
    );
    std::pair<float, SailboatAutopilotStates> _apply_tacking_state_machine(
        float current_heading, float current_bearing,
        float true_wind_angle, float apparent_wind_angle,
        Position current_position, Position last_tack_position,
        SailboatAutopilotStates current_state
    );


public:

    /**
     * @brief PID controller for the rudder.
     */
    DiscretePID heading_pid_controller;



    /**
     * @brief Heading to hold in HOLD_HEADING mode.
     */
    float heading_to_hold;

    /**
     * @brief Construct a new Sailboat Autopilot object (default).
     */
    SailboatAutopilot();

    /**
     * @brief Construct a new Sailboat Autopilot object with parameters.
     * @param autopilot_parameters_ Pointer to a map of autopilot configuration parameters.
     */
    SailboatAutopilot(std::map<std::string, json> *autopilot_parameters_);

    /**
     * @brief Resets the autopilot to its initial state.
     */
    void reset();

    /**
     * @brief Updates the list of waypoints that the boat should follow.
     * @param waypoints_list A list of Position objects forming the path.
     */
    void update_waypoints_list(const std::vector<Position>& waypoints_list);

    /**
     * @brief Get the current waypoint index.
     * @return float The index of the waypoint currently being targeted.
     */
    float get_current_waypoint_index();

    /**
     * @brief Get the current waypoints list.
     * @return std::vector<Position> The list of waypoints.
     */
    std::vector<Position> get_current_waypoints_list();

    /**
     * @brief Get the current waypoint mission state.
     * @return SailboatAutopilotStates The current state.
     */
    SailboatAutopilotStates get_current_waypoint_mission_state();

    /**
     * @brief Gets what the autopilot should do in order to not crash into the object in front of it.
     * @param heading_object_was_detected_at Direction the boat was facing when it encountered the object.
     * @param current_heading Direction the boat is facing in degrees.
     * @param apparent_wind_angle Apparent wind angle in degrees.
     * @return std::pair<float, float> Pair of (sail_angle, rudder_angle).
     */
    std::pair<float, float> run_emergency_stop_step(float heading_object_was_detected_at, float current_heading, float apparent_wind_angle);

    /**
     * @brief Gets the optimal sail angle given the apparent wind angle.
     * @param apparent_wind_angle The apparent wind angle in degrees.
     * @return float The optimal sail angle (0 to 90).
     */
    float get_optimal_sail_angle(float apparent_wind_angle);

    /**
     * @brief Gets the optimal rudder angle using a PID controller.
     * @param heading The current heading of the boat in degrees.
     * @param desired_heading The desired heading of the boat in degrees.
     * @return float The suggested rudder angle.
     */
    float get_optimal_rudder_angle(float heading, float desired_heading);

    /**
     * @brief Converts joystick inputs into desired sail and rudder angles.
     * @param rc_rudder_control Steering input (-100 to 100).
     * @param rc_sail_control Sail input (-100 to 100).
     * @return std::pair<float, float> Pair of (sail_angle, rudder_angle).
     */
    std::pair<float, float> run_rc_control(float rc_rudder_control, float rc_sail_control);

    /**
     * @brief Runs a single step of the waypoint mission algorithm.
     * @param current_position The boat's current position.
     * @param global_velocity_vector The boat's velocity vector in m/s.
     * @param heading The boat's current heading in degrees.
     * @param apparent_wind_vector The apparent wind vector in m/s.
     * @return std::tuple<std::optional<float>, std::optional<float>, std::optional<float>> Tuple of (sail_angle, rudder_angle, desired_heading).
     */
    std::tuple<std::optional<float>, std::optional<float>, std::optional<float>> run_waypoint_mission_step(Position current_position, std::array<float, 2> global_velocity_vector, float heading, std::array<float, 2> apparent_wind_vector);
};