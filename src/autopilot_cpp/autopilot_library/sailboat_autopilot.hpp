#pragma once


#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <array>

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
    
    float desired_tacking_angle;
    
    SailboatStates current_waypoint_mission_state = SailboatStates::NORMAL;

    float get_decision_zone_size(float distance_to_waypoint);
    SailboatManeuvers get_maneuver_from_desired_heading(float heading, float desired_heading, float true_wind_angle);
    std::pair<float, bool> apply_decision_zone_tacking_logic(
        float current_heading, float desired_heading, 
        float true_wind_angle, float apparent_wind_angle, float distance
    );


public:

    /**
     * @brief PID controller for the rudder.
     */
    DiscretePID rudder_pid_controller;

    /**
     * @brief Current autopilot mode.
     */
    SailboatAutopilotModes current_autopilot_mode = SailboatAutopilotModes::Waypoint_Mission;

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
     * @return SailboatStates The current state (e.g., NORMAL, TACKING).
     */
    SailboatStates get_current_waypoint_mission_state();

    /**
     * @brief Computes the desired rudder and sail angles for the current timestep.
     * @param current_position The boat's current position.
     * @param current_global_velocity_vector The boat's velocity vector in m/s.
     * @param current_heading The boat's current heading in degrees.
     * @param current_apparent_wind_vector The apparent wind vector in m/s.
     * @param heading_to_hold Target heading for hold mode.
     * @param rc_rudder_control Manual rudder input (-100 to 100).
     * @param rc_sail_control Manual sail input (-100 to 100).
     * @return std::pair<float, float> Pair of (rudder_angle, sail_angle).
     */
    std::pair<float, float> step(
        Position current_position, 
        std::array<float, 2> current_global_velocity_vector, 
        float current_heading, 
        std::array<float, 2> current_apparent_wind_vector,

        float heading_to_hold = 0.0,
        float rc_rudder_control = 0.0,
        float rc_sail_control = 0.0
    );

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
     * @return std::pair<float, float> Pair of (sail_angle, rudder_angle).
     */
    std::pair<float, float> run_waypoint_mission_step(Position current_position, std::array<float, 2> global_velocity_vector, float heading, std::array<float, 2> apparent_wind_vector);
};