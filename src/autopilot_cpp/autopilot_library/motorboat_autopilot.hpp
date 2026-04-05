#pragma once

#include <vector>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

#include "discrete_pid.hpp"
#include "autopilot_utils.hpp"
#include "position.hpp"


using json = nlohmann::json;


/**
 * @class MotorboatAutopilot
 * @brief Logic for controlling a motorboat based on sensor data.
 * 
 * Provides methods for waypoint following, PID-based heading control, 
 * and RPM optimization.
 */
class MotorboatAutopilot {

private:
    
    DiscretePID rudder_angle_to_heading_pid_controller;
    
    std::map<std::string, json> *autopilot_parameters;

    std::vector<Position> waypoints;
    int current_waypoint_index = 0;


public:

    /**
     * @brief Construct a new Motorboat Autopilot object (default).
     */
    MotorboatAutopilot();

    /**
     * @brief Construct a new Motorboat Autopilot object with parameters.
     * @param autopilot_parameters_ Pointer to a map of autopilot configuration parameters.
     */
    MotorboatAutopilot(std::map<std::string, json> *autopilot_parameters_);

    /**
     * @brief Get the current waypoint index.
     * @return int The index of the waypoint currently being targeted.
     */
    int get_current_waypoint_index() const;

    /**
     * @brief Get the current waypoints list.
     * @return std::vector<Position> The list of waypoints.
     */
    std::vector<Position> get_current_waypoints_list() const;

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
     * @brief Gets the optimal rudder angle using a PID controller.
     * @param heading The current heading of the boat in degrees.
     * @param target_heading The desired heading of the boat in degrees.
     * @return float The rudder angle to reach the target heading.
     */
    float get_optimal_rudder_angle(float heading, float target_heading);

    /**
     * @brief Handles manual RC control inputs.
     * @param joystick_left_y Throttle input.
     * @param joystick_right_x Steering input.
     * @return std::pair<float, float> Pair of (rudder_angle, rpm).
     */
    std::pair<float, float> run_rc_control(float joystick_left_y, float joystick_right_x);

    /**
     * @brief Gets the optimal motor RPM based on current rudder angle.
     * @param rudder_angle The current rudder angle.
     * @return float The suggested motor RPM.
     */
    float get_optimal_rpm(float rudder_angle);

    /**
     * @brief Runs a single step of the waypoint mission algorithm.
     * @param current_position The boat's current position.
     * @param heading The boat's current heading in degrees.
     * @return std::pair<float, float> Pair of (rpm, rudder_angle).
     */
    std::pair<float, float> run_waypoint_mission_step(Position current_position, float heading);
};