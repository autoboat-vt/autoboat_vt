#pragma once

#include <vector>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

#include "discrete_pid.hpp"
#include "autopilot_utils.hpp"
#include "position.hpp"


using json = nlohmann::json;


/*
This class contains the main control algorithms to control a motorboat given sensor data.
As the person using this class, you generally should not have to worry about any of the 
functions that start with an underscore such as _function_name.
These functions are meant to be private to the class and are essentially helper functions. 

This class is mainly used by the Motorboat Autopilot Node to control the boat through a ROS topic
*/
class MotorboatAutopilot {

private:
    
    DiscretePID rudder_angle_to_heading_pid_controller;
    
    std::map<std::string, json> *autopilot_parameters;

    std::vector<Position> waypoints;
    int current_waypoint_index = 0;


public:

    MotorboatAutopilot();
    MotorboatAutopilot(std::map<std::string, json> *autopilot_parameters_);

    int get_current_waypoint_index() const;
    std::vector<Position> get_current_waypoints_list() const;
    void reset();
    void update_waypoints_list(const std::vector<Position>& waypoints_list);
    float get_optimal_rudder_angle(float heading, float target_heading);
    std::pair<float, float> run_rc_control(float joystick_left_y, float joystick_right_x);
    float get_optimal_rpm(float rudder_angle);
    std::pair<float, float> run_waypoint_mission_step(Position current_position, float heading);
};