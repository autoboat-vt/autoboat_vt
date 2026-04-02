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

    DiscretePID rudder_pid_controller;
    SailboatAutopilotModes current_autopilot_mode = SailboatAutopilotModes::Waypoint_Mission;
    float heading_to_hold;

    // TODO: INITIALIZE AUTOPILOT PARAMETERS TO ZEROES HERE SO THIS ISN'T A USELESS CONSTRUCTOR
    SailboatAutopilot();
    SailboatAutopilot(std::map<std::string, json> *autopilot_parameters_);

    void reset();
    void update_waypoints_list(const std::vector<Position>& waypoints_list);
    float get_current_waypoint_index();
    std::vector<Position> get_current_waypoints_list();
    SailboatStates get_current_waypoint_mission_state();

    // Returns the {desired_rudder_angle, desired_sail_angle} of the autopilot at the current timestep
    // Returns {NAN, NAN} if the autopilot is disabled
    std::pair<float, float> step(
        Position current_position, 
        std::array<float, 2> current_global_velocity_vector, 
        float current_heading, 
        std::array<float, 2> current_apparent_wind_vector,

        float heading_to_hold = 0.0,
        float rc_rudder_control = 0.0,
        float rc_sail_control = 0.0
    );

    // TODO: I DON'T TRUST THIS
    float get_optimal_sail_angle(float apparent_wind_angle);
    float get_optimal_rudder_angle(float heading, float desired_heading);

    // rc rudder control and rc sail_control are both numbers from -100 to 100 that represent the state of the rc controller
    std::pair<float, float> run_rc_control(float rc_rudder_control, float rc_sail_control);
    std::pair<float, float> run_waypoint_mission_step(Position current_position, std::array<float, 2> global_velocity_vector, float heading, std::array<float, 2> apparent_wind_vector);
};