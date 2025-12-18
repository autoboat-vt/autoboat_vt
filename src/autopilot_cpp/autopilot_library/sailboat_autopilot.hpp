#pragma once


#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <array>


#include "discrete_pid.hpp"
#include "autopilot_utils.hpp"
#include "position.hpp"



using json = nlohmann::json;



class SailboatAutopilot {

public:

    std::map<std::string, json> autopilot_parameters;
    std::vector<Position> waypoints;
    int current_waypoint_index = 0;
    SailboatStates current_state = SailboatStates::NORMAL;
    float desired_tacking_angle = 0.0;
    DiscretePID rudder_pid_controller;


    SailboatAutopilot() {}


    SailboatAutopilot(std::map<std::string, json> autopilot_parameters_) {

        autopilot_parameters = autopilot_parameters_;

        rudder_pid_controller = DiscretePID(
            1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>(),
            autopilot_parameters["heading_p_gain"].get<float>(),
            autopilot_parameters["heading_i_gain"].get<float>(),
            autopilot_parameters["heading_d_gain"].get<float>(),
            autopilot_parameters["heading_n_gain"].get<float>()
        );
    }


    void reset() {
        waypoints.clear();
        current_waypoint_index = 0;
        current_state = SailboatStates::NORMAL;
        desired_tacking_angle = 0.0;
        rudder_pid_controller.reset();
    }


    void update_waypoints_list(const std::vector<Position>& waypoints_list) {
        waypoints = waypoints_list;
        current_waypoint_index = 0;
    }


    // TODO: I DON'T TRUST THIS
    float get_optimal_sail_angle(float apparent_wind_angle) {

        std::vector<float> wind_angles = autopilot_parameters["sail_lookup_table_wind_angles"].get<std::vector<float>>();
        std::vector<float> sail_angles = autopilot_parameters["sail_lookup_table_sail_positions"].get<std::vector<float>>();

        float abs_awa = std::abs(get_distance_between_angles(apparent_wind_angle, 0.0));
        
        if (abs_awa <= wind_angles.front()) {
            return sail_angles.front();
        }

        if (abs_awa >= wind_angles.back()) {
            return sail_angles.back();
        }


        for (size_t i = 0; i < wind_angles.size() - 1; ++i) {
            if (abs_awa >= wind_angles[i] && abs_awa <= wind_angles[i+1]) {
                float t = (abs_awa - wind_angles[i]) / (wind_angles[i+1] - wind_angles[i]);
                return sail_angles[i] + t * (sail_angles[i+1] - sail_angles[i]);
            }
        }

        return autopilot_parameters["min_sail_angle"].get<float>();
    }

    
    float get_optimal_rudder_angle(float heading, float desired_heading) {
        rudder_pid_controller.set_gains(
            1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>(),
            autopilot_parameters["heading_p_gain"].get<float>(),
            autopilot_parameters["heading_i_gain"].get<float>(),
            autopilot_parameters["heading_d_gain"].get<float>(),
            autopilot_parameters["heading_n_gain"].get<float>()
        );

        float error = get_distance_between_angles(desired_heading, heading);
        float rudder_angle = rudder_pid_controller.step(error);
        
        return std::clamp((float)rudder_angle, autopilot_parameters["min_rudder_angle"].get<float>(), autopilot_parameters["max_rudder_angle"].get<float>());
    }


    std::pair<float, float> run_rc_control(float joystick_left_y, float joystick_right_x) {

        // Formula: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        auto map_range = [](float value, float out_min, float out_max) {
            return (value - (-100.0)) * (out_max - out_min) / (100.0 - (-100.0)) + out_min;
        };

        float sail_angle = map_range(joystick_left_y, autopilot_parameters["min_sail_angle"].get<float>(), autopilot_parameters["max_sail_angle"].get<float>());
        float rudder_angle = map_range(joystick_right_x, autopilot_parameters["min_rudder_angle"].get<float>(), autopilot_parameters["max_rudder_angle"].get<float>());

        return {sail_angle, rudder_angle};
    }

    
    
    std::pair<float, float> run_waypoint_mission_step(Position current_position, std::array<float, 2> global_velocity_vector, float heading, std::array<float, 2> apparent_wind_vector) {
        
        if (waypoints.empty()) {
            return {0.0, 0.0};
        }


        // Calculate Vectors
        float boat_speed = std::sqrt(std::pow(global_velocity_vector[0], 2) + std::pow(global_velocity_vector[1], 2));
        auto [_, global_velocity_angle] = cartesian_vector_to_polar(global_velocity_vector[0], global_velocity_vector[1]);
        
        float local_velocity_angle = (global_velocity_angle - heading) * M_PI / 180.0;
        std::array<float, 2> local_velocity_vector = { boat_speed * std::cos(local_velocity_angle), boat_speed * std::sin(local_velocity_angle) };

        std::array<float, 2> true_wind_vector = { apparent_wind_vector[0] + local_velocity_vector[0], apparent_wind_vector[1] + local_velocity_vector[1] };
        auto [true_wind_speed, true_wind_angle] = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1]);
        auto [apparent_wind_speed, apparent_wind_angle] = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1]);

        float global_true_wind_angle = std::fmod(true_wind_angle + heading, 360.0);


        if (global_true_wind_angle < 0) {
            global_true_wind_angle += 360.0;
        }


        // Waypoint Management
        Position current_waypoint = waypoints[current_waypoint_index];
        float distance_to_waypoint = get_distance_between_positions(current_position, current_waypoint);

        if (distance_to_waypoint < autopilot_parameters["waypoint_accuracy"].get<float>()) {
            if (current_waypoint_index + 1 < waypoints.size()) {
                current_waypoint_index++;
            } 
            
            else {
                return {autopilot_parameters["max_sail_angle"].get<float>(), 0.0};
            }
        }
        


        float desired_sail_angle = 0.0;
        float desired_rudder_angle = 0.0;


        // State Machine
        if (current_state == SailboatStates::NORMAL) {
            float desired_heading = get_bearing(current_position, current_waypoint);
            auto [decision_zone_heading, should_tack_decision_zone] = apply_decision_zone_tacking_logic(heading, desired_heading, true_wind_angle, apparent_wind_angle, distance_to_waypoint);
            
            float global_true_upwind_angle = std::fmod(global_true_wind_angle + 180.0, 360.0);
            bool should_tack_simple = is_angle_between_boundaries(global_true_upwind_angle, heading, desired_heading);

            if (should_tack_decision_zone || should_tack_simple) {
                desired_tacking_angle = decision_zone_heading;
                float rudder_test = get_optimal_rudder_angle(heading, decision_zone_heading);
                current_state = (rudder_test > 0) ? SailboatStates::CW_TACKING : SailboatStates::CCW_TACKING;
            }

            desired_rudder_angle = get_optimal_rudder_angle(heading, decision_zone_heading);
            desired_sail_angle = get_optimal_sail_angle(apparent_wind_angle);
        } 

        else if (current_state == SailboatStates::CW_TACKING || current_state == SailboatStates::CCW_TACKING) {
            desired_sail_angle = get_optimal_sail_angle(apparent_wind_angle);
            float tack_direction = (current_state == SailboatStates::CW_TACKING) ? 1.0 : -1.0;
            desired_rudder_angle = autopilot_parameters["rudder_hard_over"].get<float>() * tack_direction;

            if (autopilot_parameters["perform_forced_jibe_instead_of_tack"].get<float>() > 0) {
                desired_rudder_angle *= -1.0;
            }

            if (std::abs(get_distance_between_angles(heading, desired_tacking_angle)) < autopilot_parameters["tack_tolerance"].get<float>()) {
                current_state = SailboatStates::NORMAL;
            }
        }

        return {desired_sail_angle, desired_rudder_angle};
    }






private:

    float get_decision_zone_size(float distance_to_waypoint) {
        float tack_dist = autopilot_parameters["tack_distance"].get<float>();
        float nsz_half = autopilot_parameters["no_sail_zone_size"].get<float>() / 2.0;
        float arg = (tack_dist / distance_to_waypoint) * std::sin(nsz_half * M_PI / 180.0);
        arg = std::clamp(arg, (float) -1.0, (float) 1.0);
        float decision_zone_size = std::asin(arg) * 180.0 / M_PI;

        return std::clamp(decision_zone_size, (float) 0.0, autopilot_parameters["no_sail_zone_size"].get<float>());
    }

    SailboatManeuvers get_maneuver_from_desired_heading(float heading, float desired_heading, float true_wind_angle) {
        float global_wind = std::fmod(true_wind_angle + heading, 360.0);
        float global_upwind = std::fmod(global_wind + 180.0, 360.0);
        
        if (is_angle_between_boundaries(global_wind, heading, desired_heading)) {
            return SailboatManeuvers::JIBE;
        }
        
        if (is_angle_between_boundaries(global_upwind, heading, desired_heading)) { 
            return SailboatManeuvers::TACK;
        }
        
        return SailboatManeuvers::STANDARD;
    }


    std::pair<float, bool> apply_decision_zone_tacking_logic(float current_heading, float desired_heading, float true_wind_angle, float apparent_wind_angle, float dist) {
        float global_true_wind = std::fmod(current_heading + true_wind_angle, 360.0);
        float global_true_upwind = std::fmod(global_true_wind + 180.0, 360.0);
        float global_app_upwind = std::fmod(current_heading + apparent_wind_angle + 180.0, 360.0);

        float nsz_half = autopilot_parameters["no_sail_zone_size"].get<float>() / 2.0;
        float nsz_lower = std::fmod(global_app_upwind - nsz_half + 360.0, 360.0);
        float nsz_upper = std::fmod(global_app_upwind + nsz_half, 360.0);

        float decision_zone_size_half = get_decision_zone_size(dist) / 2.0;
        float decision_zone_lower = std::fmod(global_true_upwind - decision_zone_size_half + 360.0, 360.0);
        float decision_zone_upper = std::fmod(global_true_upwind + decision_zone_size_half, 360.0);

        if (!is_angle_between_boundaries(desired_heading, nsz_lower, nsz_upper)) {
            bool tack = (get_maneuver_from_desired_heading(current_heading, desired_heading, true_wind_angle) == SailboatManeuvers::TACK);
            return {desired_heading, tack};
        }

        if (is_angle_between_boundaries(desired_heading, decision_zone_upper, nsz_upper)) {
            bool port_side = (std::fmod(current_heading - global_true_upwind + 360.0, 360.0) >= 180.0);
            return {nsz_upper, port_side};
        }
        if (is_angle_between_boundaries(desired_heading, decision_zone_lower, nsz_lower)) {
            bool star_side = (std::fmod(current_heading - global_true_upwind + 360.0, 360.0) < 180.0);
            return {nsz_lower, star_side};
        }

        float d_low = std::abs(get_distance_between_angles(nsz_lower, current_heading));
        float d_high = std::abs(get_distance_between_angles(nsz_upper, current_heading));


        return (d_low < d_high) ? std::make_pair(nsz_lower, false) : std::make_pair(nsz_upper, false);
    }
};