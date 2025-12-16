// #include <autopilot_cpp/sailboat_autopilot.hpp>

#pragma once

#include <vector>
#include <string>
#include <map>
#include <variant>
#include "discrete_pid.hpp"
#include "autopilot_utils.hpp"

class SailboatAutopilot {
public:
    std::vector<Position> waypoints;
    int current_waypoint_index = 0;
    SailboatStates current_state = SailboatStates::NORMAL;

    SailboatAutopilot(std::map<std::string, double>& params) 
        : params_(params), 
          rudder_pid(1.0/params["autopilot_refresh_rate"], params["heading_p_gain"], 0, 0, 1.0) {}

    void reset() {
        waypoints.clear();
        current_waypoint_index = 0;
        current_state = SailboatStates::NORMAL;
        rudder_pid.reset();
    }

    double get_optimal_sail_angle(double apparent_wind_angle) {
        // Simplified lookup table logic
        double abs_awa = std::abs(get_distance_between_angles(apparent_wind_angle, 0));
        // Map 0->180 to min_sail_angle -> max_sail_angle
        double sail_range = params_["max_sail_angle"] - params_["min_sail_angle"];
        return params_["min_sail_angle"] + (abs_awa / 180.0) * sail_range;
    }

    double get_optimal_rudder_angle(double current_heading, double target_heading) {
        double error = get_distance_between_angles(target_heading, current_heading);
        double out = rudder_pid.step(error);
        return std::clamp(out, params_["min_rudder_angle"], params_["max_rudder_angle"]);
    }

    std::pair<double, double> run_rc_control(double joy_ly, double joy_rx) {
        double sail = ((joy_ly + 100.0) / 200.0) * (params_["max_sail_angle"] - params_["min_sail_angle"]) + params_["min_sail_angle"];
        double rudder = ((joy_rx + 100.0) / 200.0) * (params_["max_rudder_angle"] - params_["min_rudder_angle"]) + params_["min_rudder_angle"];
        return {sail, rudder};
    }

    std::pair<double, double> run_waypoint_mission_step(const Position& pos, const std::array<double, 2>& vel_vec, double heading, const std::array<double, 2>& awa_vec) {
        if (waypoints.empty()) return {0.0, 0.0};

        // 1. Calculate Winds
        auto [boat_speed, global_vel_angle] = cartesian_vector_to_polar(vel_vec[0], vel_vec[1]);
        double local_vel_angle = (global_vel_angle - heading) * M_PI / 180.0;
        std::array<double, 2> local_vel = {boat_speed * std::cos(local_vel_angle), boat_speed * std::sin(local_vel_angle)};
        
        std::array<double, 2> true_wind_vec = {awa_vec[0] + local_vel[0], awa_vec[1] + local_vel[1]};
        auto [tws, twa] = cartesian_vector_to_polar(true_wind_vec[0], true_wind_vec[1]);
        auto [aws, awa] = cartesian_vector_to_polar(awa_vec[0], awa_vec[1]);

        // 2. Check Waypoint Arrival
        double dist = get_distance_between_positions(pos, waypoints[current_waypoint_index]);
        if (dist < params_["waypoint_accuracy"]) {
            if (current_waypoint_index + 1 < waypoints.size()) current_waypoint_index++;
        }

        // 3. Decision Zone Logic (Simplified for brevity)
        double target_heading = get_bearing(pos, waypoints[current_waypoint_index]);
        
        // Upwind/No-Sail Logic
        double global_twa = std::fmod(twa + heading, 360.0);
        double upwind_angle = std::fmod(global_twa + 180.0, 360.0);

        if (is_angle_between_boundaries(upwind_angle, heading, target_heading)) {
            // Needs to Tack: Force to edge of No-Sail Zone
            double offset = params_["no_sail_zone_size"] / 2.0;
            target_heading = std::fmod(upwind_angle + offset, 360.0);
        }

        return {get_optimal_sail_angle(awa), get_optimal_rudder_angle(heading, target_heading)};
    }

private:
    std::map<std::string, double>& params_;
    DiscretePID rudder_pid;
};