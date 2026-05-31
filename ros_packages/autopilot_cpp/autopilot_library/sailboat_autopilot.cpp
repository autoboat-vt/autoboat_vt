#include "sailboat_autopilot.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>
#include <rclcpp/rclcpp.hpp>



// --- Private methods ---

float SailboatAutopilot::get_decision_zone_size(float distance_to_waypoint) {
    float tack_distance = (*autopilot_parameters)["tack_distance"].get<float>();
    float no_sail_zone_half = (*autopilot_parameters)["no_sail_zone_size"].get<float>() / 2.0;
    float temp = (tack_distance / distance_to_waypoint) * std::sin(no_sail_zone_half * static_cast<float>(M_PI) / 180.0f);
    temp = std::clamp(temp, -1.0f, 1.0f);
    float decision_zone_size = std::asin(temp) * 180.0f / static_cast<float>(M_PI);

    return std::clamp(decision_zone_size, (float) 0.0, (*autopilot_parameters)["no_sail_zone_size"].get<float>());
}



SailboatManeuvers SailboatAutopilot::get_maneuver_from_desired_heading(float heading, float desired_heading, float true_wind_angle) {
    float global_wind_angle = std::fmod(true_wind_angle + heading, 360.0f);
    float global_upwind_angle = std::fmod(global_wind_angle + 180.0f, 360.0f);
    
    if (is_angle_between_boundaries(global_wind_angle, heading, desired_heading)) {
        return SailboatManeuvers::JIBE;
    }
    
    if (is_angle_between_boundaries(global_upwind_angle, heading, desired_heading)) { 
        return SailboatManeuvers::TACK;
    }
    
    return SailboatManeuvers::STANDARD;
}




std::pair<float, bool> SailboatAutopilot::apply_decision_zone_tacking_logic(
    float current_heading, float desired_heading, 
    float true_wind_angle, float apparent_wind_angle, float distance
) {
    
    float global_true_wind = std::fmod(current_heading + true_wind_angle, 360.0f);
    float global_true_upwind = std::fmod(global_true_wind + 180.0f, 360.0f);
    float global_app_upwind = std::fmod(current_heading + apparent_wind_angle + 180.0f, 360.0f);

    float no_sail_zone_half = (*autopilot_parameters)["no_sail_zone_size"].get<float>() / 2.0f;
    float no_sail_zone_lower = std::fmod(global_app_upwind - no_sail_zone_half + 360.0f, 360.0f);
    float no_sail_zone_upper = std::fmod(global_app_upwind + no_sail_zone_half, 360.0f);

    float decision_zone_size_half = get_decision_zone_size(distance) / 2.0f;
    float decision_zone_lower = std::fmod(global_true_upwind - decision_zone_size_half + 360.0f, 360.0f);
    float decision_zone_upper = std::fmod(global_true_upwind + decision_zone_size_half, 360.0f);


    if (!is_angle_between_boundaries(desired_heading, no_sail_zone_lower, no_sail_zone_upper)) {
        bool tack = (get_maneuver_from_desired_heading(current_heading, desired_heading, true_wind_angle) == SailboatManeuvers::TACK);
        return {desired_heading, tack};
    }

    if (is_angle_between_boundaries(desired_heading, decision_zone_upper, no_sail_zone_upper)) {
        bool port_side = (std::fmod(current_heading - global_true_upwind + 360.0f, 360.0f) >= 180.0f);
        return {no_sail_zone_upper, port_side};
    }

    if (is_angle_between_boundaries(desired_heading, decision_zone_lower, no_sail_zone_lower)) {
        bool star_side = (std::fmod(current_heading - global_true_upwind + 360.0f, 360.0f) < 180.0f);
        return {no_sail_zone_lower, star_side};
    }


    float distance_to_lower_no_sail_zone = std::abs(get_distance_between_angles(no_sail_zone_lower, current_heading));
    float distance_to_upper_no_sail_zone = std::abs(get_distance_between_angles(no_sail_zone_upper, current_heading));

    if (distance_to_lower_no_sail_zone < distance_to_upper_no_sail_zone) {
        return std::make_pair(no_sail_zone_lower, false);
    }
    else {
        return std::make_pair(no_sail_zone_upper, false);
    }
}



// --- Public methods ---

SailboatAutopilot::SailboatAutopilot() {}


SailboatAutopilot::SailboatAutopilot(std::map<std::string, json> *autopilot_parameters_) {
    autopilot_parameters = autopilot_parameters_;
    reset();
}


void SailboatAutopilot::reset() {
    heading_pid_controller = DiscretePID(
        1.0 / (*autopilot_parameters)["autopilot_refresh_rate"].get<float>(),
        (*autopilot_parameters)["heading_p_gain"].get<float>(),
        (*autopilot_parameters)["heading_i_gain"].get<float>(),
        (*autopilot_parameters)["heading_d_gain"].get<float>(),
        (*autopilot_parameters)["heading_n_gain"].get<float>()
    );

    current_state = SailboatAutopilotStates::DOWNWIND_SAILING;
    desired_tacking_angle = 0.0f;
    current_waypoint_index = 0;
    last_tacking_position = Position(0.0, 0.0);
    last_time_out_of_no_sail_zone = std::chrono::steady_clock::now();
    waypoints = {};
}


void SailboatAutopilot::update_waypoints_list(const std::vector<Position>& waypoints_list) {
    waypoints = waypoints_list;
    current_waypoint_index = 0;
}


float SailboatAutopilot::get_current_waypoint_index() {
    return current_waypoint_index;
}

std::vector<Position> SailboatAutopilot::get_current_waypoints_list() {
    return waypoints;
}

SailboatAutopilotStates SailboatAutopilot::get_current_waypoint_mission_state() {
    return current_state;
}


std::pair<float, float> SailboatAutopilot::run_emergency_stop_step(
    float heading_object_was_detected_at, float current_heading, float apparent_wind_angle
) {
    float global_apparent_wind_angle = std::fmod(current_heading + apparent_wind_angle, 360.0f);
    if (global_apparent_wind_angle < 0.0f) {
        global_apparent_wind_angle += 360.0f;
    }

    float global_apparent_upwind_angle = std::fmod(global_apparent_wind_angle + 180.0f, 360.0f);
    if (global_apparent_upwind_angle < 0.0f) {
        global_apparent_upwind_angle += 360.0f;
    }

    float no_sail_zone_size = (*autopilot_parameters)["no_sail_zone_size"].get<float>();
    float no_sail_zone_bounds_0 = std::fmod(global_apparent_upwind_angle + no_sail_zone_size / 2.0f, 360.0f);
    if (no_sail_zone_bounds_0 < 0.0f) {
        no_sail_zone_bounds_0 += 360.0f;
    }
    float no_sail_zone_bounds_1 = std::fmod(global_apparent_upwind_angle - no_sail_zone_size / 2.0f, 360.0f);
    if (no_sail_zone_bounds_1 < 0.0f) {
        no_sail_zone_bounds_1 += 360.0f;
    }

    float desired_heading = std::fmod(heading_object_was_detected_at + 180.0f, 360.0f);
    if (desired_heading < 0.0f) {
        desired_heading += 360.0f;
    }

    float distance_between_desired_heading_and_left_no_sail_zone = std::abs(get_distance_between_angles(desired_heading, no_sail_zone_bounds_0));
    float distance_between_desired_heading_and_right_no_sail_zone = std::abs(get_distance_between_angles(desired_heading, no_sail_zone_bounds_1));

    bool port_tack_is_closer = distance_between_desired_heading_and_left_no_sail_zone > distance_between_desired_heading_and_right_no_sail_zone;

    if (is_angle_between_boundaries(desired_heading, no_sail_zone_bounds_0, no_sail_zone_bounds_1)) {
        if (port_tack_is_closer) {
            desired_heading = no_sail_zone_bounds_1;
        } else {
            desired_heading = no_sail_zone_bounds_0;
        }
    }

    float desired_rudder_angle = get_optimal_rudder_angle(current_heading, desired_heading);
    float desired_sail_angle = get_optimal_sail_angle(apparent_wind_angle);

    return {desired_sail_angle, desired_rudder_angle};
}


float SailboatAutopilot::get_optimal_sail_angle(float apparent_wind_angle) {
    std::vector<float> wind_angles = (*autopilot_parameters)["sail_lookup_table_wind_angles"].get<std::vector<float>>();
    std::vector<float> sail_angles = (*autopilot_parameters)["sail_lookup_table_sail_positions"].get<std::vector<float>>();

    int left_index = -1;
    int right_index = -1;

    float max_less_than_equal = -std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < wind_angles.size(); ++i) {
        if (wind_angles[i] <= apparent_wind_angle) {
            if (wind_angles[i] > max_less_than_equal) {
                max_less_than_equal = wind_angles[i];
                left_index = i;
            }
        }
    }

    float min_greater_than_equal = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < wind_angles.size(); ++i) {
        if (wind_angles[i] >= apparent_wind_angle) {
            if (wind_angles[i] < min_greater_than_equal) {
                min_greater_than_equal = wind_angles[i];
                right_index = i;
            }
        }
    }

    if (left_index == -1 || right_index == -1) {
        return 0.0f;
    }

    if (left_index == right_index) {
        return sail_angles[left_index];
    } 
    else {
        float slope = (sail_angles[right_index] - sail_angles[left_index]) / (wind_angles[right_index] - wind_angles[left_index]);
        return slope * (apparent_wind_angle - wind_angles[left_index]) + sail_angles[left_index];
    }
}

    
float SailboatAutopilot::get_optimal_rudder_angle(float heading, float desired_heading) {
    heading_pid_controller.set_gains(
        1.0f / (*autopilot_parameters)["autopilot_refresh_rate"].get<float>(),
        (*autopilot_parameters)["heading_p_gain"].get<float>(),
        (*autopilot_parameters)["heading_i_gain"].get<float>(),
        (*autopilot_parameters)["heading_d_gain"].get<float>(),
        (*autopilot_parameters)["heading_n_gain"].get<float>()
    );

    float error = get_distance_between_angles(desired_heading, heading);
    float rudder_angle = heading_pid_controller.step(error);
    
    return std::clamp(
        rudder_angle, 
        (*autopilot_parameters)["min_rudder_angle"].get<float>(), 
        (*autopilot_parameters)["max_rudder_angle"].get<float>()
    );
}


std::pair<float, float> SailboatAutopilot::run_rc_control(float rc_rudder_control, float rc_sail_control) {

    // Formula: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    auto map_range = [](float value, float out_min, float out_max) {
        return (value - (-100.0f)) * (out_max - out_min) / (100.0f - (-100.0f)) + out_min;
    };

    float rudder_angle = map_range(
        rc_rudder_control, 
        (*autopilot_parameters)["min_rudder_angle"].get<float>(), 
        (*autopilot_parameters)["max_rudder_angle"].get<float>()
    );
    
    float sail_angle = map_range(
        rc_sail_control, 
        (*autopilot_parameters)["min_sail_angle"].get<float>(), 
        (*autopilot_parameters)["max_sail_angle"].get<float>()
    );

    return {sail_angle, rudder_angle};
}

std::pair<float, SailboatAutopilotStates> SailboatAutopilot::_apply_tacking_state_machine(
    float current_heading, float current_bearing,
    float true_wind_angle, float apparent_wind_angle,
    Position current_position, Position last_tack_position,
    SailboatAutopilotStates current_state
) {
    float global_true_wind_angle = std::fmod(current_heading + true_wind_angle, 360.0f);
    if (global_true_wind_angle < 0.0f) global_true_wind_angle += 360.0f;

    float global_true_upwind_angle = std::fmod(global_true_wind_angle + 180.0f, 360.0f);
    if (global_true_upwind_angle < 0.0f) global_true_upwind_angle += 360.0f;

    float global_apparent_wind_angle = std::fmod(current_heading + apparent_wind_angle, 360.0f);
    if (global_apparent_wind_angle < 0.0f) global_apparent_wind_angle += 360.0f;

    float global_apparent_upwind_angle = std::fmod(global_apparent_wind_angle + 180.0f, 360.0f);
    if (global_apparent_upwind_angle < 0.0f) global_apparent_upwind_angle += 360.0f;

    float no_sail_zone_size = (*autopilot_parameters)["no_sail_zone_size"].get<float>();
    float no_sail_zone_bounds_0 = std::fmod(global_apparent_upwind_angle + no_sail_zone_size / 2.0f, 360.0f);
    if (no_sail_zone_bounds_0 < 0.0f) no_sail_zone_bounds_0 += 360.0f;
    float no_sail_zone_bounds_1 = std::fmod(global_apparent_upwind_angle - no_sail_zone_size / 2.0f, 360.0f);
    if (no_sail_zone_bounds_1 < 0.0f) no_sail_zone_bounds_1 += 360.0f;

    float distance_between_heading_and_left_no_sail_zone = std::abs(get_distance_between_angles(current_heading, no_sail_zone_bounds_0));
    float distance_between_heading_and_right_no_sail_zone = std::abs(get_distance_between_angles(current_heading, no_sail_zone_bounds_1));

    bool port_tack_is_closer = distance_between_heading_and_left_no_sail_zone > distance_between_heading_and_right_no_sail_zone;

    bool is_waypoint_in_no_sail_zone_biased_value = false;
    if (current_state == SailboatAutopilotStates::CW_TACKING ||
        current_state == SailboatAutopilotStates::CCW_TACKING ||
        current_state == SailboatAutopilotStates::PORT_TACK ||
        current_state == SailboatAutopilotStates::STARBOARD_TACK ||
        current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK ||
        current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK) {
        is_waypoint_in_no_sail_zone_biased_value = true;
    }

    float hysteresis_amount_angles = (*autopilot_parameters)["hysteresis_amount_angles"].get<float>();
    bool is_waypoint_in_no_sail_zone = is_angle_between_boundaries_with_hysteresis(
        current_bearing, no_sail_zone_bounds_0, no_sail_zone_bounds_1,
        is_waypoint_in_no_sail_zone_biased_value, hysteresis_amount_angles
    );

    bool is_heading_in_no_sail_zone_biased_value = false;
    if (current_state == SailboatAutopilotStates::CW_TACKING ||
        current_state == SailboatAutopilotStates::CCW_TACKING ||
        current_state == SailboatAutopilotStates::PORT_TACK ||
        current_state == SailboatAutopilotStates::STARBOARD_TACK ||
        current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK ||
        current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK) {
        is_heading_in_no_sail_zone_biased_value = true;
    }

    bool is_heading_in_no_sail_zone = is_angle_between_boundaries_with_hysteresis(
        current_heading, no_sail_zone_bounds_0, no_sail_zone_bounds_1,
        is_heading_in_no_sail_zone_biased_value, hysteresis_amount_angles
    );

    auto now = std::chrono::steady_clock::now();
    if (!is_heading_in_no_sail_zone) {
        last_time_out_of_no_sail_zone = now;
    }
    else {
        double elapsed = std::chrono::duration<double>(now - last_time_out_of_no_sail_zone).count();
        if (elapsed > (*autopilot_parameters)["max_no_sail_zone_time"].get<float>()) {
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "BEEN IN NO SAIL ZONE TOO LONG TRYING TO WIGGLE. PORT? %d", port_tack_is_closer);
            if (port_tack_is_closer) {
                current_state = SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK;
            } else {
                current_state = SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK;
            }
        }
    }
    double elapsed_log = std::chrono::duration<double>(now - last_time_out_of_no_sail_zone).count();
    RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "time in no sail zone: %f", elapsed_log);

    float distance_from_last_tack_position = get_distance_between_positions(current_position, last_tack_position);
    float tack_distance = (*autopilot_parameters)["tack_distance"].get<float>();
    if (distance_from_last_tack_position > tack_distance) {
        if (current_state == SailboatAutopilotStates::STARBOARD_TACK) {
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "BEEN ON STARBOARD TACK FOR TOO LONG. INITIATING CW TACKING");
            current_state = SailboatAutopilotStates::CW_TACKING;
        } else if (current_state == SailboatAutopilotStates::PORT_TACK) {
            current_state = SailboatAutopilotStates::CCW_TACKING;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "BEEN ON PORT TACK FOR TOO LONG. INITIATING CCW TACKING");
        }
    }

    if (is_heading_in_no_sail_zone && current_state == SailboatAutopilotStates::DOWNWIND_SAILING) {
        if (port_tack_is_closer) {
            current_state = SailboatAutopilotStates::CW_TACKING;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "TRANSITION FROM DOWNWIND SAILING TO CW TACKING");
        } else {
            current_state = SailboatAutopilotStates::CCW_TACKING;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "TRANSITION FROM DOWNWIND SAILING TO CCW TACKING");
        }
    }
    else if (is_waypoint_in_no_sail_zone && current_state == SailboatAutopilotStates::DOWNWIND_SAILING) {
        if (port_tack_is_closer) {
            current_state = SailboatAutopilotStates::PORT_TACK;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "TRANSITION FROM DOWNWIND SAILING TO PORT TACK");
        } else {
            current_state = SailboatAutopilotStates::STARBOARD_TACK;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "TRANSITION FROM DOWNWIND SAILING TO STARBOARD TACK");
        }
    }

    bool is_bearing_around_no_sail_zone = is_angle_between_boundaries_with_hysteresis(
        global_true_upwind_angle, current_heading, current_bearing,
        false, hysteresis_amount_angles
    );
    if (is_bearing_around_no_sail_zone && current_state == SailboatAutopilotStates::DOWNWIND_SAILING) {
        if (get_optimal_rudder_angle(current_heading, current_bearing) > 0.0f) {
            current_state = SailboatAutopilotStates::CW_TACKING;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "GET AROUND NO SAIL ZONE CLOCKWISE TACKING");
        } else {
            current_state = SailboatAutopilotStates::CCW_TACKING;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "GET AROUND NO SAIL ZONE COUNTER CLOCKWISE TACKING");
        }
    }

    if (current_state == SailboatAutopilotStates::STARBOARD_TACK && port_tack_is_closer) {
        RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "distance between heading and left no sail zone: %f", distance_between_heading_and_left_no_sail_zone);
        RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "distance between heading and right no sail zone: %f", distance_between_heading_and_right_no_sail_zone);
        RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "PORT TACK IS CLOSER, SWITCHING FROM STARBOARD TO PORT TACK");
        current_state = SailboatAutopilotStates::PORT_TACK;
    }

    if (current_state == SailboatAutopilotStates::PORT_TACK && !port_tack_is_closer) {
        RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "distance between heading and left no sail zone: %f", distance_between_heading_and_left_no_sail_zone);
        RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "distance between heading and right no sail zone: %f", distance_between_heading_and_right_no_sail_zone);
        RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "STARBOARD TACK IS CLOSER, SWITCHING FROM PORT TO STARBOARD TACK");
        current_state = SailboatAutopilotStates::STARBOARD_TACK;
    }

    if (!is_waypoint_in_no_sail_zone && !is_heading_in_no_sail_zone && (
        current_state == SailboatAutopilotStates::PORT_TACK ||
        current_state == SailboatAutopilotStates::STARBOARD_TACK ||
        current_state == SailboatAutopilotStates::CW_TACKING ||
        current_state == SailboatAutopilotStates::CCW_TACKING ||
        current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK ||
        current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK)) {
        current_state = SailboatAutopilotStates::DOWNWIND_SAILING;
        RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "NO LONGER NEED TO HOLD A TACK. NOW TRANSITIONING TO DOWNWIND SAILING");
    }

    float tack_tolerance = (*autopilot_parameters)["tack_tolerance"].get<float>();
    if (current_state == SailboatAutopilotStates::CCW_TACKING || current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK) {
        float tack_target_heading = no_sail_zone_bounds_0;
        float distance_to_tack_target_heading = std::abs(get_distance_between_angles(current_heading, tack_target_heading));
        if (distance_to_tack_target_heading < tack_tolerance) {
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "JUST FINISHED STARBOARD TACK MANEUVER. NOW HOLDING STARBOARD TACK");
            current_state = SailboatAutopilotStates::STARBOARD_TACK;
        }
    } else if (current_state == SailboatAutopilotStates::CW_TACKING || current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK) {
        float tack_target_heading = no_sail_zone_bounds_1;
        float distance_to_tack_target_heading = std::abs(get_distance_between_angles(current_heading, tack_target_heading));
        if (distance_to_tack_target_heading < tack_tolerance) {
            current_state = SailboatAutopilotStates::PORT_TACK;
            RCLCPP_INFO(rclcpp::get_logger("sailboat_autopilot"), "JUST FINISHED CW TACK MANEUVER. NOW HOLDING PORT TACK");
        }
    }

    float desired_heading = 0.0f;
    if (current_state == SailboatAutopilotStates::STARBOARD_TACK || current_state == SailboatAutopilotStates::CCW_TACKING) {
        desired_heading = no_sail_zone_bounds_0;
    } else if (current_state == SailboatAutopilotStates::PORT_TACK || current_state == SailboatAutopilotStates::CW_TACKING) {
        desired_heading = no_sail_zone_bounds_1;
    } else if (current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK) {
        desired_heading = no_sail_zone_bounds_0;
    } else if (current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK) {
        desired_heading = no_sail_zone_bounds_1;
    } else if (current_state == SailboatAutopilotStates::DOWNWIND_SAILING) {
        desired_heading = current_bearing;
    }

    return {desired_heading, current_state};
}


std::tuple<std::optional<float>, std::optional<float>, std::optional<float>> SailboatAutopilot::run_waypoint_mission_step(
    Position current_position, 
    std::array<float, 2> global_velocity_vector, 
    float heading, 
    std::array<float, 2> apparent_wind_vector
) {
    if (waypoints.empty()) {
        return {std::nullopt, std::nullopt, std::nullopt};
    }

    float boat_speed = std::sqrt(std::pow(global_velocity_vector[0], 2) + std::pow(global_velocity_vector[1], 2));
    auto [_, global_velocity_angle] = cartesian_vector_to_polar(global_velocity_vector[0], global_velocity_vector[1]);
    
    float local_velocity_angle = (global_velocity_angle - heading) * M_PI / 180.0;
    std::array<float, 2> local_velocity_vector = { boat_speed * static_cast<float>(std::cos(local_velocity_angle)), boat_speed * static_cast<float>(std::sin(local_velocity_angle)) };

    std::array<float, 2> true_wind_vector = { apparent_wind_vector[0] + local_velocity_vector[0], apparent_wind_vector[1] + local_velocity_vector[1] };
    auto [true_wind_speed, true_wind_angle] = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1]);
    auto [apparent_wind_speed, apparent_wind_angle] = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1]);

    Position desired_position = waypoints[current_waypoint_index];
    float distance_to_desired_position = get_distance_between_positions(current_position, desired_position);

    float current_bearing = get_bearing(current_position, desired_position);

    float waypoint_accuracy = (*autopilot_parameters)["waypoint_accuracy"].get<float>();
    if (distance_to_desired_position < waypoint_accuracy) {
        if (waypoints.size() <= static_cast<size_t>(current_waypoint_index + 1)) {
            reset();
            return {std::nullopt, std::nullopt, std::nullopt};
        }
        current_waypoint_index++;
    }

    float sail_angle = get_optimal_sail_angle(apparent_wind_angle);
    float rudder_angle = 0.0f;

    auto [desired_heading, next_state] = _apply_tacking_state_machine(
        heading, current_bearing, true_wind_angle, apparent_wind_angle,
        current_position, last_tacking_position, current_state
    );
    current_state = next_state;

    if (current_state == SailboatAutopilotStates::DOWNWIND_SAILING ||
        current_state == SailboatAutopilotStates::PORT_TACK ||
        current_state == SailboatAutopilotStates::STARBOARD_TACK) {
        rudder_angle = get_optimal_rudder_angle(heading, desired_heading);
    } 
    else if (current_state == SailboatAutopilotStates::CW_TACKING ||
             current_state == SailboatAutopilotStates::CCW_TACKING) {
        float tack_direction = (current_state == SailboatAutopilotStates::CW_TACKING) ? 1.0f : -1.0f;
        rudder_angle = (*autopilot_parameters)["rudder_hard_over"].get<float>() * tack_direction;

        if ((*autopilot_parameters)["perform_forced_jibe_instead_of_tack"].get<bool>()) {
            rudder_angle *= -1.0f;
        }

        last_tacking_position = current_position;
    }
    else if (current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK) {
        sail_angle = (*autopilot_parameters)["sail_angle_when_wiggling_out_of_no_sail_zone"].get<float>();
        rudder_angle = (*autopilot_parameters)["min_rudder_angle"].get<float>();
    }
    else if (current_state == SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK) {
        sail_angle = (*autopilot_parameters)["sail_angle_when_wiggling_out_of_no_sail_zone"].get<float>();
        rudder_angle = (*autopilot_parameters)["max_rudder_angle"].get<float>();
    }
    else {
        throw std::runtime_error("Unsupported State Transition In `run_waypoint_mission_step`");
    }

    return {sail_angle, rudder_angle, desired_heading};
}
