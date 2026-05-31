#include "motorboat_autopilot.hpp"

#include <cmath>
#include <algorithm>


MotorboatAutopilot::MotorboatAutopilot() {}


MotorboatAutopilot::MotorboatAutopilot(std::map<std::string, json> *autopilot_parameters_) {
    autopilot_parameters = autopilot_parameters_;
    reset();
}


int MotorboatAutopilot::get_current_waypoint_index() const {
    return current_waypoint_index;
}

std::vector<Position> MotorboatAutopilot::get_current_waypoints_list() const {
    return waypoints;
}


void MotorboatAutopilot::reset() {
    heading_pid_controller = DiscretePID(
        1.0f / (*autopilot_parameters)["autopilot_refresh_rate"].get<float>(), 
        (*autopilot_parameters)["heading_p_gain"].get<float>(), 
        (*autopilot_parameters)["heading_i_gain"].get<float>(), 
        (*autopilot_parameters)["heading_d_gain"].get<float>(), 
        (*autopilot_parameters)["heading_n_gain"].get<float>()
    );

    waypoints.clear();
    current_waypoint_index = 0;
}


void MotorboatAutopilot::update_waypoints_list(const std::vector<Position>& waypoints_list) {
    waypoints = waypoints_list;
    current_waypoint_index = 0;
}


float MotorboatAutopilot::get_optimal_rudder_angle(float heading, float target_heading) {
    float error = get_distance_between_angles(target_heading, heading);

    // Update PID gains in case they were changed via YAML/JSON
    heading_pid_controller.set_gains(
        1.0f / (*autopilot_parameters)["autopilot_refresh_rate"].get<float>(), // sample_period
        (*autopilot_parameters)["heading_p_gain"].get<float>(),
        (*autopilot_parameters)["heading_i_gain"].get<float>(),
        (*autopilot_parameters)["heading_d_gain"].get<float>(),
        (*autopilot_parameters)["heading_n_gain"].get<float>()
    );

    float rudder_angle = heading_pid_controller.step(error);
    
    return std::clamp(
        rudder_angle, 
        (*autopilot_parameters)["min_rudder_angle"].get<float>(), 
        (*autopilot_parameters)["max_rudder_angle"].get<float>()
    );
}


std::tuple<std::string, float, float> MotorboatAutopilot::run_rc_control(
    float joystick_left_y, float joystick_right_x,
    PropellerMotorControlMode propeller_motor_control_mode
) {
    float min_rudder_angle = (*autopilot_parameters)["min_rudder_angle"].get<float>();
    float max_rudder_angle = (*autopilot_parameters)["max_rudder_angle"].get<float>();
    float desired_rudder_angle = (((joystick_right_x - -100.0f) * (max_rudder_angle - min_rudder_angle)) / (100.0f - -100.0f)) + min_rudder_angle;

    std::string desired_vesc_control_type;
    float desired_vesc_control_value = 0.0f;

    if (propeller_motor_control_mode == PropellerMotorControlMode::RPM) {
        desired_vesc_control_type = "rpm";
        desired_vesc_control_value = 100.0f * joystick_left_y;
    }
    else if (propeller_motor_control_mode == PropellerMotorControlMode::DUTY_CYCLE) {
        desired_vesc_control_type = "duty_cycle";
        desired_vesc_control_value = joystick_left_y;
    }
    else if (propeller_motor_control_mode == PropellerMotorControlMode::CURRENT) {
        desired_vesc_control_type = "current";
        desired_vesc_control_value = joystick_left_y;
    }
    else {
        desired_vesc_control_type = "rpm";
        desired_vesc_control_value = 0.0f;
    }

    return {desired_vesc_control_type, desired_vesc_control_value, desired_rudder_angle};
}

float MotorboatAutopilot::get_optimal_rpm(float rudder_angle) {
    float error = std::abs(rudder_angle);
    float max_rpm = (*autopilot_parameters)["max_rpm"].get<float>();
    float min_rpm = (*autopilot_parameters)["min_rpm"].get<float>();
    
    float rpm_output = min_rpm + (max_rpm - min_rpm) * std::exp(-(*autopilot_parameters)["rpm_decay_rate"].get<float>() * error);
    return std::clamp(rpm_output, min_rpm, max_rpm);
}

std::pair<float, std::optional<float>> MotorboatAutopilot::run_waypoint_mission_step(Position current_position, float heading) {
    if (waypoints.empty()) {
        return {0.0f, std::nullopt};
    }

    Position desired_position = waypoints[current_waypoint_index];
    float distance_to_desired_position = get_distance_between_positions(current_position, desired_position);
    float desired_heading = get_bearing(current_position, desired_position);

    float rudder_angle = get_optimal_rudder_angle(heading, desired_heading);
    float propeller_rpm = get_optimal_rpm(rudder_angle);

    if (distance_to_desired_position < (*autopilot_parameters)["waypoint_accuracy"].get<float>()) {
        rudder_angle = 0.0f;
        propeller_rpm = 0.0f;
        
        if (waypoints.size() <= (size_t)(current_waypoint_index + 1)) {
            reset();
            return {0.0f, std::nullopt};
        }
        
        current_waypoint_index++;
        desired_position = waypoints[current_waypoint_index];
    }

    return {propeller_rpm, rudder_angle};
}
