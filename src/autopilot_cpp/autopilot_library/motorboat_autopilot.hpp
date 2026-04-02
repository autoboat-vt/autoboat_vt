#include <vector>


#include "discrete_pid.hpp"
#include "autopilot_utils.hpp"
#include "position.hpp"


using json = nlohmann::json;


/*
This class contains the main control algorithms to control a motorboat given sensor data.
As the person using this class, you generally should not have to worry about any of the functions that start with an underscore such as _function_name.
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

    MotorboatAutopilot() {}


    MotorboatAutopilot(std::map<std::string, json> *autopilot_parameters_) {

        autopilot_parameters = autopilot_parameters_;
        
        rudder_angle_to_heading_pid_controller = DiscretePID(
            1/ (*autopilot_parameters)["autopilot_refresh_rate"].get<float>(), 
            (*autopilot_parameters)["heading_p_gain"].get<float>(), 
            (*autopilot_parameters)["heading_i_gain"].get<float>(), 
            (*autopilot_parameters)["heading_d_gain"].get<float>(), 
            (*autopilot_parameters)["heading_n_gain"].get<float>()
        );

        waypoints.clear();
        current_waypoint_index = 0;
        rudder_angle_to_heading_pid_controller.reset();        

    }


    int get_current_waypoint_index() const {
        return current_waypoint_index;
    }

    std::vector<Position> get_current_waypoints_list() const {
        return waypoints;
    }


    void reset() {
        waypoints.clear();
        current_waypoint_index = 0;
        rudder_angle_to_heading_pid_controller.reset();
    }


    void update_waypoints_list(const std::vector<Position>& waypoints_list) {
        waypoints = waypoints_list;
        current_waypoint_index = 0;
    }


    float get_optimal_rudder_angle(float heading, float target_heading) {
        float error = get_distance_between_angles(target_heading, heading);

        // Update PID gains in case they were changed via YAML/JSON
        rudder_angle_to_heading_pid_controller.set_gains(
            1.0 / (*autopilot_parameters)["autopilot_refresh_rate"].get<float>(), // sample_period
            (*autopilot_parameters)["heading_p_gain"].get<float>(),
            (*autopilot_parameters)["heading_i_gain"].get<float>(),
            (*autopilot_parameters)["heading_d_gain"].get<float>(),
            (*autopilot_parameters)["heading_n_gain"].get<float>()
        );

        float rudder_angle = rudder_angle_to_heading_pid_controller.step(error);
        
        return std::clamp(
            rudder_angle, 
            (*autopilot_parameters)["min_rudder_angle"].get<float>(), 
            (*autopilot_parameters)["max_rudder_angle"].get<float>()
        );
    }


    std::pair<float, float> run_rc_control(float joystick_left_y, float joystick_right_x) {
        // Formula: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        auto map_range = [](float value, float out_min, float out_max) {
            return (value - (-100.0)) * (out_max - out_min) / (100.0 - (-100.0)) + out_min;
        };

        float sail_angle = map_range(
            joystick_left_y, 
            (*autopilot_parameters)["min_sail_angle"].get<float>(), 
            (*autopilot_parameters)["max_sail_angle"].get<float>()
        );
        
        float rudder_angle = map_range(
            joystick_right_x, 
            (*autopilot_parameters)["min_rudder_angle"].get<float>(), 
            (*autopilot_parameters)["max_rudder_angle"].get<float>()
        );

        return {sail_angle, rudder_angle};
    }

    float get_optimal_rpm(float rudder_angle) {
        float error = std::abs(rudder_angle);
        float max_rpm = (*autopilot_parameters)["max_rpm"].get<float>();
        float min_rpm = (*autopilot_parameters)["min_rpm"].get<float>();
        
        float rpm_output = min_rpm + (max_rpm - min_rpm) * std::exp(-(*autopilot_parameters)["rpm_decay_rate"].get<float>() * error);
        return std::clamp(rpm_output, min_rpm, max_rpm);
    }

    std::pair<float, float> run_waypoint_mission_step(Position current_position, float heading) {
        if (waypoints.empty()) {
            return {0.0f, 0.0f};
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
                return {0.0f, 0.0f};
            }
            
            current_waypoint_index++;
            desired_position = waypoints[current_waypoint_index];
        }

        return {propeller_rpm, rudder_angle};
    }
};