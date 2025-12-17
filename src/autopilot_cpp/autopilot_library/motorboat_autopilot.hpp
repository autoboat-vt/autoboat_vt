// #include <autopilot_cpp/motorboat_autopilot.hpp>
// #include <rclcpp/rclcpp.hpp>

#include "discrete_pid.hpp"
#include "autopilot_utils.hpp"
#include "position.hpp"
#include <vector>


using json = nlohmann::json;


/*
This class contains the main control algorithms to control a motorboat given sensor data.
As the person using this class, you generally should not have to worry about any of the functions that start with an underscore such as _function_name.
These functions are meant to be private to the class and are essentially helper functions. 

This class is mainly used by the Motorboat Autopilot Node to control the boat through a ROS topic
*/
class MotorboatAutopilot {

public:

    MotorboatAutopilot() {
        
    }


    MotorboatAutopilot(std::map<std::string, json> autopilot_parameters_) {

        autopilot_parameters = autopilot_parameters_;
        
        rudder_angle_to_heading_pid_controller = DiscretePID(
            1/ autopilot_parameters["autopilot_refresh_rate"].get<float>(), 
            autopilot_parameters["heading_p_gain"].get<float>(), 
            autopilot_parameters["heading_i_gain"].get<float>(), 
            autopilot_parameters["heading_d_gain"].get<float>(), 
            autopilot_parameters["heading_n_gain"].get<float>()
        );

        //     self.logger = logger
        //     self.waypoints: list[Position] = None        
        //     self.current_state = SailboatStates.NORMAL
        
        //     self.current_waypoint_index = 0

    }


    void reset() {

    }


    void update_waypoints_list(std::vector<Position>) {
        
    }


    float get_optimal_rudder_angle(float heading, float target_heading) {
        float error = get_distance_between_angles(target_heading, heading);

        // Update PID gains in case they were changed via YAML/JSON
        rudder_angle_to_heading_pid_controller.set_gains(
            autopilot_parameters["heading_p_gain"].get<float>(),
            autopilot_parameters["heading_i_gain"].get<float>(),
            autopilot_parameters["heading_d_gain"].get<float>(),
            autopilot_parameters["heading_n_gain"].get<float>(),
            1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>() // sample_period
        );

        float rudder_angle = rudder_angle_to_heading_pid_controller.step(error);
        
        return std::clamp(rudder_angle, autopilot_parameters["min_rudder_angle"].get<float>(), autopilot_parameters["max_rudder_angle"].get<float>());
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



private:
    DiscretePID rudder_angle_to_heading_pid_controller;
    
    std::map<std::string, json> autopilot_parameters;

    std::vector<Position> waypoints;
    int current_waypoint_index = 0;
};
    
    // def __init__(self, autopilot_parameters: dict[str, Any], logger: RcutilsLogger):
    //     """
    //     Args:
    //         parameters (dict): a dictionary that should contain the information from the config/motorboat_default_parameters.yaml file.
    //             For more information on specific parameters you are allowed to use, please see that file.
            
    //         logger (RcutilsLogger): A ROS logger to use instead of print statements which works a little better with ROS.
    //             For more information: https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html.
    //             This logger is what you get by running self.get_logger(). So for example in order to log an info message,
    //             please use logger.info("message").
    //     """


    //     self.rudder_pid_controller = Discrete_PID(
    //         sample_period=(1 / autopilot_parameters['autopilot_refresh_rate']), 
    //         Kp=autopilot_parameters['heading_p_gain'], Ki=autopilot_parameters['heading_i_gain'], 
    //         Kd=autopilot_parameters['heading_d_gain'], n=autopilot_parameters['heading_n_gain'], 
    //     )
        
    //     self.autopilot_parameters = autopilot_parameters
    //     self.logger = logger
    //     self.waypoints: list[Position] = None        
    //     self.current_state = SailboatStates.NORMAL
        
    //     self.desired_tacking_angle = 0
    //     self.current_waypoint_index = 0
        
    
    // def reset(self):
    //     """
    //     Reinitializes the MotorboatAutopilot with the same parameters and the same logger. This essentially resets things like waypoints. 
    //     """ 

    //     pass
    
    // def update_waypoints_list(self, waypoints_list: list[Position]) -> None:
    //     """
    //     Args:
    //         waypoints_list (list[Position]): a list of all the waypoints you would like the boat to follow
    //     """
    //     pass