#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "autoboat_msgs/msg/waypoint_list.hpp"
#include "autoboat_msgs/msg/rc_data.hpp"
#include "autoboat_msgs/msg/vesc_control_data.hpp"

#include "autopilot_library/discrete_pid.hpp"
#include "autopilot_library/geographic_function_library.hpp"
#include "autopilot_library/autopilot_utils.hpp"


using namespace std::chrono_literals;


class MotorboatAutopilotNode : public rclcpp::Node {
public:
    MotorboatAutopilotNode() : Node("motorboat_autopilot"), pid_controller(0.1, 1.0, 0.0, 0.0, 1.0) {
        
        
        // TODO FIX THIS 
        params["autopilot_refresh_rate"] = 10.0;
        params["heading_p_gain"] = 1.0;
        params["min_rudder_angle"] = -45.0;
        params["max_rudder_angle"] = 45.0;
        params["min_sail_angle"] = 0.0;
        params["max_sail_angle"] = 90.0;

        auto sensor_qos = rclcpp::SensorDataQoS();

        // Subscriptions
        sub_rc_data = create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&MotorboatAutopilotNode::rc_data_callback, this, std::placeholders::_1));
        sub_position = create_subscription<sensor_msgs::msg::NavSatFix>("/position", sensor_qos, std::bind(&MotorboatAutopilotNode::position_callback, this, std::placeholders::_1));
        sub_heading = create_subscription<std_msgs::msg::Float32>("/heading", sensor_qos, std::bind(&MotorboatAutopilotNode::heading_callback, this, std::placeholders::_1));

        // Publishers
        pub_rudder = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", sensor_qos);
        pub_vesc = create_publisher<autoboat_msgs::msg::VESCControlData>("/propeller_motor_control_struct", sensor_qos);
        pub_prop_power = create_publisher<std_msgs::msg::Bool>("/should_propeller_motor_be_powered", 10);

        // Timer
        double period = 1.0 / params["autopilot_refresh_rate"];
        timer = create_wall_timer(std::chrono::duration<double>(period), std::bind(&MotorboatAutopilotNode::update_ros_topics, this));
    }

private:

    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg) {
        last_rc_time = this->now();

        // Hold heading logic
        if (msg->toggle_f == 1 && prev_toggle_f != 1) {
            heading_to_hold = current_heading;
        }
        prev_toggle_f = msg->toggle_f;

        joystick_left_y = -1.0 * msg->joystick_left_y;
        joystick_right_x = msg->joystick_right_x;

        // Mode Logic
        if (msg->toggle_b == 1) should_prop_power = false;
        else should_prop_power = true;

        if (msg->toggle_f == 2) mode = MotorboatAutopilotMode::Waypoint_Mission;
        else if (msg->toggle_f == 1) mode = MotorboatAutopilotMode::Hold_Heading;
        else mode = MotorboatAutopilotMode::Full_RC;

        // Control Type
        if (msg->toggle_c == 0) control_mode = MotorboatControls::RPM;
        else if (msg->toggle_c == 1) control_mode = MotorboatControls::DUTY_CYCLE;
        else control_mode = MotorboatControls::CURRENT;
    }

    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_lat = msg->latitude;
        current_lon = msg->longitude;
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading = msg->data;
    }


    void update_ros_topics() {
        double rudder_angle = 0.0;
        bool valid_output = false;

        if (mode == MotorboatAutopilotMode::Hold_Heading) {
            double error = get_distance_between_angles(heading_to_hold, current_heading);
            rudder_angle = pid_controller.step(error);
            // Clamp
            rudder_angle = std::max(params["min_rudder_angle"], std::min(rudder_angle, params["max_rudder_angle"]));
            valid_output = true;
        } 
        else if (mode == MotorboatAutopilotMode::Full_RC) {
            // Formula: (((val - old_min) * (new_range)) / (old_range)) + new_min
            double range = params["max_rudder_angle"] - params["min_rudder_angle"];
            rudder_angle = (((joystick_right_x - (-100.0)) * range) / 200.0) + params["min_rudder_angle"];
            valid_output = true;
        }

        if (valid_output) {
            auto msg = std_msgs::msg::Float32();
            msg.data = rudder_angle;
            pub_rudder->publish(msg);
        }

        // Handle VESC / RC Disconnect
        auto vesc_msg = autoboat_msgs::msg::VESCControlData();
        if ((this->now() - last_rc_time).seconds() > 3.0) {
            vesc_msg.control_type_for_vesc = "rpm";
            vesc_msg.desired_vesc_rpm = 0.0;
        } 
        
        else if (mode == MotorboatAutopilotMode::Full_RC) {
            
            if (control_mode == MotorboatControls::RPM) {
                vesc_msg.control_type_for_vesc = "rpm";
                vesc_msg.desired_vesc_rpm = 100.0 * joystick_left_y;
            } 
            
            else if (control_mode == MotorboatControls::DUTY_CYCLE) {
                vesc_msg.control_type_for_vesc = "duty_value";
                vesc_msg.desired_vesc_duty_cycle = joystick_left_y;
            }
        }


        pub_vesc->publish(vesc_msg);

        auto power_msg = std_msgs::msg::Bool();
        power_msg.data = should_prop_power;
        pub_prop_power->publish(power_msg);
    }



    // Members
    std::map<std::string, double> params;
    MotorboatAutopilotMode mode = MotorboatAutopilotMode::Full_RC;
    MotorboatControls control_mode = MotorboatControls::RPM;
    DiscretePID pid_controller;

    rclcpp::Subscription<autoboat_msgs::msg::RCData>::SharedPtr sub_rc_data;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_position;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_heading;
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_rudder;
    rclcpp::Publisher<autoboat_msgs::msg::VESCControlData>::SharedPtr pub_vesc;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_prop_power;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time last_rc_time;

    double current_heading = 0.0;
    double heading_to_hold = 0.0;
    double current_lat = 0.0, current_lon = 0.0;
    double joystick_left_y = 0.0, joystick_right_x = 0.0;
    int prev_toggle_f = 0;
    bool should_prop_power = false;
};




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}