#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <any>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


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
#include "autopilot_library/motorboat_autopilot.hpp"





using namespace std::chrono_literals;
using json = nlohmann::json;



class MotorboatAutopilotNode : public rclcpp::Node {
public:

    MotorboatAutopilotNode() : Node("motorboat_autopilot_cpp") {
        
        
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("autopilot_cpp");
        std::string yaml_file_path = package_share_directory + "/config/sailboat_default_parameters.yaml";
    
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        for (auto it = config.begin(); it != config.end(); ++it) {
            std::string key = it->first.as<std::string>();
            autopilot_parameters[key] = yaml_to_json(it->second);
        }

        
        motorboat_autopilot = MotorboatAutopilot(autopilot_parameters);
        
        rclcpp::SensorDataQoS sensor_qos = rclcpp::SensorDataQoS();
        
        // Timer
        autopilot_refresh_timer = create_wall_timer(std::chrono::duration<float>(1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>()), std::bind(&MotorboatAutopilotNode::update_ros_topics, this));


        // Subscriptions
        autopilot_parameters_subscriber = this->create_subscription<std_msgs::msg::String>("/autopilot_parameters", 10, std::bind(&MotorboatAutopilotNode::autopilot_parameters_callback, this, std::placeholders::_1));
        waypoints_list_subscriber = create_subscription<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10, std::bind(&MotorboatAutopilotNode::waypoints_list_callback, this, std::placeholders::_1));

        position_subscriber = create_subscription<sensor_msgs::msg::NavSatFix>("/position", sensor_qos, std::bind(&MotorboatAutopilotNode::position_callback, this, std::placeholders::_1));
        heading_subscriber = create_subscription<std_msgs::msg::Float32>("/heading", sensor_qos, std::bind(&MotorboatAutopilotNode::heading_callback, this, std::placeholders::_1));

        velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("/velocity", sensor_qos, std::bind(&MotorboatAutopilotNode::velocity_callback, this, std::placeholders::_1));
        rc_data_subscriber = create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&MotorboatAutopilotNode::rc_data_callback, this, std::placeholders::_1));



        // Publishers
        current_waypoint_index_publisher = this->create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", 10);
        autopilot_mode_publisher = this->create_publisher<std_msgs::msg::String>("/autopilot_mode", sensor_qos);
        full_autonomy_maneuver_publisher = this->create_publisher<std_msgs::msg::String>("/full_autonomy_maneuver", sensor_qos);

        should_propeller_motor_be_powered_publisher = create_publisher<std_msgs::msg::Bool>("/should_propeller_motor_be_powered", 10);
        propeller_motor_control_struct_publisher = create_publisher<autoboat_msgs::msg::VESCControlData>("/propeller_motor_control_struct", sensor_qos);

        desired_rudder_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", sensor_qos);
        zero_rudder_encoder_publisher = create_publisher<std_msgs::msg::Bool>("/zero_rudder_encoder", 10);
        desired_heading_publisher = this->create_publisher<std_msgs::msg::Float32>("/desired_heading", 10); // this is not really for actually controlling the boat but more for debugging
    }



private:

    MotorboatAutopilot motorboat_autopilot;
    std::map<std::string, json> autopilot_parameters;
    MotorboatAutopilotMode autopilot_mode = MotorboatAutopilotMode::Full_RC;
    MotorboatControls propeller_motor_control_mode = MotorboatControls::RPM;
    DiscretePID pid_controller;


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autopilot_parameters_subscriber;
    rclcpp::Subscription<autoboat_msgs::msg::WaypointList>::SharedPtr waypoints_list_subscriber;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;
    rclcpp::Subscription<autoboat_msgs::msg::RCData>::SharedPtr rc_data_subscriber;


    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_waypoint_index_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autopilot_mode_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr full_autonomy_maneuver_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_heading_publisher;
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_rudder_angle_publisher;
    rclcpp::Publisher<autoboat_msgs::msg::VESCControlData>::SharedPtr propeller_motor_control_struct_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr should_propeller_motor_be_powered_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zero_rudder_encoder_publisher;

    rclcpp::TimerBase::SharedPtr autopilot_refresh_timer;
    rclcpp::Time last_rc_data_time;

    
    double current_latitude = 0.0, current_longitude = 0.0;
    std::vector<float> current_global_velocity = {0.0, 0.0};
    float current_speed = 0.0;
    float current_heading = 0.0;
    float heading_to_hold = 0.0;
    float joystick_left_y = 0.0, joystick_right_x = 0.0;

    int previous_toggle_f = 0;
    
    bool should_propeller_motor_be_powered = false;
    bool should_zero_encoder = false;
    bool encoder_has_been_zeroed = false;



    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_latitude = msg->latitude;
        current_longitude = msg->longitude;
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_global_velocity = std::vector<float>({msg->linear.x, msg->linear.y});
        current_speed = sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2));
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading = msg->data;
    }


    
    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg) {
        last_rc_data_time = this->now();

        // Hold heading logic
        if (msg->toggle_f == 1 && previous_toggle_f != 1) {
            heading_to_hold = current_heading;
        }
        previous_toggle_f = msg->toggle_f;

        joystick_left_y = -1.0 * msg->joystick_left_y;
        joystick_right_x = msg->joystick_right_x;

        // Mode Logic
        if (msg->toggle_b == 1) should_propeller_motor_be_powered = false;
        else should_propeller_motor_be_powered = true;

        if (msg->toggle_f == 2) autopilot_mode = MotorboatAutopilotMode::Waypoint_Mission;
        else if (msg->toggle_f == 1) autopilot_mode = MotorboatAutopilotMode::Hold_Heading;
        else autopilot_mode = MotorboatAutopilotMode::Full_RC;

        // Control Type
        if (msg->toggle_c == 0) propeller_motor_control_mode = MotorboatControls::RPM;
        else if (msg->toggle_c == 1) propeller_motor_control_mode = MotorboatControls::DUTY_CYCLE;
        else propeller_motor_control_mode = MotorboatControls::CURRENT;
    }



    void autopilot_parameters_callback(const std_msgs::msg::String::SharedPtr new_parameters) {

        json new_parameters_json = json::parse(new_parameters->data);

        for (auto it = new_parameters_json.begin(); it != new_parameters_json.end(); ++it) {
            std::string key = it.key();

            if (autopilot_parameters.find(key) == autopilot_parameters.end()) {
                RCLCPP_WARN(this->get_logger(), "New parameter received: %s", key.c_str());
            }

            autopilot_parameters[key] = it.value();
        }


        // Handle Different Autopilot Refresh Rate
        if (new_parameters_json.contains("autopilot_refresh_rate")) {
            RCLCPP_INFO(this->get_logger(), "Updating autopilot refresh rate...");
            
            if (autopilot_refresh_timer) autopilot_refresh_timer->cancel();
            float period = 1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>();
            autopilot_refresh_timer = this->create_wall_timer(std::chrono::duration<float>(period), std::bind(&MotorboatAutopilotNode::update_ros_topics, this));
        }
    }


    void waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr waypoint_list) {
        if (waypoint_list->waypoints.empty()) {
            return;
        }

        // Reset the internal autopilot state
        motorboat_autopilot.reset();

        std::vector<Position> waypoint_positions;
        
        // Convert each NavSatFix to our internal Position class
        for (const auto & navsat : waypoint_list->waypoints) {
            waypoint_positions.emplace_back(navsat.longitude, navsat.latitude);
        }

        // Pass the vector to the autopilot logic
        motorboat_autopilot.update_waypoints_list(waypoint_positions);
        
        RCLCPP_INFO(this->get_logger(), "Received %zu new waypoints.", waypoint_positions.size());
    }




    std::optional<float> step() {
        if (autopilot_mode == MotorboatAutopilotMode::Waypoint_Mission) {
            return 0.0;
        } 
        
        else if (autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
            return motorboat_autopilot.get_optimal_rudder_angle(current_heading, heading_to_hold);
        } 

        else if (autopilot_mode == MotorboatAutopilotMode::Full_RC) {
            auto [_, rudder_angle] = motorboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x);
            return rudder_angle;
        }

        return std::nullopt;
    }


    void update_ros_topics() {
        // Calculate and Publish Rudder Angle
        auto desired_rudder_angle = step();
        

        if (desired_rudder_angle.has_value()) {
            std_msgs::msg::Float32 rudder_msg;
            rudder_msg.data = static_cast<float>(desired_rudder_angle.value());
            desired_rudder_angle_publisher->publish(rudder_msg);
        }


        // Publish Current Target Heading for Telemetry
        if (autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
            std_msgs::msg::Float32 head_msg;
            head_msg.data = static_cast<float>(heading_to_hold);
            desired_heading_publisher->publish(head_msg);
        }


        // RC Disconnect Safety Check (3-second timeout)
        bool has_rc_disconnected = false;
        auto now = this->get_clock()->now();
        if ((now - last_rc_data_time).seconds() >= 3.0) {
            has_rc_disconnected = true;
            
            // Safety: Zero the propeller if signal lost
            autoboat_msgs::msg::VESCControlData stop_msg;
            stop_msg.control_type_for_vesc = "rpm";
            stop_msg.desired_vesc_rpm = 0.0;
            propeller_motor_control_struct_publisher->publish(stop_msg);
        }


        // Propeller Motor Control (Only in Full RC mode)
        if (autopilot_mode == MotorboatAutopilotMode::Full_RC && !has_rc_disconnected) {
            autoboat_msgs::msg::VESCControlData vesc_msg;

            if (propeller_motor_control_mode == MotorboatControls::RPM) {
                vesc_msg.control_type_for_vesc = "rpm";
                vesc_msg.desired_vesc_rpm = 100.0 * joystick_left_y; 
            } 
            else if (propeller_motor_control_mode == MotorboatControls::DUTY_CYCLE) {
                vesc_msg.control_type_for_vesc = "duty_value";
                vesc_msg.desired_vesc_duty_cycle = joystick_left_y;
            } 
            else if (propeller_motor_control_mode == MotorboatControls::CURRENT) {
                vesc_msg.control_type_for_vesc = "current";
                vesc_msg.desired_vesc_current = joystick_left_y;
            }
            
            propeller_motor_control_struct_publisher->publish(vesc_msg);
        }


        // Zero Encoder Handling
        if (should_zero_encoder) {
            std_msgs::msg::Bool zero_msg;
            zero_msg.data = true;
            zero_rudder_encoder_publisher->publish(zero_msg);
            encoder_has_been_zeroed = true;
            should_zero_encoder = false; // Reset flag after publishing
        }


        // Should Propeller Motor Be Powered
        std_msgs::msg::Bool should_propeller_motor_be_powered_msg;
        should_propeller_motor_be_powered_msg.data = should_propeller_motor_be_powered;
        should_propeller_motor_be_powered_publisher->publish(should_propeller_motor_be_powered_msg);
    }
};






int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}