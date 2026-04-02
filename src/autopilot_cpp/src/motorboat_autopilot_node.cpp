#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <any>
#include <fstream>
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
        std::string json_file_path = package_share_directory + "/config/motorboat_default_parameters.json";
    
        std::ifstream file(json_file_path);
        json config = json::parse(file);

        for (auto it = config.begin(); it != config.end(); ++it) {
            autopilot_parameters[it.key()] = it.value()["default"];
        }

        auto trans_qos = rclcpp::QoS(1).reliable().transient_local();
        config_path_publisher = this->create_publisher<std_msgs::msg::String>("/autopilot_param_config_path", trans_qos);
        
        std_msgs::msg::String msg;
        msg.data = json_file_path;
        config_path_publisher->publish(msg);

        
        motorboat_autopilot = MotorboatAutopilot(&autopilot_parameters);
        
        rclcpp::SensorDataQoS sensor_qos = rclcpp::SensorDataQoS();
        
        // Timer
        autopilot_refresh_timer = create_wall_timer(std::chrono::duration<float>(1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>()), std::bind(&MotorboatAutopilotNode::update_ros_topics, this));


        // Subscriptions
        subscriptions.push_back(create_subscription<std_msgs::msg::String>("/autopilot_parameters", 10, std::bind(&MotorboatAutopilotNode::autopilot_parameters_callback, this, std::placeholders::_1)));
        subscriptions.push_back(create_subscription<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10, std::bind(&MotorboatAutopilotNode::waypoints_list_callback, this, std::placeholders::_1)));

        subscriptions.push_back(create_subscription<sensor_msgs::msg::NavSatFix>("/position", 10, std::bind(&MotorboatAutopilotNode::position_callback, this, std::placeholders::_1)));
        subscriptions.push_back(create_subscription<std_msgs::msg::Float32>("/heading", 10, std::bind(&MotorboatAutopilotNode::heading_callback, this, std::placeholders::_1)));

        subscriptions.push_back(create_subscription<geometry_msgs::msg::Twist>("/velocity", 10, std::bind(&MotorboatAutopilotNode::velocity_callback, this, std::placeholders::_1)));
        subscriptions.push_back(create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&MotorboatAutopilotNode::rc_data_callback, this, std::placeholders::_1)));



        // Publishers
        current_waypoint_index_publisher = this->create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", 10);
        autopilot_mode_publisher = this->create_publisher<std_msgs::msg::String>("/autopilot_mode", sensor_qos);
        full_autonomy_maneuver_publisher = this->create_publisher<std_msgs::msg::String>("/full_autonomy_maneuver", sensor_qos);

        should_propeller_motor_be_powered_publisher = create_publisher<std_msgs::msg::Bool>("/should_propeller_motor_be_powered", 10);
        propeller_motor_control_struct_publisher = create_publisher<autoboat_msgs::msg::VESCControlData>("/propeller_motor_control_struct", 10);

        desired_rudder_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", 10);
        zero_rudder_encoder_publisher = create_publisher<std_msgs::msg::Bool>("/zero_rudder_encoder", 10);
        desired_heading_publisher = this->create_publisher<std_msgs::msg::Float32>("/desired_heading", 10); // this is not really for actually controlling the boat but more for debugging
    }



private:

    MotorboatAutopilot motorboat_autopilot;
    std::map<std::string, json> autopilot_parameters;
    MotorboatAutopilotMode autopilot_mode = MotorboatAutopilotMode::Waypoint_Mission;
    MotorboatControls propeller_motor_control_mode = MotorboatControls::RPM;
    DiscretePID pid_controller;


    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_waypoint_index_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autopilot_mode_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr full_autonomy_maneuver_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_heading_publisher;
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_rudder_angle_publisher;
    rclcpp::Publisher<autoboat_msgs::msg::VESCControlData>::SharedPtr propeller_motor_control_struct_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr should_propeller_motor_be_powered_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zero_rudder_encoder_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_path_publisher;

    rclcpp::TimerBase::SharedPtr autopilot_refresh_timer;
    rclcpp::Time last_rc_data_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_gps_position_received_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_heading_received_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

    
    double current_latitude = 0.0, current_longitude = 0.0;
    std::vector<float> current_global_velocity = {0.0, 0.0};
    float current_speed = 0.0;
    float current_heading = 0.0;
    float heading_to_hold = 0.0;
    float joystick_left_y = 0.0, joystick_right_x = 0.0;

    int previous_toggle_f = 0;
    bool previous_button_d = false;
    
    bool should_propeller_motor_be_powered = true;
    bool should_zero_encoder = false;
    bool encoder_has_been_zeroed = false;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;



    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_latitude = msg->latitude;
        current_longitude = msg->longitude;
        last_gps_position_received_time = this->now();
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_global_velocity = std::vector<float>({static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y)});
        current_speed = std::hypot(msg->linear.x, msg->linear.y);
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading = msg->data;
        last_heading_received_time = this->now();
    }


    
    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg) {
        last_rc_data_time = this->now();

        // Hold heading logic
        if (msg->toggle_f == 1 && previous_toggle_f != 1) {
            heading_to_hold = current_heading;
        }
        previous_toggle_f = msg->toggle_f;

        // Rudder Zeroing Logic
        if (!previous_button_d && msg->button_d) {
            should_zero_encoder = true;
            encoder_has_been_zeroed = false;
        } else if (encoder_has_been_zeroed) {
            should_zero_encoder = false;
        }
        previous_button_d = msg->button_d;

        joystick_left_y = -1.0 * msg->joystick_left_y;
        joystick_right_x = msg->joystick_right_x;

        // Mode Logic
        if (msg->toggle_b == 1) should_propeller_motor_be_powered = false;
        else should_propeller_motor_be_powered = true;

        if (msg->toggle_b == 2) autopilot_mode = MotorboatAutopilotMode::Disabled;
        else if (msg->toggle_f == 2) autopilot_mode = MotorboatAutopilotMode::Waypoint_Mission;
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
                RCLCPP_INFO(this->get_logger(), "New parameter received: %s", key.c_str());
            }

            if (it.value().is_object() && it.value().contains("default")) {
                autopilot_parameters[key] = it.value()["default"];
            } else {
                autopilot_parameters[key] = it.value();
            }
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




    std::pair<float, float> step() {
        float rpm = 0.0;
        float rudder_angle = 0.0;

        if (autopilot_mode == MotorboatAutopilotMode::Waypoint_Mission && motorboat_autopilot.get_current_waypoints_list().size() > 0) {
            Position pos(current_longitude, current_latitude);
            auto [res_rpm, res_rudder] = motorboat_autopilot.run_waypoint_mission_step(pos, current_heading);
            rpm = res_rpm;
            rudder_angle = res_rudder;
        } 
        
        else if (autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
            rudder_angle = motorboat_autopilot.get_optimal_rudder_angle(current_heading, heading_to_hold);
        } 

        else if (autopilot_mode == MotorboatAutopilotMode::Full_RC) {
            auto [_, res_rudder] = motorboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x);
            rudder_angle = res_rudder;
        }

        return {rudder_angle, rpm};
    }


    void update_ros_topics() {

        // Publish config path to guarantee delivery
        if (config_path_publisher) {
            std_msgs::msg::String msg;
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("autopilot_cpp");
            msg.data = package_share_directory + "/config/motorboat_default_parameters.json";
            config_path_publisher->publish(msg);
        }

        // Publish mode and telemetry info
        std_msgs::msg::Int32 index_msg;
        index_msg.data = motorboat_autopilot.get_current_waypoint_index();
        current_waypoint_index_publisher->publish(index_msg);

        std_msgs::msg::String mode_msg;
        mode_msg.data = to_string(autopilot_mode);
        autopilot_mode_publisher->publish(mode_msg);

        std_msgs::msg::String maneuver_msg;
        maneuver_msg.data = "N/A";
        full_autonomy_maneuver_publisher->publish(maneuver_msg);

        // Calculate and Publish Rudder Angle
        auto [desired_rudder_angle, desired_rpm] = step();
        
        std_msgs::msg::Float32 rudder_msg;
        rudder_msg.data = desired_rudder_angle;
        desired_rudder_angle_publisher->publish(rudder_msg);

        // Publish Current Target Heading for Telemetry
        if (autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
            std_msgs::msg::Float32 head_msg;
            head_msg.data = static_cast<float>(heading_to_hold);
            desired_heading_publisher->publish(head_msg);
        } else if (autopilot_mode == MotorboatAutopilotMode::Waypoint_Mission && motorboat_autopilot.get_current_waypoints_list().size() > 0) {
            Position pos(current_longitude, current_latitude);
            int idx = motorboat_autopilot.get_current_waypoint_index();
            auto list = motorboat_autopilot.get_current_waypoints_list();
            if (idx >= 0 && idx < (int)list.size()) {
                double bearing = get_bearing(pos, list[idx]);
                std_msgs::msg::Float32 head_msg;
                head_msg.data = static_cast<float>(bearing);
                desired_heading_publisher->publish(head_msg);
            } else {
                std_msgs::msg::Float32 head_msg;
                head_msg.data = 0.0f;
                desired_heading_publisher->publish(head_msg);
            }
        } else {
            std_msgs::msg::Float32 head_msg;
            head_msg.data = 0.0f;
            desired_heading_publisher->publish(head_msg);
        }


        // Disconnect Safety Check
        auto now = this->get_clock()->now();
        double rc_dt = (now - last_rc_data_time).seconds();
        double gps_dt = (now - last_gps_position_received_time).seconds();
        double heading_dt = (now - last_heading_received_time).seconds();

        bool has_rc_disconnected = rc_dt >= autopilot_parameters["rc_data_failsafe_time"].get<double>();
        bool has_gps_disconnected = gps_dt >= autopilot_parameters["gps_position_failsafe_time"].get<double>();
        bool has_heading_disconnected = heading_dt >= autopilot_parameters["heading_data_failsafe_time"].get<double>();
        bool has_sensor_disconnected = has_rc_disconnected || has_gps_disconnected || has_heading_disconnected;

        autoboat_msgs::msg::VESCControlData vesc_msg;

        if (has_sensor_disconnected) {
            vesc_msg.control_type_for_vesc = "rpm";
            vesc_msg.desired_vesc_rpm = 0.0;
            vesc_msg.desired_vesc_current = 0.0;
            vesc_msg.desired_vesc_duty_cycle = 0.0;
        }
        else if (autopilot_mode == MotorboatAutopilotMode::Full_RC || autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
            if (propeller_motor_control_mode == MotorboatControls::RPM) {
                vesc_msg.control_type_for_vesc = "rpm";
                vesc_msg.desired_vesc_rpm = 100.0 * joystick_left_y; 
                vesc_msg.desired_vesc_current = 0.0;
                vesc_msg.desired_vesc_duty_cycle = 0.0;
            } 
            else if (propeller_motor_control_mode == MotorboatControls::DUTY_CYCLE) {
                vesc_msg.control_type_for_vesc = "duty_cycle";
                vesc_msg.desired_vesc_duty_cycle = joystick_left_y;
                vesc_msg.desired_vesc_current = 0.0;
                vesc_msg.desired_vesc_rpm = 0.0;
            } 
            else if (propeller_motor_control_mode == MotorboatControls::CURRENT) {
                vesc_msg.control_type_for_vesc = "current";
                vesc_msg.desired_vesc_current = joystick_left_y;
                vesc_msg.desired_vesc_rpm = 0.0;
                vesc_msg.desired_vesc_duty_cycle = 0.0;
            }
        }
        else if (autopilot_mode == MotorboatAutopilotMode::Waypoint_Mission) {
            vesc_msg.control_type_for_vesc = "rpm";
            vesc_msg.desired_vesc_current = 0.0;
            vesc_msg.desired_vesc_rpm = desired_rpm;
            vesc_msg.desired_vesc_duty_cycle = 0.0;
        }
        else {
            vesc_msg.control_type_for_vesc = "rpm";
            vesc_msg.desired_vesc_current = 0.0;
            vesc_msg.desired_vesc_rpm = 0.0;
            vesc_msg.desired_vesc_duty_cycle = 0.0;
        }
        
        propeller_motor_control_struct_publisher->publish(vesc_msg);


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