#pragma once

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
#include "std_msgs/msg/u_int8.hpp"
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



using json = nlohmann::json;



/**
 * @class MotorboatAutopilotNode
 * @brief ROS 2 node that handles motorboat autopilot logic.
 * 
 * The autopilot takes in sensor data and waypoints and attempts to traverse through
 * the waypoints by continuously publishing to the propeller motor control struct 
 * and rudder angle topics.
 * 
 * The main function to pay attention to is update_ros_topics,
 * since this is the function that is called periodically on a timer.
 * 
 * NOTE: All units are in standard SI units and angles are generally measured 
 * in degrees unless otherwise specified.
 */
class MotorboatAutopilotNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Motorboat Autopilot Node object.
     * 
     * Initializes publishers, subscriptions, and the autopilot core.
     */
    MotorboatAutopilotNode();


private:

    MotorboatAutopilot motorboat_autopilot;
    std::map<std::string, json> autopilot_parameters;
    MotorboatAutopilotMode autopilot_mode = MotorboatAutopilotMode::Waypoint_Mission;
    MotorboatControls propeller_motor_control_mode = MotorboatControls::RPM;
    DiscretePID pid_controller;


    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_waypoint_index_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr autopilot_mode_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr full_autonomy_maneuver_publisher;
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
    std::vector<rclcpp::SubscriptionBase::SharedPtr> sub;



    /**
     * @brief Callback for position (GPS) updates.
     * @param msg The NavSatFix message containing the current boat position.
     */
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief Callback for velocity/speed updates.
     * @param msg The Twist message containing the current boat velocity.
     */
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback for heading updates.
     * @param msg The Float32 message containing the current boat heading in degrees.
     */
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback for remote control (RC) data.
     * 
     * Processes RC inputs for manual control, mode switching, and failsafes.
     * @param msg The RCData message.
     */
    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg);

    /**
     * @brief Callback for autopilot parameter updates via JSON.
     * @param new_parameters JSON string containing parameter updates.
     */
    void autopilot_parameters_callback(const std_msgs::msg::String::SharedPtr new_parameters);

    /**
     * @brief Callback for waypoint list updates.
     * @param waypoint_list List of waypoints to follow.
     */
    void waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr waypoint_list);

    /**
     * @brief Runs one step of the autopilot control logic.
     * @return std::pair<float, float> A pair containing the desired rudder angle and propeller RPM.
     */
    std::pair<float, float> step();

    /**
     * @brief Periodically called to update ROS topics based on autopilot output.
     * 
     * Handles publisher updates for rudder, RPM, and status topics.
     */
    void update_ros_topics();
};
