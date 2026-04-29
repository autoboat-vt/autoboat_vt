// TODO: Maybe add a low power mode when the autopilot is disabled
// TODO: Make this into a standalone executable

#pragma once

#include <chrono>
#include <memory>
#include <map>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "autoboat_msgs/msg/rc_data.hpp"
#include "autoboat_msgs/msg/waypoint_list.hpp"


#include "autopilot_library/sailboat_autopilot.hpp"
#include "autopilot_library/position.hpp"
#include "autopilot_library/autopilot_utils.hpp"



using json = nlohmann::json;


/**
 * @class SailboatAutopilotNode
 * @brief ROS 2 node that handles sailboat autopilot logic.
 * 
 * The autopilot takes in sensor data and waypoints and attempts to traverse through
 * the waypoints by continuously publishing to the sail angle and rudder angle topics.
 * 
 * The main function to pay attention to is update_ros_topics,
 * since this is the function that is called periodically on a timer.
 * 
 * NOTE: All units are in standard SI units and angles are generally measured 
 * in degrees unless otherwise specified.
 */
class SailboatAutopilotNode : public rclcpp::Node {

public:
    /**
     * @brief Construct a new Sailboat Autopilot Node object.
     * 
     * Initializes publishers, subscriptions, and the autopilot core.
     */
    SailboatAutopilotNode();


private:
    
    std::map<std::string, json> autopilot_parameters;
    SailboatAutopilot sailboat_autopilot;
    
    Position current_position = {0.0, 0.0};
    std::array<float, 2> current_global_velocity_vector = {0.0, 0.0};
    std::array<float, 2> current_apparent_wind_vector = {0.0, 0.0};

    float current_heading = 0.0;
    float heading_to_hold = 0.0;

    float joystick_left_y = 0.0;
    float joystick_right_x = 0.0;

    bool should_zero_rudder_encoder = false;
    bool should_zero_winch_encoder = false;

    bool has_rudder_encoder_been_zeroed = false;
    bool has_winch_encoder_been_zeroed = false;
    


    autoboat_msgs::msg::RCData previous_rc_data;

    rclcpp::TimerBase::SharedPtr autopilot_refresh_timer;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_sail_angle_publisher, desired_rudder_angle_publisher, desired_heading_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zero_rudder_encoder_publisher, zero_winch_encoder_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr autopilot_mode_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr full_autonomy_maneuver_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_path_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_index_publisher;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;




    /**
     * @brief Callback for remote control (RC) data.
     * 
     * Processes RC inputs for manual control, mode switching (including semi-autonomous modes),
     * and encoder zeroing.
     * @param msg The RCData message.
     */
    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg);

    /**
     * @brief Callback for position (GPS) updates.
     * @param msg The NavSatFix message containing the current boat position.
     */
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief Callback for heading updates.
     * @param msg The Float32 message containing the current boat heading in degrees.
     */
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback for apparent wind vector updates.
     * @param msg The Vector3 message containing the apparent wind vector.
     */
    void apparent_wind_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /**
     * @brief Callback for velocity/speed updates.
     * @param msg The Twist message containing the current boat velocity.
     */
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback for waypoint list updates.
     * @param msg List of waypoints to follow.
     */
    void waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr msg);

    /**
     * @brief Callback for autopilot parameter updates via JSON.
     * @param new_parameters JSON string containing parameter updates.
     */
    void autopilot_parameters_callback(const std_msgs::msg::String::SharedPtr new_parameters);

    /**
     * @brief Periodically called to update ROS topics based on autopilot output.
     * 
     * Handles publisher updates for rudder, sail, and status topics.
     */
    void update_ros_topics();
};
