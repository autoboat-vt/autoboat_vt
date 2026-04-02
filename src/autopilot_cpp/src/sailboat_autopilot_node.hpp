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


class SailboatAutopilotNode : public rclcpp::Node {

public:
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autopilot_mode_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr full_autonomy_maneuver_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_path_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_index_publisher;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;




    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg);
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void apparent_wind_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr msg);
    void autopilot_parameters_callback(const std_msgs::msg::String::SharedPtr new_parameters);
    void update_ros_topics();
};
