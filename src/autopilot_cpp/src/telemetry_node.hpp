#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <autoboat_msgs/msg/waypoint_list.hpp>
#include <autoboat_msgs/msg/vesc_telemetry_data.hpp>

#include <nlohmann/json.hpp>
#include <cpr/cpr.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
#include <cmath>
#include <openssl/sha.h>
#include <iomanip>
#include <sstream>

#include "telemetry_payloads.hpp"


using json = nlohmann::json;



class TelemetryNode : public rclcpp::Node {
public:
    TelemetryNode();

private:
    std::string sha256(const std::string& str);

    void config_path_callback(const std_msgs::msg::String::SharedPtr msg);
    void initialize_server_connection();
    void send_boat_status();
    void post_bytes(const std::string& route, const char* data, size_t size, cpr::Session* session = nullptr);
    void update_waypoints();
    void update_autopilot_params();
    json get_response_from_telemetry_server(const std::string& route, cpr::Session* session = nullptr);
    void post_json_to_telemetry_server(const std::string& route, const json& j, cpr::Session* session = nullptr);
    void post_empty_to_telemetry_server(const std::string& route, cpr::Session* session = nullptr);
    double calculate_distance(double lat1, double lon1, double lat2, double lon2);

    // Callbacks
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void apparent_wind_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void vesc_data_callback(const autoboat_msgs::msg::VESCTelemetryData::SharedPtr msg);
    void desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void waypoint_index_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void full_autonomy_maneuver_callback(const std_msgs::msg::String::SharedPtr msg);
    void autopilot_mode_callback(const std_msgs::msg::String::SharedPtr msg);
    void desired_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void desired_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void current_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void current_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // State
    bool params_loaded = false;
    bool is_sailboat_mode = false;
    bool is_motorboat_mode = false;
    json autopilot_parameters;
    int instance_id = -1;

    std::string telemetry_server_url_string;
    cpr::Url telemetry_server_url;
    cpr::Session write_boat_status_session, read_waypoints_session, read_autopilot_parameters_session;

    double position_latitude = 0, position_longitude = 0;
    double velocity_vector_x = 0, velocity_vector_y = 0;
    double heading = 0, desired_heading = 0;
    double apparent_wind_x = 0, apparent_wind_y = 0;
    double apparent_wind_speed = 0, apparent_wind_angle = 0;
    double desired_sail_angle = 0, desired_rudder_angle = 0;
    double current_sail_angle = 0, current_rudder_angle = 0;
    int current_waypoint_index = 0;
    std::string autopilot_mode = "DISABLED", full_autonomy_maneuver = "NORMAL";
    std::vector<std::pair<double,double>> current_waypoints_list;

    float vesc_rpm = 0, vesc_duty_cycle = 0, vesc_amp_hours = 0, vesc_amp_hours_charged = 0;
    float vesc_current_to_vesc = 0, vesc_voltage_to_motor = 0, vesc_voltage_to_vesc = 0;
    float vesc_wattage_to_motor = 0, vesc_motor_temperature = 0, vesc_vesc_temperature = 0, vesc_time_ms = 0;

    rclcpp::TimerBase::SharedPtr boat_status_timer, waypoints_timer, autopilot_params_timer;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autopilot_parameters_publisher, sensors_parameters_publisher;
    rclcpp::Publisher<autoboat_msgs::msg::WaypointList>::SharedPtr waypoints_list_publisher;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
};
