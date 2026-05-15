#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
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

#include "autopilot_library/telemetry_payloads.hpp"
#include "autopilot_library/geographic_function_library.hpp"

using json = nlohmann::json;



/**
 * @class TelemetryNode
 * @brief ROS 2 node that collects information from multiple topics and transmits it to the groundstation.
 * 
 * This node handles communication with a telemetry server, including sending boat status
 * and receiving waypoint or parameter updates.
 */
class TelemetryNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Telemetry Node object.
     * 
     * Initializes publishers, subscriptions, and connection to the telemetry server.
     */
    TelemetryNode();


    
private:
    /**
     * @brief Computes the SHA256 hash of a given string.
     * @param str The input string to hash.
     * @return The hex-encoded SHA256 hash.
     */
    std::string sha256(const std::string& str);

    /**
     * @brief Callback function for receiving the configuration path.
     * @param msg The String message containing the path to the configuration file.
     */
    void config_path_callback(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief Initializes the connection settings and sessions for the telemetry server.
     */
    void initialize_server_connection();

    /**
     * @brief Sends the current boat status/telemetry to the telemetry server.
     */
    void send_boat_status();

    /**
     * @brief Sends raw byte data to a specific route on the telemetry server using POST.
     * @param route The server endpoint route.
     * @param data Pointer to the byte data.
     * @param size Size of the data in bytes.
     * @param session Optional cpr::Session to use for the request.
     */
    void post_bytes(const std::string& route, const char* data, size_t size, cpr::Session* session = nullptr);

    /**
     * @brief Periodically checks for and updates the mission waypoints from the server.
     */
    void update_waypoints();

    /**
     * @brief Periodically checks for and updates autopilot parameters from the server.
     */
    void update_autopilot_params();

    /**
     * @brief Performs a GET request to the telemetry server and parses the response as JSON.
     * @param route The server endpoint route.
     * @param session Optional cpr::Session to use for the request.
     * @return The JSON response from the server.
     */
    json get_response_from_telemetry_server(const std::string& route, cpr::Session* session = nullptr);

    /**
     * @brief Performs a POST request with a JSON payload to the telemetry server.
     * @param route The server endpoint route.
     * @param j The JSON object to send.
     * @param session Optional cpr::Session to use for the request.
     */
    void post_json_to_telemetry_server(const std::string& route, const json& j, cpr::Session* session = nullptr);

    /**
     * @brief Performs an empty POST request to the telemetry server.
     * @param route The server endpoint route.
     * @param session Optional cpr::Session to use for the request.
     */
    void post_empty_to_telemetry_server(const std::string& route, cpr::Session* session = nullptr);


    

    // ----------------------------------------------------------------------------------------
    // Callbacks
    // ----------------------------------------------------------------------------------------
    /**
     * @brief Callback for position (GPS) updates.
     * @param msg The NavSatFix message.
     */
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief Callback for velocity updates.
     * @param msg The Twist message.
     */
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback for heading updates.
     * @param msg The Float32 message.
     */
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback for apparent wind updates.
     * @param msg The Vector3 message.
     */
    void apparent_wind_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /**
     * @brief Callback for VESC telemetry data.
     * @param msg The VESCTelemetryData message.
     */
    void vesc_data_callback(const autoboat_msgs::msg::VESCTelemetryData::SharedPtr msg);

    /**
     * @brief Callback for desired heading updates.
     * @param msg The Float32 message.
     */
    void desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback for current waypoint index updates.
     * @param msg The Int32 message.
     */
    void waypoint_index_callback(const std_msgs::msg::Int32::SharedPtr msg);

    /**
     * @brief Callback for full autonomy maneuver updates.
     * @param msg The UInt8 message.
     */
    void full_autonomy_maneuver_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    /**
     * @brief Callback for autopilot mode updates.
     * @param msg The UInt8 message.
     */
    void autopilot_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    /**
     * @brief Callback for desired sail angle updates.
     * @param msg The Float32 message.
     */
    void desired_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback for desired rudder angle updates.
     * @param msg The Float32 message.
     */
    void desired_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback for current sail angle updates.
     * @param msg The Float32 message.
     */
    void current_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback for current rudder angle updates.
     * @param msg The Float32 message.
     */
    void current_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);


    // ----------------------------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------------------------
    bool params_loaded = false;
    bool is_sailboat_mode = false;
    bool is_motorboat_mode = false;
    json autopilot_parameters;
    int instance_id = -1;

    std::string telemetry_server_url_string;
    cpr::Url telemetry_server_url;
    cpr::Session write_boat_status_session, read_waypoints_session, read_autopilot_parameters_session;

    double position_latitude = 0, position_longitude = 0;
    float velocity_vector_x = 0.0f, velocity_vector_y = 0.0f;
    float heading = 0.0f, desired_heading = 0.0f;
    float apparent_wind_x = 0.0f, apparent_wind_y = 0.0f;
    float apparent_wind_speed = 0.0f, apparent_wind_angle = 0.0f;
    float desired_sail_angle = 0.0f, desired_rudder_angle = 0.0f;
    float current_sail_angle = 0.0f, current_rudder_angle = 0.0f;
    int current_waypoint_index = 0;
    uint8_t autopilot_mode = 0, full_autonomy_maneuver = 0;
    std::vector<std::pair<double,double>> current_waypoints_list;

    float vesc_rpm = 0, vesc_duty_cycle = 0, vesc_amp_hours = 0, vesc_amp_hours_charged = 0;
    float vesc_current_to_vesc = 0, vesc_voltage_to_motor = 0, vesc_voltage_to_vesc = 0;
    float vesc_wattage_to_motor = 0, vesc_motor_temperature = 0, vesc_vesc_temperature = 0, vesc_time_ms = 0;

    rclcpp::TimerBase::SharedPtr boat_status_timer, waypoints_timer, autopilot_params_timer;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autopilot_parameters_publisher, sensors_parameters_publisher;
    rclcpp::Publisher<autoboat_msgs::msg::WaypointList>::SharedPtr waypoints_list_publisher;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
};
