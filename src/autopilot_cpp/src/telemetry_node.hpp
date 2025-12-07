#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>

#include <autoboat_msgs/msg/waypoint_list.hpp>
#include <autoboat_msgs/msg/vesc_telemetry_data.hpp>

#include <nlohmann/json.hpp>


// https requests library https://github.com/libcpr/cpr
#include <cpr/cpr.h>



#include <chrono>
#include <thread>
#include <mutex>


using namespace std::chrono_literals;
using json = nlohmann::json;



class TelemetryNode : public rclcpp::Node {
public:
    TelemetryNode(const std::string &telemetry_server_url_string = "https://vt-autoboat-telemetry.uk");


private:

    // ------------------------------------------------------------
    // CALLBACKS
    // ------------------------------------------------------------
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void apparent_wind_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void vesc_telemetry_data_callback(const autoboat_msgs::msg::VESCTelemetryData::SharedPtr msg);
    void current_waypoint_index_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void full_autonomy_maneuver_callback(const std_msgs::msg::String::SharedPtr msg);
    void autopilot_mode_callback(const std_msgs::msg::String::SharedPtr msg);
    void desired_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void desired_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);


    std::pair<double,double> cartesian_vector_to_polar(double x, double y);

    // TODO: FIX THIS
    double get_distance_between_positions(double lat1, double lon1, double lat2, double lon2);


    json get_response_from_telemetry_server(const std::string &route, cpr::Session *session = nullptr);
    bool post_json_to_telemetry_server(const std::string &route, const json &json_to_post, cpr::Session *session = nullptr);

    // ------------------------------------------------------------
    // PERIODIC TASKS TIED TO TIMERS
    // ------------------------------------------------------------
    void write_boat_status_to_telemetry_server();
    void read_waypoints_from_telemetry_server();
    void read_autopilot_parameters_from_telemetry_server();






    // ------------------------------------------------------------
    // PRIVATE VARIABLES
    // ------------------------------------------------------------
    std::vector<std::pair<double,double>> current_waypoints_list_;
    int current_waypoint_index_;
    double position_latitude_, position_longitude_;

    std::string autopilot_mode_;
    std::string full_autonomy_maneuver_;

    double velocity_vector_x_, velocity_vector_y_;
    double speed_;
    double heading_;
    double desired_heading_;

    double apparent_wind_x_, apparent_wind_y_;
    double apparent_wind_speed_, apparent_wind_angle_;
    double desired_sail_angle_, desired_rudder_angle_;

    // vesc telemetry
    int vesc_rpm_;

    int instance_id_{-1};
    cpr::Url telemetry_server_url_;
    std::string telemetry_server_url_string_;

    cpr::Session write_boat_status_to_telemetry_server_session_;
    cpr::Session read_waypoints_from_telemetry_server_session_;
    cpr::Session read_autopilot_parameters_from_telemetry_server_session_;

    std::mutex mutex_;



    // ROS pubs/subs
    rclcpp::TimerBase::SharedPtr boat_status_timer_;
    rclcpp::TimerBase::SharedPtr waypoints_timer_;
    rclcpp::TimerBase::SharedPtr autopilot_params_timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autopilot_parameters_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensors_parameters_pub_;
    rclcpp::Publisher<autoboat_msgs::msg::WaypointList>::SharedPtr waypoints_list_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_heading_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_waypoint_index_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr full_autonomy_maneuver_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autopilot_mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr apparent_wind_sub_;
    rclcpp::Subscription<autoboat_msgs::msg::VESCTelemetryData>::SharedPtr vesc_telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_sail_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_rudder_angle_sub_;
};