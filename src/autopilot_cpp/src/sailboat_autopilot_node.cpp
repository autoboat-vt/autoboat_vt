#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "autoboat_msgs/msg/rc_data.hpp"
#include "autoboat_msgs/msg/waypoint_list.hpp"


#include "autopilot_library/sailboat_autopilot.hpp"



using namespace std::chrono_literals;


class SailboatAutopilotNode : public rclcpp::Node {

public:
    SailboatAutopilotNode() : Node("sailboat_autopilot_cpp") {
        load_parameters();
        autopilot = std::make_unique<SailboatAutopilot>(params);

        auto sensor_qos = rclcpp::SensorDataQoS();

        // Subscriptions
        rc_data_subscriber = create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&SailboatAutopilotNode::rc_cb, this, std::placeholders::_1));
        position_subscriber = create_subscription<sensor_msgs::msg::NavSatFix>("/position", sensor_qos, std::bind(&SailboatAutopilotNode::pos_cb, this, std::placeholders::_1));
        sub_wind = create_subscription<geometry_msgs::msg::Vector3>("/apparent_wind_vector", sensor_qos, std::bind(&SailboatAutopilotNode::wind_cb, this, std::placeholders::_1));
        sub_head = create_subscription<std_msgs::msg::Float32>("/heading", sensor_qos, std::bind(&SailboatAutopilotNode::head_cb, this, std::placeholders::_1));
        sub_waypoints = create_subscription<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10, std::bind(&SailboatAutopilotNode::waypoint_cb, this, std::placeholders::_1));

        // Publishers
        pub_sail = create_publisher<std_msgs::msg::Float32>("/desired_sail_angle", sensor_qos);
        pub_rudder = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", sensor_qos);
        pub_mode = create_publisher<std_msgs::msg::String>("/autopilot_mode", sensor_qos);

        timer = create_wall_timer(std::chrono::duration<double>(1.0/params["autopilot_refresh_rate"]), std::bind(&SailboatAutopilotNode::update_loop, this));
    }

private:

    std::map<std::string, double> params;
    std::unique_ptr<SailboatAutopilot> autopilot;
    SailboatAutopilotMode mode = SailboatAutopilotMode::Full_RC;
    
    Position current_pos;
    std::array<double, 2> awa_vec = {0,0};
    double current_heading = 0;
    double heading_to_hold = 0;
    double joy_ly = 0, joy_rx = 0;
    int prev_toggle_f = 0;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<autoboat_msgs::msg::RCData>::SharedPtr rc_data_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_wind;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_head;
    rclcpp::Subscription<autoboat_msgs::msg::WaypointList>::SharedPtr sub_waypoints;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_sail, pub_rudder;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode;

    
    void load_parameters() {
        YAML::Node config = YAML::LoadFile("config/sailboat_default_parameters.yaml");
        params["autopilot_refresh_rate"] = config["autopilot_refresh_rate"].as<double>(10.0);
        params["no_sail_zone_size"] = config["no_sail_zone_size"].as<double>(90.0);
        params["waypoint_accuracy"] = config["waypoint_accuracy"].as<double>(5.0);
        params["min_sail_angle"] = config["min_sail_angle"].as<double>(0.0);
        params["max_sail_angle"] = config["max_sail_angle"].as<double>(90.0);
        params["min_rudder_angle"] = config["min_rudder_angle"].as<double>(-45.0);
        params["max_rudder_angle"] = config["max_rudder_angle"].as<double>(45.0);
        params["heading_p_gain"] = config["heading_p_gain"].as<double>(1.0);
    }

    void rc_cb(const autoboat_msgs::msg::RCData::SharedPtr msg) {
        if (msg->toggle_f == 1 && prev_toggle_f != 1) heading_to_hold = current_heading;
        prev_toggle_f = msg->toggle_f;

        joy_ly = msg->joystick_left_y;
        joy_rx = msg->joystick_right_x;

        if (msg->toggle_b != 0) mode = SailboatAutopilotMode::Disabled;
        else if (msg->toggle_f == 2) mode = SailboatAutopilotMode::Waypoint_Mission;
        else if (msg->toggle_f == 1) mode = SailboatAutopilotMode::Hold_Heading;
        else mode = SailboatAutopilotMode::Full_RC;
    }

    void pos_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_pos = Position(msg->longitude, msg->latitude);
    }

    void wind_cb(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        awa_vec = {msg->x, msg->y};
    }

    void head_cb(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading = msg->data;
    }

    void waypoint_cb(const autoboat_msgs::msg::WaypointList::SharedPtr msg) {
        autopilot->reset();
        for (auto &wp : msg->waypoints) {
            autopilot->waypoints.push_back(Position(wp.longitude, wp.latitude));
        }
    }

    void update_loop() {
        double sail = 0.0, rudder = 0.0;

        switch(mode) {
            case SailboatAutopilotMode::Waypoint_Mission: {
                auto res = autopilot->run_waypoint_mission_step(current_pos, {0,0}, current_heading, awa_vec);
                sail = res.first; rudder = res.second;
                break;
            }

            case SailboatAutopilotMode::Hold_Heading: {
                rudder = autopilot->get_optimal_rudder_angle(current_heading, heading_to_hold);
                sail = autopilot->run_rc_control(joy_ly, joy_rx).first;
                break;
            }

            case SailboatAutopilotMode::Full_RC: {
                auto rc = autopilot->run_rc_control(joy_ly, joy_rx);
                sail = rc.first; rudder = rc.second;
                break;
            }

            
            default: return;
        }

        std_msgs::msg::Float32 s_msg, r_msg;
        s_msg.data = sail; r_msg.data = rudder;
        pub_sail->publish(s_msg);
        pub_rudder->publish(r_msg);
    }
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SailboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}