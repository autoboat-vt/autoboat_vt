// TODO: Maybe add a low power mode when the autopilot is disabled
// TODO: Make this into a standalone executable

#include <chrono>
#include <memory>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>


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



using namespace std::chrono_literals;
using json = nlohmann::json;




class SailboatAutopilotNode : public rclcpp::Node {

    public:
    SailboatAutopilotNode() : Node("sailboat_autopilot") {


        std::string package_share_directory = ament_index_cpp::get_package_share_directory("autopilot_cpp");
        std::string yaml_file_path = package_share_directory + "/config/sailboat_default_parameters.yaml";
    
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        for (auto it = config.begin(); it != config.end(); ++it) {
            std::string key = it->first.as<std::string>();
            autopilot_parameters[key] = yaml_to_json(it->second);
        }



        sailboat_autopilot = SailboatAutopilot(autopilot_parameters);
        auto sensor_qos = rclcpp::SensorDataQoS();


        // Subscriptions
        rc_data_listener = create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&SailboatAutopilotNode::rc_data_callback, this, std::placeholders::_1));
        position_listener = create_subscription<sensor_msgs::msg::NavSatFix>("/position", sensor_qos, std::bind(&SailboatAutopilotNode::position_callback, this, std::placeholders::_1));
        velocity_listener = create_subscription<geometry_msgs::msg::Twist>("/velocity", sensor_qos, std::bind(&SailboatAutopilotNode::velocity_callback, this, std::placeholders::_1));
        heading_listener = create_subscription<std_msgs::msg::Float32>("/heading", sensor_qos, std::bind(&SailboatAutopilotNode::heading_callback, this, std::placeholders::_1));
        apparent_wind_vector_listener = create_subscription<geometry_msgs::msg::Vector3>("/apparent_wind_vector", sensor_qos, std::bind(&SailboatAutopilotNode::apparent_wind_vector_callback, this, std::placeholders::_1));
        waypoints_list_listener = create_subscription<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10, std::bind(&SailboatAutopilotNode::waypoints_list_callback, this, std::placeholders::_1));

        // Publishers
        desired_sail_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_sail_angle", sensor_qos);
        desired_rudder_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", sensor_qos);
        autopilot_mode_publisher = create_publisher<std_msgs::msg::String>("/autopilot_mode", sensor_qos);
        waypoint_index_publisher = create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", 10);
        desired_heading_publisher = create_publisher<std_msgs::msg::Float32>("/desired_heading", 10);

        autopilot_refresh_timer = create_wall_timer(std::chrono::duration<double>(1.0/autopilot_parameters["autopilot_refresh_rate"].get<float>()), std::bind(&SailboatAutopilotNode::update_ros_topics, this));
    }




private:
    
    std::map<std::string, json> autopilot_parameters;
    SailboatAutopilot sailboat_autopilot;
    SailboatAutopilotMode autopilot_mode = SailboatAutopilotMode::Disabled;
    
    Position current_position = {0.0, 0.0};
    std::array<float, 2> global_velocity_vector = {0.0, 0.0};
    std::array<float, 2> apparent_wind_vector = {0.0, 0.0};
    float current_heading = 0.0;
    float heading_to_hold = 0.0;
    float joystick_left_y = 0.0;
    float joystick_right_x = 0.0;

    autoboat_msgs::msg::RCData previous_rc_data;


    rclcpp::Subscription<autoboat_msgs::msg::RCData>::SharedPtr rc_data_listener;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_listener;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_listener;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_listener;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr apparent_wind_vector_listener;
    rclcpp::Subscription<autoboat_msgs::msg::WaypointList>::SharedPtr waypoints_list_listener;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_sail_angle_publisher, desired_rudder_angle_publisher, desired_heading_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autopilot_mode_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_index_publisher;


    rclcpp::TimerBase::SharedPtr autopilot_refresh_timer;




    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg) {
        if (msg->toggle_f == 1 && previous_rc_data.toggle_f != 1) { 
            heading_to_hold = current_heading;
        }

        joystick_left_y = msg->joystick_left_y; 
        joystick_right_x = msg->joystick_right_x;

        if (msg->toggle_b != 0) {
            autopilot_mode = SailboatAutopilotMode::Disabled;
        }

        else if (msg->toggle_f == 2) {
            autopilot_mode = SailboatAutopilotMode::Waypoint_Mission;
        }

        else if (msg->toggle_f == 1) {

            if (msg->toggle_c == 0) {
                autopilot_mode = SailboatAutopilotMode::Hold_Heading;
            }

            else if (msg->toggle_c == 1) {
                autopilot_mode = SailboatAutopilotMode::Hold_Best_Sail;
            }

            else {
                autopilot_mode = SailboatAutopilotMode::Hold_Heading_And_Best_Sail;
            }
        } 
        
        else {
            autopilot_mode = SailboatAutopilotMode::Full_RC;
        }
        
        
        previous_rc_data = *msg;
    }

    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { 
        current_position = Position(msg->longitude, msg->latitude); 
    }
    
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
        current_heading = msg->data; 
    }
    
    void apparent_wind_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { 
        apparent_wind_vector = {msg->x, msg->y}; 
    }
    
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { 
        global_velocity_vector = {msg->linear.x, msg->linear.y}; 
    }
    

    void waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr msg) {

        std::vector<Position> waypoints;
        for (auto &waypoint : msg->waypoints) {
            waypoints.push_back(Position(waypoint.longitude, waypoint.latitude));
        }

        sailboat_autopilot.update_waypoints_list(waypoints);
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
            
            if (autopilot_refresh_timer) {
                autopilot_refresh_timer->cancel();
            }
            
            float period = 1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>();
            autopilot_refresh_timer = this->create_wall_timer(std::chrono::duration<float>(period), std::bind(&SailboatAutopilotNode::update_ros_topics, this));
        }
    }





    void update_ros_topics() {
        float desired_sail_angle = 0.0; 
        float desired_rudder_angle = 0.0;
        
        bool active = true;

        switch(autopilot_mode) {
            case SailboatAutopilotMode::Waypoint_Mission: {
                auto res = sailboat_autopilot.run_waypoint_mission_step(current_position, global_velocity_vector, current_heading, apparent_wind_vector);
                desired_sail_angle = res.first; 
                desired_rudder_angle = res.second;
                break;
            }

            case SailboatAutopilotMode::Hold_Heading: {
                desired_rudder_angle = sailboat_autopilot.get_optimal_rudder_angle(current_heading, heading_to_hold);
                desired_sail_angle = sailboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x).first;
                break;
            }
            
            case SailboatAutopilotMode::Hold_Best_Sail: {
                auto [apparent_wind_speed, apparent_wind_angle] = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1]);
                desired_sail_angle = sailboat_autopilot.get_optimal_sail_angle(apparent_wind_angle);
                desired_rudder_angle = sailboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x).second;
                break;
            }
            
            case SailboatAutopilotMode::Hold_Heading_And_Best_Sail: {
                auto [apparent_wind_speed, apparent_wind_angle] = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1]);
                desired_sail_angle = sailboat_autopilot.get_optimal_sail_angle(apparent_wind_angle);
                desired_rudder_angle = sailboat_autopilot.get_optimal_rudder_angle(current_heading, heading_to_hold);
                break;
            }
            
            case SailboatAutopilotMode::Full_RC: {
                auto res = sailboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x);
                desired_sail_angle = res.first; 
                desired_rudder_angle = res.second;
                break;
            }
            
            default: { 
                active = false; break;
        
            }
        }


        if (active) {
            desired_sail_angle_publisher->publish(std_msgs::msg::Float32().set__data(desired_sail_angle));
            desired_rudder_angle_publisher->publish(std_msgs::msg::Float32().set__data(desired_rudder_angle));
        
            std_msgs::msg::Int32 waypoint_index_message; 
            waypoint_index_message.data = sailboat_autopilot.current_waypoint_index;
            waypoint_index_publisher->publish(waypoint_index_message);
        }
    }
};





int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SailboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}