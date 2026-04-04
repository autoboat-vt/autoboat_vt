#include "sailboat_autopilot_node.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


SailboatAutopilotNode::SailboatAutopilotNode() : Node("sailboat_autopilot") {


    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autopilot_cpp");
    std::string json_file_path = package_share_directory + "/config/sailboat_default_parameters.json";

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



    sailboat_autopilot = SailboatAutopilot(&autopilot_parameters);
    auto sensor_qos = rclcpp::SensorDataQoS();


    // Subscriptions
    subs.push_back(create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&SailboatAutopilotNode::rc_data_callback, this, _1)));
    subs.push_back(create_subscription<sensor_msgs::msg::NavSatFix>("/position", sensor_qos, std::bind(&SailboatAutopilotNode::position_callback, this, _1)));
    subs.push_back(create_subscription<geometry_msgs::msg::Twist>("/velocity", sensor_qos, std::bind(&SailboatAutopilotNode::velocity_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Float32>("/heading", sensor_qos, std::bind(&SailboatAutopilotNode::heading_callback, this, _1)));
    subs.push_back(create_subscription<geometry_msgs::msg::Vector3>("/apparent_wind_vector", sensor_qos, std::bind(&SailboatAutopilotNode::apparent_wind_vector_callback, this, _1)));
    subs.push_back(create_subscription<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10, std::bind(&SailboatAutopilotNode::waypoints_list_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::String>("/autopilot_parameters", 10, std::bind(&SailboatAutopilotNode::autopilot_parameters_callback, this, _1)));

    // Publishers
    desired_sail_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_sail_angle", 10);
    desired_rudder_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", 10);
    autopilot_mode_publisher = create_publisher<std_msgs::msg::UInt8>("/autopilot_mode", sensor_qos);
    full_autonomy_maneuver_publisher = create_publisher<std_msgs::msg::UInt8>("/full_autonomy_maneuver", sensor_qos);
    waypoint_index_publisher = create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", 10);
    desired_heading_publisher = create_publisher<std_msgs::msg::Float32>("/desired_heading", 10);
    autopilot_refresh_timer = create_wall_timer(std::chrono::duration<double>(1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>()), std::bind(&SailboatAutopilotNode::update_ros_topics, this));

}




void SailboatAutopilotNode::rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg) {
    if (msg->toggle_f == 1 && previous_rc_data.toggle_f != 1) { 
        heading_to_hold = current_heading;
    }


    // Are we trying to zero the rudder?
    if (previous_rc_data.button_d == false && msg->button_d == true) {
        should_zero_rudder_encoder = true;
        has_rudder_encoder_been_zeroed = false;
    }

    else if (has_rudder_encoder_been_zeroed) {
        should_zero_rudder_encoder = false;
    }

    // Are we trying to zero the winch?
    if (previous_rc_data.button_a == false && msg->button_a == true) {
        should_zero_winch_encoder = true;
        has_winch_encoder_been_zeroed = false;
    }

    else if (has_winch_encoder_been_zeroed) {
        should_zero_winch_encoder = false;
    }


    joystick_left_y = msg->joystick_left_y; 
    joystick_right_x = msg->joystick_right_x;

    if (msg->toggle_b != 0) {
        sailboat_autopilot.current_autopilot_mode = SailboatAutopilotModes::Disabled;
    }

    else if (msg->toggle_f == 2) {
        sailboat_autopilot.current_autopilot_mode = SailboatAutopilotModes::Waypoint_Mission;
    }

    else if (msg->toggle_f == 1) {

        if (msg->toggle_c == 0) {
            sailboat_autopilot.current_autopilot_mode = SailboatAutopilotModes::Hold_Heading;
        }

        else if (msg->toggle_c == 1) {
            sailboat_autopilot.current_autopilot_mode = SailboatAutopilotModes::Hold_Best_Sail;
        }

        else {
            sailboat_autopilot.current_autopilot_mode = SailboatAutopilotModes::Hold_Heading_And_Best_Sail;
        }
    } 
    
    else {
        sailboat_autopilot.current_autopilot_mode = SailboatAutopilotModes::Full_RC;
    }
    
    
    previous_rc_data = *msg;
}


void SailboatAutopilotNode::position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { 
    current_position = Position(msg->longitude, msg->latitude); 
}

void SailboatAutopilotNode::heading_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
    current_heading = msg->data; 
}

void SailboatAutopilotNode::apparent_wind_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { 
    current_apparent_wind_vector = {static_cast<float>(msg->x), static_cast<float>(msg->y)}; 
}

void SailboatAutopilotNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { 
    current_global_velocity_vector = {static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y)}; 
}


void SailboatAutopilotNode::waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr msg) {

    std::vector<Position> waypoints;
    for (auto &waypoint : msg->waypoints) {
        waypoints.push_back(Position(waypoint.longitude, waypoint.latitude));
    }

    sailboat_autopilot.update_waypoints_list(waypoints);
}




void SailboatAutopilotNode::autopilot_parameters_callback(const std_msgs::msg::String::SharedPtr new_parameters) {

    json new_parameters_json = json::parse(new_parameters->data);

    for (auto it = new_parameters_json.begin(); it != new_parameters_json.end(); ++it) {
        std::string key = it.key();

        if (autopilot_parameters.find(key) == autopilot_parameters.end()) {
            RCLCPP_WARN(this->get_logger(), "New parameter received: %s", key.c_str());
        }

        else if (it.value().is_object() && it.value().contains("default")) {
            RCLCPP_INFO(this->get_logger(), "TEST: %s", key.c_str());
            autopilot_parameters[key] = it.value()["default"];
        } 
        
        else {
            RCLCPP_INFO(this->get_logger(), "TEST2: %s", key.c_str());
            autopilot_parameters[key] = it.value();
        }
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





void SailboatAutopilotNode::update_ros_topics() {
    // Publish config path to guarantee delivery
    if (config_path_publisher) {
        std_msgs::msg::String msg;
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("autopilot_cpp");
        msg.data = package_share_directory + "/config/sailboat_default_parameters.json";
        config_path_publisher->publish(msg);
    }

    if (sailboat_autopilot.get_current_waypoints_list().size() == 0) {
        return;
    }


    auto [desired_rudder_angle, desired_sail_angle] = sailboat_autopilot.step(
        current_position, current_global_velocity_vector, current_heading, current_apparent_wind_vector,
        heading_to_hold, joystick_right_x, joystick_left_y
    );
    
    
    // This happens if the autopilot is currently in the "Disabled" mode
    if (std::isnan(desired_rudder_angle) || std::isnan(desired_sail_angle)) {
        return;
    }


    desired_sail_angle_publisher->publish(std_msgs::msg::Float32().set__data(desired_sail_angle));
    desired_rudder_angle_publisher->publish(std_msgs::msg::Float32().set__data(desired_rudder_angle));



    autopilot_mode_publisher->publish(std_msgs::msg::UInt8().set__data(static_cast<uint8_t>(sailboat_autopilot.current_autopilot_mode)));
    if (sailboat_autopilot.current_autopilot_mode == SailboatAutopilotModes::Waypoint_Mission) {
        full_autonomy_maneuver_publisher->publish(std_msgs::msg::UInt8().set__data(static_cast<uint8_t>(sailboat_autopilot.get_current_waypoint_mission_state())));
    }
    else {
        full_autonomy_maneuver_publisher->publish(std_msgs::msg::UInt8().set__data(255));
    }

    waypoint_index_publisher->publish(std_msgs::msg::Int32().set__data(sailboat_autopilot.get_current_waypoint_index()));



    // Publish the desired heading
    if (sailboat_autopilot.current_autopilot_mode == SailboatAutopilotModes::Hold_Heading || sailboat_autopilot.current_autopilot_mode == SailboatAutopilotModes::Hold_Heading_And_Best_Sail) {
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(heading_to_hold));
    }

    else if (sailboat_autopilot.current_autopilot_mode == SailboatAutopilotModes::Waypoint_Mission && !sailboat_autopilot.get_current_waypoints_list().empty()) {
        Position current_waypoint = sailboat_autopilot.get_current_waypoints_list()[sailboat_autopilot.get_current_waypoint_index()];

        // TODO make it so that the bearing is the actual heading the autopilot is trying to follow (this is different when tacking)
        // when tacking, the boat is not trying to head straight towards the waypoint, but rather, it is travelling on a tacking line
        // maybe add a new ROOS topic specifically for the actual heading that the autopilot is trying to follow
        float bearing_to_waypoint = get_bearing(current_position, current_waypoint);
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(bearing_to_waypoint));
    }

    else {
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(0.0));
    }




    if (should_zero_rudder_encoder) {
        zero_rudder_encoder_publisher->publish(std_msgs::msg::Bool().set__data(should_zero_rudder_encoder));
        has_rudder_encoder_been_zeroed = true;
    }

    if (should_zero_winch_encoder) {
        zero_winch_encoder_publisher->publish(std_msgs::msg::Bool().set__data(should_zero_winch_encoder));
        has_winch_encoder_been_zeroed = true;
    }

    
    std_msgs::msg::Int32 waypoint_index_message; 
    waypoint_index_message.data = sailboat_autopilot.get_current_waypoint_index();
    waypoint_index_publisher->publish(waypoint_index_message);

}




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SailboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}