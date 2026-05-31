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

    auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    config_path_publisher = this->create_publisher<std_msgs::msg::String>("/autopilot_param_config_path", transient_qos);
    config_path_publisher->publish(std_msgs::msg::String().set__data(json_file_path));



    sailboat_autopilot = SailboatAutopilot(&autopilot_parameters);
    auto sensor_qos = rclcpp::SensorDataQoS();


    subs.push_back(create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&SailboatAutopilotNode::rc_data_callback, this, _1)));
    subs.push_back(create_subscription<sensor_msgs::msg::NavSatFix>("/position", sensor_qos, std::bind(&SailboatAutopilotNode::position_callback, this, _1)));
    subs.push_back(create_subscription<geometry_msgs::msg::Twist>("/velocity", sensor_qos, std::bind(&SailboatAutopilotNode::velocity_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Float32>("/heading", sensor_qos, std::bind(&SailboatAutopilotNode::heading_callback, this, _1)));
    subs.push_back(create_subscription<geometry_msgs::msg::Vector3>("/apparent_wind_vector", sensor_qos, std::bind(&SailboatAutopilotNode::apparent_wind_vector_callback, this, _1)));
    subs.push_back(create_subscription<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10, std::bind(&SailboatAutopilotNode::waypoints_list_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::String>("/autopilot_parameters", 10, std::bind(&SailboatAutopilotNode::autopilot_parameters_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Bool>("/object_detection_emergency_stop", 10, std::bind(&SailboatAutopilotNode::emergency_stop_callback, this, _1)));

    // Publishers
    desired_sail_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_sail_angle", 10);
    desired_rudder_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", 10);
    autopilot_mode_publisher = create_publisher<std_msgs::msg::UInt8>("/autopilot_mode", sensor_qos);
    full_autonomy_maneuver_publisher = create_publisher<std_msgs::msg::UInt8>("/full_autonomy_maneuver", sensor_qos);
    waypoint_index_publisher = create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", 10);
    zero_rudder_encoder_publisher = create_publisher<std_msgs::msg::Bool>("/zero_rudder_encoder", 10);
    zero_winch_encoder_publisher = create_publisher<std_msgs::msg::Bool>("/zero_winch_encoder", 10);
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
        sailboat_control_mode = SailboatControlModes::DISABLED;
    }

    else if (msg->toggle_f == 2) {
        sailboat_control_mode = SailboatControlModes::WAYPOINT_MISSION;
    }

    else if (msg->toggle_f == 1) {

        if (msg->toggle_c == 0) {
            sailboat_control_mode = SailboatControlModes::HOLD_HEADING;
        }

        else if (msg->toggle_c == 1) {
            sailboat_control_mode = SailboatControlModes::HOLD_BEST_SAIL;
        }

        else {
            sailboat_control_mode = SailboatControlModes::HOLD_HEADING_AND_BEST_SAIL;
        }
    } 
    
    else {
        sailboat_control_mode = SailboatControlModes::FULL_RC;
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
    auto [_, apparent_wind_angle] = cartesian_vector_to_polar(msg->x, msg->y);
    current_apparent_wind_angle = apparent_wind_angle;
}

void SailboatAutopilotNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { 
    current_global_velocity_vector = {static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y)}; 
}


void SailboatAutopilotNode::waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr msg) {
    if (msg->waypoints.empty()) {
        return;
    }

    sailboat_autopilot.reset();

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





void SailboatAutopilotNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && sailboat_control_mode == SailboatControlModes::WAYPOINT_MISSION) {
        sailboat_control_mode = SailboatControlModes::EMERGENCY_STOP;
        heading_entered_emergency_stop_in = current_heading;
    }
    else if (!msg->data) {
        sailboat_control_mode = SailboatControlModes::WAYPOINT_MISSION;
    }
}


void SailboatAutopilotNode::update_ros_topics() {
    std::optional<float> desired_sail_angle = std::nullopt;
    std::optional<float> desired_rudder_angle = std::nullopt;
    std::optional<float> desired_heading = std::nullopt;

    if (sailboat_control_mode == SailboatControlModes::EMERGENCY_STOP) {
        auto [stop_sail, stop_rudder] = sailboat_autopilot.run_emergency_stop_step(
            heading_entered_emergency_stop_in, current_heading, current_apparent_wind_angle
        );
        desired_sail_angle = stop_sail;
        desired_rudder_angle = stop_rudder;
    }

    else if (sailboat_control_mode == SailboatControlModes::WAYPOINT_MISSION) {
        auto [waypoint_sail, waypoint_rudder, waypoint_heading] = sailboat_autopilot.run_waypoint_mission_step(
            current_position, current_global_velocity_vector, current_heading, current_apparent_wind_vector
        );
        desired_sail_angle = waypoint_sail;
        desired_rudder_angle = waypoint_rudder;
        desired_heading = waypoint_heading;
    }
    else if (sailboat_control_mode == SailboatControlModes::HOLD_BEST_SAIL) {
        desired_sail_angle = sailboat_autopilot.get_optimal_sail_angle(current_apparent_wind_angle);
        auto [_, rc_rudder] = sailboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x);
        desired_rudder_angle = rc_rudder;
    }
    else if (sailboat_control_mode == SailboatControlModes::HOLD_HEADING) {
        desired_rudder_angle = sailboat_autopilot.get_optimal_rudder_angle(current_heading, heading_to_hold);
        auto [rc_sail, _] = sailboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x);
        desired_sail_angle = rc_sail;
    }
    else if (sailboat_control_mode == SailboatControlModes::HOLD_HEADING_AND_BEST_SAIL) {
        desired_rudder_angle = sailboat_autopilot.get_optimal_rudder_angle(current_heading, heading_to_hold);
        desired_sail_angle = sailboat_autopilot.get_optimal_sail_angle(current_apparent_wind_angle);
    }
    else if (sailboat_control_mode == SailboatControlModes::FULL_RC) {
        auto [rc_sail, rc_rudder] = sailboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x);
        desired_sail_angle = rc_sail;
        desired_rudder_angle = rc_rudder;
    }

    // Now that we have computed what the desired sail and rudder angles should be, we should publish everything
    if (desired_rudder_angle.has_value()) {
        desired_rudder_angle_publisher->publish(std_msgs::msg::Float32().set__data(*desired_rudder_angle));
    }
    if (desired_sail_angle.has_value()) {
        desired_sail_angle_publisher->publish(std_msgs::msg::Float32().set__data(*desired_sail_angle));
    }

    waypoint_index_publisher->publish(std_msgs::msg::Int32().set__data(sailboat_autopilot.get_current_waypoint_index()));

    autopilot_mode_publisher->publish(std_msgs::msg::UInt8().set__data(static_cast<uint8_t>(sailboat_control_mode)));
    if (sailboat_control_mode == SailboatControlModes::WAYPOINT_MISSION) {
        full_autonomy_maneuver_publisher->publish(std_msgs::msg::UInt8().set__data(static_cast<uint8_t>(sailboat_autopilot.get_current_waypoint_mission_state())));
    }
    else {
        full_autonomy_maneuver_publisher->publish(std_msgs::msg::UInt8().set__data(255));
    }

    // Publish the desired heading
    if (sailboat_control_mode == SailboatControlModes::HOLD_HEADING || sailboat_control_mode == SailboatControlModes::HOLD_HEADING_AND_BEST_SAIL) {
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(heading_to_hold));
    }
    else if (sailboat_control_mode == SailboatControlModes::WAYPOINT_MISSION && desired_heading.has_value()) {
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(*desired_heading));
    }
    else {
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(0.0f));
    }

    if (should_zero_rudder_encoder) {
        zero_rudder_encoder_publisher->publish(std_msgs::msg::Bool().set__data(should_zero_rudder_encoder));
        has_rudder_encoder_been_zeroed = true;
    }

    if (should_zero_winch_encoder) {
        zero_winch_encoder_publisher->publish(std_msgs::msg::Bool().set__data(should_zero_winch_encoder));
        has_winch_encoder_been_zeroed = true;
    }

    

}




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SailboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}