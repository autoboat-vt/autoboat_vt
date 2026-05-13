#include "motorboat_autopilot_node.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;



MotorboatAutopilotNode::MotorboatAutopilotNode() : Node("motorboat_autopilot_cpp") {
    
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autopilot_cpp");
    std::string json_file_path = package_share_directory + "/config/motorboat_default_parameters.json";

    std::ifstream file(json_file_path);
    json config = json::parse(file);

    // create a dictionary directly out of the default values of the autopilot parameters
    for (auto it = config.begin(); it != config.end(); ++it) {
        autopilot_parameters[it.key()] = it.value()["default"];
    }

    auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    config_path_publisher = this->create_publisher<std_msgs::msg::String>("/autopilot_param_config_path", transient_qos);
    
    config_path_publisher->publish(std_msgs::msg::String().set__data(json_file_path));


    motorboat_autopilot = MotorboatAutopilot(&autopilot_parameters);
    
    rclcpp::SensorDataQoS sensor_qos = rclcpp::SensorDataQoS();
    
    // Timer
    autopilot_refresh_timer = create_wall_timer(
        std::chrono::duration<float>(1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>()), 
        std::bind(&MotorboatAutopilotNode::update_ros_topics, this)
    );


    // Subscriptions
    sub.push_back(create_subscription<std_msgs::msg::String>("/autopilot_parameters", 10, std::bind(&MotorboatAutopilotNode::autopilot_parameters_callback, this, _1)));
    sub.push_back(create_subscription<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10, std::bind(&MotorboatAutopilotNode::waypoints_list_callback, this, _1)));

    sub.push_back(create_subscription<sensor_msgs::msg::NavSatFix>("/position", 10, std::bind(&MotorboatAutopilotNode::position_callback, this, _1)));
    sub.push_back(create_subscription<std_msgs::msg::Float32>("/heading", 10, std::bind(&MotorboatAutopilotNode::heading_callback, this, _1)));

    sub.push_back(create_subscription<geometry_msgs::msg::Twist>("/velocity", 10, std::bind(&MotorboatAutopilotNode::velocity_callback, this, _1)));
    sub.push_back(create_subscription<autoboat_msgs::msg::RCData>("/rc_data", sensor_qos, std::bind(&MotorboatAutopilotNode::rc_data_callback, this, _1)));



    // Publishers
    current_waypoint_index_publisher = this->create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", 10);
    autopilot_mode_publisher = this->create_publisher<std_msgs::msg::UInt8>("/autopilot_mode", sensor_qos);
    full_autonomy_maneuver_publisher = this->create_publisher<std_msgs::msg::UInt8>("/full_autonomy_maneuver", sensor_qos);

    should_propeller_motor_be_powered_publisher = create_publisher<std_msgs::msg::Bool>("/should_propeller_motor_be_powered", 10);
    propeller_motor_control_struct_publisher = create_publisher<autoboat_msgs::msg::VESCControlData>("/propeller_motor_control_struct", 10);

    desired_rudder_angle_publisher = create_publisher<std_msgs::msg::Float32>("/desired_rudder_angle", 10);
    zero_rudder_encoder_publisher = create_publisher<std_msgs::msg::Bool>("/zero_rudder_encoder", 10);
    
    // this is not really for actually controlling the boat but more for debugging
    desired_heading_publisher = this->create_publisher<std_msgs::msg::Float32>("/desired_heading", 10);
}




void MotorboatAutopilotNode::position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    current_latitude = msg->latitude;
    current_longitude = msg->longitude;
    last_gps_position_received_time = this->now();
}

void MotorboatAutopilotNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    current_global_velocity = std::vector<float>({static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y)});
    current_speed = std::hypot(msg->linear.x, msg->linear.y);
}

void MotorboatAutopilotNode::heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_heading = msg->data;
    last_heading_received_time = this->now();
}



void MotorboatAutopilotNode::rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg) {
    last_rc_data_time = this->now();

    // Hold heading logic
    if (msg->toggle_f == 1 && previous_toggle_f != 1) {
        heading_to_hold = current_heading;
    }
    previous_toggle_f = msg->toggle_f;

    // Rudder Zeroing Logic
    if (!previous_button_d && msg->button_d) {
        should_zero_encoder = true;
        encoder_has_been_zeroed = false;
    } 
    else if (encoder_has_been_zeroed) {
        should_zero_encoder = false;
    }

    previous_button_d = msg->button_d;

    joystick_left_y = -1.0 * msg->joystick_left_y;
    joystick_right_x = msg->joystick_right_x;

    // Mode Logic
    if (msg->toggle_b == 1) should_propeller_motor_be_powered = false;
    else should_propeller_motor_be_powered = true;

    if (msg->toggle_b == 2) autopilot_mode = MotorboatAutopilotMode::Disabled;
    else if (msg->toggle_f == 2) autopilot_mode = MotorboatAutopilotMode::Waypoint_Mission;
    else if (msg->toggle_f == 1) autopilot_mode = MotorboatAutopilotMode::Hold_Heading;
    else autopilot_mode = MotorboatAutopilotMode::Full_RC;

    // Control Type
    if (msg->toggle_c == 0) propeller_motor_control_mode = MotorboatControls::RPM;
    else if (msg->toggle_c == 1) propeller_motor_control_mode = MotorboatControls::DUTY_CYCLE;
    else propeller_motor_control_mode = MotorboatControls::CURRENT;
}



void MotorboatAutopilotNode::autopilot_parameters_callback(const std_msgs::msg::String::SharedPtr new_parameters) {

    json new_parameters_json = json::parse(new_parameters->data);
    
    for (auto it = new_parameters_json.begin(); it != new_parameters_json.end(); ++it) {
        std::string key = it.key();

        if (autopilot_parameters.find(key) == autopilot_parameters.end()) {
            RCLCPP_INFO(this->get_logger(), "New parameter received: %s", key.c_str());
        }

        if (it.value().is_object() && it.value().contains("default")) {
            autopilot_parameters[key] = it.value()["default"];
        } 
        else {
            autopilot_parameters[key] = it.value();
        }
    }


    // Handle autopilot refresh rate differently, since it isn't a simple value
    if (new_parameters_json.contains("autopilot_refresh_rate")) {
        RCLCPP_INFO(this->get_logger(), "Updating autopilot refresh rate...");
        
        if (autopilot_refresh_timer) autopilot_refresh_timer->cancel();
        float period = 1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>();
        autopilot_refresh_timer = this->create_wall_timer(std::chrono::duration<float>(period), std::bind(&MotorboatAutopilotNode::update_ros_topics, this));
    }
}


void MotorboatAutopilotNode::waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr waypoint_list) {
    if (waypoint_list->waypoints.empty()) {
        return;
    }

    // Reset the internal autopilot state
    motorboat_autopilot.reset();

    std::vector<Position> waypoint_positions;
    
    // Convert each NavSatFix to our internal Position class
    for (const auto & navsat : waypoint_list->waypoints) {
        waypoint_positions.emplace_back(navsat.longitude, navsat.latitude);
    }

    // Pass the vector to the autopilot logic
    motorboat_autopilot.update_waypoints_list(waypoint_positions);
    
    RCLCPP_INFO(this->get_logger(), "Received %zu new waypoints.", waypoint_positions.size());
}




std::pair<float, float> MotorboatAutopilotNode::step() {
    float rpm = 0.0;
    float rudder_angle = 0.0;

    if (autopilot_mode == MotorboatAutopilotMode::Waypoint_Mission && motorboat_autopilot.get_current_waypoints_list().size() > 0) {
        Position pos(current_longitude, current_latitude);
        auto [res_rpm, res_rudder] = motorboat_autopilot.run_waypoint_mission_step(pos, current_heading);
        rpm = res_rpm;
        rudder_angle = res_rudder;
    } 
    
    else if (autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
        rudder_angle = motorboat_autopilot.get_optimal_rudder_angle(current_heading, heading_to_hold);
    } 

    else if (autopilot_mode == MotorboatAutopilotMode::Full_RC) {
        auto [_, res_rudder] = motorboat_autopilot.run_rc_control(joystick_left_y, joystick_right_x);
        rudder_angle = res_rudder;
    }

    return {rudder_angle, rpm};
}


void MotorboatAutopilotNode::update_ros_topics() {
    
    // Publish mode and telemetry info
    current_waypoint_index_publisher->publish(std_msgs::msg::Int32().set__data(motorboat_autopilot.get_current_waypoint_index()));

    autopilot_mode_publisher->publish(std_msgs::msg::UInt8().set__data(static_cast<uint8_t>(autopilot_mode)));


    // TODO LOOK INTO WHY WE ARE SENDING THIS AT ALL FOR THE MOTORBOAT AUTOPILOT NODE
    full_autonomy_maneuver_publisher->publish(std_msgs::msg::UInt8().set__data(255));

    // Calculate and Publish Rudder Angle
    auto [desired_rudder_angle, desired_rpm] = step();
    

    desired_rudder_angle_publisher->publish(std_msgs::msg::Float32().set__data(desired_rudder_angle));

    // Publish Current Target Heading for Telemetry
    if (autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(heading_to_hold));
    } 
    else if (autopilot_mode == MotorboatAutopilotMode::Waypoint_Mission && motorboat_autopilot.get_current_waypoints_list().size() > 0) {
        Position pos(current_longitude, current_latitude);
        int waypoint_index = motorboat_autopilot.get_current_waypoint_index();
        auto waypoints_list = motorboat_autopilot.get_current_waypoints_list();
        
        if (waypoint_index >= 0 && waypoint_index < (int)waypoints_list.size()) {
            float bearing = get_bearing(pos, waypoints_list[waypoint_index]);
            desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(bearing));
        } 
        else {
            desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(0.0f));
        }
    } 
    else {
        desired_heading_publisher->publish(std_msgs::msg::Float32().set__data(0.0f));
    }


    // Disconnect Safety Check
    auto now = this->get_clock()->now();
    double rc_dt = (now - last_rc_data_time).seconds();
    double gps_dt = (now - last_gps_position_received_time).seconds();
    double heading_dt = (now - last_heading_received_time).seconds();

    bool has_rc_disconnected = rc_dt >= autopilot_parameters["rc_data_failsafe_time"].get<double>();
    bool has_gps_disconnected = gps_dt >= autopilot_parameters["gps_position_failsafe_time"].get<double>();
    bool has_heading_disconnected = heading_dt >= autopilot_parameters["heading_data_failsafe_time"].get<double>();
    bool has_sensor_disconnected = has_rc_disconnected || has_gps_disconnected || has_heading_disconnected;

    if (has_sensor_disconnected) {
        propeller_motor_control_struct_publisher->publish(autoboat_msgs::msg::VESCControlData()
            .set__control_type_for_vesc("rpm")
            .set__control_value(0.0)
        );
    }
    else if (autopilot_mode == MotorboatAutopilotMode::Full_RC || autopilot_mode == MotorboatAutopilotMode::Hold_Heading) {
        if (propeller_motor_control_mode == MotorboatControls::RPM) {
            propeller_motor_control_struct_publisher->publish(autoboat_msgs::msg::VESCControlData()
                .set__control_type_for_vesc("rpm")
                .set__control_value(100.0 * joystick_left_y)
            );
        } 
        else if (propeller_motor_control_mode == MotorboatControls::DUTY_CYCLE) {
            propeller_motor_control_struct_publisher->publish(autoboat_msgs::msg::VESCControlData()
                .set__control_type_for_vesc("duty_cycle")
                .set__control_value(joystick_left_y)
            );
        } 
        else if (propeller_motor_control_mode == MotorboatControls::CURRENT) {
            propeller_motor_control_struct_publisher->publish(autoboat_msgs::msg::VESCControlData()
                .set__control_type_for_vesc("current")
                .set__control_value(joystick_left_y)
            );
        }
    }
    else if (autopilot_mode == MotorboatAutopilotMode::Waypoint_Mission) {
        propeller_motor_control_struct_publisher->publish(autoboat_msgs::msg::VESCControlData()
            .set__control_type_for_vesc("rpm")
            .set__control_value(0.0)
        );
    }
    else {
        propeller_motor_control_struct_publisher->publish(autoboat_msgs::msg::VESCControlData()
            .set__control_type_for_vesc("rpm")
            .set__control_value(0.0)
        );
    }


    // Zero Encoder Handling
    if (should_zero_encoder) {
        zero_rudder_encoder_publisher->publish(std_msgs::msg::Bool().set__data(true));
        encoder_has_been_zeroed = true;
        should_zero_encoder = false; // Reset flag after publishing
    }


    // Should Propeller Motor Be Powered
    should_propeller_motor_be_powered_publisher->publish(std_msgs::msg::Bool().set__data(should_propeller_motor_be_powered));
}




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}