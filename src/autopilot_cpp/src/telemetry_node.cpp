#include "telemetry_node.hpp"


TelemetryNode::TelemetryNode(const std::string &telemetry_server_url_string) : Node("telemetry_cpp") {
    
    telemetry_server_url_ = cpr::Url{telemetry_server_url_string};
    telemetry_server_url_string_ = telemetry_server_url_string;

    write_boat_status_to_telemetry_server_session_.SetUrl(telemetry_server_url_);
    read_waypoints_from_telemetry_server_session_.SetUrl(telemetry_server_url_);
    read_autopilot_parameters_from_telemetry_server_session_.SetUrl(telemetry_server_url_);


    // default values
    current_waypoint_index_ = 0;
    position_latitude_ = 0.0;
    position_longitude_ = 0.0;

    autopilot_mode_ = "N/A";
    full_autonomy_maneuver_ = "N/A";

    velocity_vector_x_ = 0.0;
    velocity_vector_y_ = 0.0;
    speed_ = 0.0;
    heading_ = 0.0;
    desired_heading_ = 0.0;

    apparent_wind_x_ = 0.0;
    apparent_wind_y_ = 0.0;
    apparent_wind_speed_ = 0.0;
    apparent_wind_angle_ = 0.0;

    desired_sail_angle_ = 0.0;
    desired_rudder_angle_ = 0.0;

    // vesc telemetry
    vesc_rpm_ = 0;

    
    
    // this is like while (true) but this will stop if a crl+c signal is received or if the node is forced to shutdown for whatever reason
    while (rclcpp::ok()) {
        json response = get_response_from_telemetry_server("instance_manager/create");

        if (response.is_number_integer()) {
            instance_id_ = response.get<int>();
            
            // set user (use environment USER if present)
            const char* user_env = std::getenv("USER");
            std::string user;

            
            if (user_env) 
                user = user_env;
            else 
                user = "unknown";                    

            post_json_to_telemetry_server("instance_manager/set_user/" + std::to_string(instance_id_) + "/" + user, json{});
            RCLCPP_INFO(this->get_logger(), "Created new telemetry server instance with ID %d", instance_id_);
            
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Retrying instance creation in 500ms...");
        std::this_thread::sleep_for(500ms);
    }



    // timers
    boat_status_timer_ = this->create_wall_timer(1ms, std::bind(&TelemetryNode::write_boat_status_to_telemetry_server, this));
    waypoints_timer_ = this->create_wall_timer(500ms, std::bind(&TelemetryNode::read_waypoints_from_telemetry_server, this));
    autopilot_params_timer_ = this->create_wall_timer(500ms, std::bind(&TelemetryNode::read_autopilot_parameters_from_telemetry_server, this));

    // publishers
    autopilot_parameters_pub_ = this->create_publisher<std_msgs::msg::String>("/autopilot_parameters", 10);
    sensors_parameters_pub_ = this->create_publisher<std_msgs::msg::String>("/sensors_parameters", 10);
    waypoints_list_pub_ = this->create_publisher<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10);

    // subscriptions
    desired_heading_sub_ = this->create_subscription<std_msgs::msg::Float32>("/desired_heading", 10, std::bind(&TelemetryNode::desired_heading_callback, this, std::placeholders::_1));
    current_waypoint_index_sub_ = this->create_subscription<std_msgs::msg::Int32>("/current_waypoint_index", 10, std::bind(&TelemetryNode::current_waypoint_index_callback, this, std::placeholders::_1));
    full_autonomy_maneuver_sub_ = this->create_subscription<std_msgs::msg::String>("/full_autonomy_maneuver", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::full_autonomy_maneuver_callback, this, std::placeholders::_1));
    autopilot_mode_sub_ = this->create_subscription<std_msgs::msg::String>("/autopilot_mode", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::autopilot_mode_callback, this, std::placeholders::_1));
    position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/position", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::position_callback, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/velocity", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::velocity_callback, this, std::placeholders::_1));
    heading_sub_ = this->create_subscription<std_msgs::msg::Float32>("/heading", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::heading_callback, this, std::placeholders::_1));
    apparent_wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/apparent_wind_vector", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::apparent_wind_vector_callback, this, std::placeholders::_1));
    vesc_telemetry_sub_ = this->create_subscription<autoboat_msgs::msg::VESCTelemetryData>("/vesc_telemetry_data", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::vesc_telemetry_data_callback, this, std::placeholders::_1));
    desired_sail_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>("/desired_sail_angle", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::desired_sail_angle_callback, this, std::placeholders::_1));
    desired_rudder_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>("/desired_rudder_angle", rclcpp::SensorDataQoS(), std::bind(&TelemetryNode::desired_rudder_angle_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TelemetryNode (C++) initialized");
}





// -------------------------------------------
// CALLBACKS
// -------------------------------------------
void TelemetryNode::position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    position_latitude_ = msg->latitude;
    position_longitude_ = msg->longitude;
}

void TelemetryNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    velocity_vector_x_ = msg->linear.x;
    velocity_vector_y_ = msg->linear.y;
    speed_ = std::hypot(velocity_vector_x_, velocity_vector_y_);
}

void TelemetryNode::heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    heading_ = msg->data;
}

void TelemetryNode::apparent_wind_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    apparent_wind_x_ = msg->x;
    apparent_wind_y_ = msg->y;
    std::tie(apparent_wind_speed_, apparent_wind_angle_) = cartesian_vector_to_polar(apparent_wind_x_, apparent_wind_y_);
}

void TelemetryNode::desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    desired_heading_ = msg->data;
}

void TelemetryNode::vesc_telemetry_data_callback(const autoboat_msgs::msg::VESCTelemetryData::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    vesc_rpm_ = msg->rpm;
// TODO map other fields
}

void TelemetryNode::current_waypoint_index_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    current_waypoint_index_ = msg->data;
}

void TelemetryNode::full_autonomy_maneuver_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    full_autonomy_maneuver_ = msg->data;
}

void TelemetryNode::autopilot_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    autopilot_mode_ = msg->data;
}

void TelemetryNode::desired_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    desired_sail_angle_ = msg->data;
}

void TelemetryNode::desired_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    desired_rudder_angle_ = msg->data;
}







std::pair<double,double> TelemetryNode::cartesian_vector_to_polar(double x, double y) {
    double r = std::hypot(x,y);
    double theta = std::atan2(y,x); // radians
    double theta_deg = theta * 180.0 / M_PI;
    return {r, theta_deg};
}


// TODO: FIX THIS
double TelemetryNode::get_distance_between_positions(double lat1, double lon1, double lat2, double lon2) {
    // Haversine in meters
    const double R = 6371000.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat/2)*sin(dlat/2) + cos(lat1*M_PI/180.0)*cos(lat2*M_PI/180.0)*sin(dlon/2)*sin(dlon/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return R * c;
}








json TelemetryNode::get_response_from_telemetry_server(const std::string &route, cpr::Session *session) {
    
    cpr::Response response;

    if (session != nullptr) {
        session->SetUrl(telemetry_server_url_string_ + "/" + route);
        response = session->Get();
    }
    else {
        response = cpr::Get(cpr::Url{telemetry_server_url_string_ + "/" + route});
    }


    if (response.error.code != cpr::ErrorCode::OK || response.status_code != 200) {
        RCLCPP_WARN(this->get_logger(), "Could not connect to telemetry server route %s", route.c_str());
        return json();
    }

    try {
        return json::parse(response.text);
    } 
    
    catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "JSON parse error: %s", e.what());
        return json();
    }

    return json();
}



bool TelemetryNode::post_json_to_telemetry_server(const std::string &route, const json &json_to_post, cpr::Session *session) {
    
    cpr::Response response;
    
    if (session != nullptr) {
        session->SetUrl(telemetry_server_url_string_ + "/" + route);
        session->SetBody(cpr::Body{json_to_post.dump()});
        session->SetHeader({{"Content-Type", "application/json"}});
        response = session->Post();
    }

    else {
        response = cpr::Post(
            cpr::Url{telemetry_server_url_string_ + "/" + route}, 
            cpr::Body{json_to_post.dump()},
            cpr::Header{{"Content-Type", "application/json"}}
        );
    }

    if (response.error || response.status_code != 200) {
        RCLCPP_WARN(this->get_logger(), "Could not connect to telemetry server route %s", (telemetry_server_url_string_ + "/" + route).c_str());
        return false;
    }

    return true;
}





// ------------------------------------------------------------
// PERIODIC TASKS TIED TO TIMERS
// ------------------------------------------------------------
void TelemetryNode::write_boat_status_to_telemetry_server() {
    
    auto start = std::chrono::steady_clock::now();

    // gather true wind and distance to waypoint
    double true_wind_x, true_wind_y, true_wind_speed, true_wind_angle;
    
    {
        std::lock_guard<std::mutex> lk(mutex_);
        true_wind_x = apparent_wind_x_ + velocity_vector_x_;
        true_wind_y = apparent_wind_y_ + velocity_vector_y_;
        std::tie(true_wind_speed, true_wind_angle) = cartesian_vector_to_polar(true_wind_x, true_wind_y);
    }


    double distance_to_next_waypoint = 0.0;
    if (!current_waypoints_list_.empty() && current_waypoint_index_ >= 0 && current_waypoint_index_ < (int)current_waypoints_list_.size()) {
        double lat2 = current_waypoints_list_[current_waypoint_index_].first;
        double lon2 = current_waypoints_list_[current_waypoint_index_].second;
        distance_to_next_waypoint = get_distance_between_positions(position_latitude_, position_longitude_, lat2, lon2);
    }


    json boat_status_json = json::object(); 

    {
        std::lock_guard<std::mutex> lk(mutex_);
        boat_status_json["position"] = {position_latitude_, position_longitude_};
        boat_status_json["state"] = autopilot_mode_;
        boat_status_json["full_autonomy_maneuver"] = full_autonomy_maneuver_;
        boat_status_json["speed"] = speed_;
        boat_status_json["velocity_vector"] = {velocity_vector_x_, velocity_vector_y_};
        boat_status_json["bearing"] = desired_heading_;
        boat_status_json["heading"] = heading_;
        boat_status_json["true_wind_speed"] = true_wind_speed;
        boat_status_json["true_wind_angle"] = true_wind_angle;
        boat_status_json["apparent_wind_speed"] = apparent_wind_speed_;
        boat_status_json["apparent_wind_angle"] = apparent_wind_angle_;
        boat_status_json["sail_angle"] = desired_sail_angle_;
        boat_status_json["rudder_angle"] = desired_rudder_angle_;
        boat_status_json["current_waypoint_index"] = current_waypoint_index_;
        boat_status_json["distance_to_next_waypoint"] = distance_to_next_waypoint;

        // vesc telemetry
        boat_status_json["vesc_telemetry_data_rpm"] = vesc_rpm_;
    }


    // POST to telemetry server (ignore failures)
    bool ok = post_json_to_telemetry_server("boat_status/set/" + std::to_string(instance_id_), boat_status_json, &write_boat_status_to_telemetry_server_session_);
    if (!ok) RCLCPP_WARN(this->get_logger(), "Failed to POST boat status");



    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = end - start;
    std::cout << "Time elapsed: " << elapsed_ms.count() << " ms" << std::endl;

}


void TelemetryNode::read_waypoints_from_telemetry_server() {

    json response = get_response_from_telemetry_server("waypoints/get_new/" + std::to_string(instance_id_), &read_waypoints_from_telemetry_server_session_);
    
    if (response.is_object() && response.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "No new waypoints received from telemetry server.");
        return;
    }
    if (!response.is_array()) {
        RCLCPP_WARN(this->get_logger(), "Invalid waypoints format: expected array");
        return;
    }

    autoboat_msgs::msg::WaypointList waypoint_list_msg;
    std::vector<std::pair<double,double>> new_waypoints;

    for (const auto &w : response) {
        if (!w.is_array() || w.size() != 2) {
        RCLCPP_WARN(this->get_logger(), "Invalid waypoint entry, skipping");
        continue;
        }
        double lat = w[0].get<double>();
        double lon = w[1].get<double>();
        sensor_msgs::msg::NavSatFix nav;
        nav.latitude = lat;
        nav.longitude = lon;
        waypoint_list_msg.waypoints.push_back(nav);
        new_waypoints.emplace_back(lat, lon);
    }

    {
        std::lock_guard<std::mutex> lk(mutex_);
        current_waypoints_list_ = new_waypoints;
    }

    waypoints_list_pub_->publish(waypoint_list_msg);

}

void TelemetryNode::read_autopilot_parameters_from_telemetry_server() {
    json response = get_response_from_telemetry_server("autopilot_parameters/get_new/" + std::to_string(instance_id_), &read_autopilot_parameters_from_telemetry_server_session_);

    if (response.is_object() && response.empty()) return;

    std_msgs::msg::String msg;
    msg.data = response.dump();
    autopilot_parameters_pub_->publish(msg);

}










int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
