#include "telemetry_node.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;



TelemetryNode::TelemetryNode() : Node("telemetry_cpp") {

    telemetry_server_url_string = "https://vt-autoboat-telemetry.uk";
    telemetry_server_url = cpr::Url{telemetry_server_url_string};
    
    write_boat_status_session.SetUrl(telemetry_server_url);
    read_waypoints_session.SetUrl(telemetry_server_url);
    read_autopilot_parameters_session.SetUrl(telemetry_server_url);

    // Subscriptions
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto latch_qos = rclcpp::QoS(1).reliable().transient_local();
    
    
    subs.push_back(create_subscription<std_msgs::msg::String>("/autopilot_param_config_path", latch_qos, std::bind(&TelemetryNode::config_path_callback, this, _1)));

    subs.push_back(create_subscription<std_msgs::msg::Float32>("/desired_heading", 10, std::bind(&TelemetryNode::desired_heading_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Int32>("/current_waypoint_index", 10, std::bind(&TelemetryNode::waypoint_index_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::UInt8>("/full_autonomy_maneuver", sensor_qos, std::bind(&TelemetryNode::full_autonomy_maneuver_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::UInt8>("/autopilot_mode", sensor_qos, std::bind(&TelemetryNode::autopilot_mode_callback, this, _1)));
    
    subs.push_back(create_subscription<std_msgs::msg::Float32>("/desired_sail_angle", sensor_qos, std::bind(&TelemetryNode::desired_sail_angle_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Float32>("/desired_rudder_angle", sensor_qos, std::bind(&TelemetryNode::desired_rudder_angle_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Float32>("/current_sail_angle", sensor_qos, std::bind(&TelemetryNode::current_sail_angle_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Float32>("/current_rudder_angle", sensor_qos, std::bind(&TelemetryNode::current_rudder_angle_callback, this, _1)));

    subs.push_back(create_subscription<sensor_msgs::msg::NavSatFix>("/position", sensor_qos, std::bind(&TelemetryNode::position_callback, this, _1)));
    subs.push_back(create_subscription<geometry_msgs::msg::Twist>("/velocity", sensor_qos, std::bind(&TelemetryNode::velocity_callback, this, _1)));
    subs.push_back(create_subscription<std_msgs::msg::Float32>("/heading", sensor_qos, std::bind(&TelemetryNode::heading_callback, this, _1)));
    subs.push_back(create_subscription<geometry_msgs::msg::Vector3>("/apparent_wind_vector", sensor_qos, std::bind(&TelemetryNode::apparent_wind_callback, this, _1)));
    subs.push_back(create_subscription<autoboat_msgs::msg::VESCTelemetryData>("/vesc_telemetry_data", sensor_qos, std::bind(&TelemetryNode::vesc_data_callback, this, _1)));

    // Publishers
    autopilot_parameters_publisher = create_publisher<std_msgs::msg::String>("/autopilot_parameters", 10);
    sensors_parameters_publisher = create_publisher<std_msgs::msg::String>("/sensors_parameters", 10);
    waypoints_list_publisher = create_publisher<autoboat_msgs::msg::WaypointList>("/waypoints_list", 10);

    RCLCPP_INFO(this->get_logger(), "Waiting for autopilot configuration...");
}


std::string TelemetryNode::sha256(const std::string& str) {
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int lengthOfHash = 0;

    EVP_MD_CTX* context = EVP_MD_CTX_new();
    if(context != nullptr) {
        if(EVP_DigestInit_ex(context, EVP_sha256(), nullptr)) {
            if(EVP_DigestUpdate(context, str.c_str(), str.size())) {
                EVP_DigestFinal_ex(context, hash, &lengthOfHash);
            }
        }
        EVP_MD_CTX_free(context);
    }

    std::stringstream ss;
    for(unsigned int i = 0; i < lengthOfHash; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
    }
    return ss.str();
}

void TelemetryNode::config_path_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (params_loaded) return;
    std::string path = msg->data;
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open config file: %s", path.c_str());
        return;
    }

    try {
        autopilot_parameters = json::parse(file);
    } 
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "JSON Parse error: %s", e.what());
        return;
    }

    if (path.find("sailboat_default_parameters") != std::string::npos) {
        is_sailboat_mode = true;
        is_motorboat_mode = false;
    } 
    else if (path.find("motorboat_default_parameters") != std::string::npos) {
        is_sailboat_mode = false;
        is_motorboat_mode = true;
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Unrecognized parameter config name.");
        return;
    }

    params_loaded = true;
    RCLCPP_INFO(this->get_logger(), "Loaded parameters from: %s", path.c_str());
    
    initialize_server_connection();
}

void TelemetryNode::initialize_server_connection() {
    std::thread([this]() {
        std::string dump_str = autopilot_parameters.dump(-1, ' ', false, nlohmann::detail::error_handler_t::replace);
        std::string config_hash = sha256(dump_str);

        while (rclcpp::ok() && instance_id < 0) {
            json response = get_response_from_telemetry_server("instance_manager/create");
            if (response.is_number_integer()) {
                instance_id = response.get<int>();

                const char* user_env = std::getenv("USER");
                std::string user = user_env ? user_env : "unknown";
                std::replace(user.begin(), user.end(), ' ', '_');
                post_empty_to_telemetry_server("instance_manager/set_user/" + std::to_string(instance_id) + "/" + user, &write_boat_status_session);

                json mapping = is_sailboat_mode ? get_sailboat_mapping_raw() : get_motorboat_mapping_raw();
                post_json_to_telemetry_server("boat_status/set_mapping/" + std::to_string(instance_id), mapping, &write_boat_status_session);

                send_boat_status();

                json hash_response = get_response_from_telemetry_server("autopilot_parameters/get_hash_exists/" + config_hash, &read_autopilot_parameters_session);
                bool does_hash_exist = hash_response.is_boolean() && hash_response.get<bool>();

                if (!does_hash_exist) {
                    json stringified_params = autopilot_parameters.dump(-1, ',', false, nlohmann::detail::error_handler_t::replace);
                    post_json_to_telemetry_server("autopilot_parameters/set_default/" + std::to_string(instance_id), stringified_params, &read_autopilot_parameters_session);
                } 
                else {
                    post_empty_to_telemetry_server("autopilot_parameters/set_default_from_hash/" + std::to_string(instance_id) + "/" + config_hash, &read_autopilot_parameters_session);
                }

                RCLCPP_INFO(this->get_logger(), "Instance ID: %d, Hash: %s", instance_id, config_hash.c_str());
                break;
            }
            std::this_thread::sleep_for(1s);
        }

        if (instance_id >= 0) {
            boat_status_timer = create_wall_timer(10ms, std::bind(&TelemetryNode::send_boat_status, this));
            waypoints_timer = create_wall_timer(500ms, std::bind(&TelemetryNode::update_waypoints, this));
            autopilot_params_timer = create_wall_timer(500ms, std::bind(&TelemetryNode::update_autopilot_params, this));
        }
    }).detach();
}

void TelemetryNode::send_boat_status() {
    if (instance_id < 0) return;

    float true_wind_x = velocity_vector_x + apparent_wind_x;
    float true_wind_y = velocity_vector_y + apparent_wind_y;
    float true_wind_speed = std::hypot(true_wind_x, true_wind_y);
    float true_wind_angle = std::atan2(true_wind_y, true_wind_x) * 180.0f / static_cast<float>(M_PI);

    float distance_to_next_waypoint = 0.0f;
    if (!current_waypoints_list.empty() && current_waypoint_index >= 0 && current_waypoint_index < (int)current_waypoints_list.size()) {
        double latitude2 = current_waypoints_list[current_waypoint_index].first;
        double longitude2 = current_waypoints_list[current_waypoint_index].second;
        distance_to_next_waypoint = get_distance(position_latitude, position_longitude, latitude2, longitude2);
    }

    std::string route = "boat_status/set_fast/" + std::to_string(instance_id);

    if (is_sailboat_mode) {
        SailboatStatusPayload payload;
        payload.latitude = position_latitude;
        payload.longitude = position_longitude;
        payload.distance_to_next_waypoint = distance_to_next_waypoint;
        payload.speed = std::hypot(velocity_vector_x, velocity_vector_y);
        payload.velocity_x = velocity_vector_x;
        payload.velocity_y = velocity_vector_y;
        payload.desired_heading = desired_heading;
        payload.heading = heading;
        payload.desired_rudder_angle = desired_rudder_angle;
        payload.current_rudder_angle = current_rudder_angle;
        payload.rudder_angle_error = desired_rudder_angle - current_rudder_angle;
        payload.current_waypoint_index = current_waypoint_index;
        payload.autopilot_mode = autopilot_mode;

        payload.true_wind_speed = true_wind_speed;
        payload.true_wind_angle = true_wind_angle;
        payload.apparent_wind_speed = apparent_wind_speed;
        payload.apparent_wind_angle = apparent_wind_angle;
        payload.current_sail_angle = current_sail_angle;
        payload.desired_sail_angle = desired_sail_angle;
        payload.sail_angle_error = desired_sail_angle - current_sail_angle;
        payload.full_autonomy_maneuver = full_autonomy_maneuver;

        post_bytes(route, (const char*)&payload, sizeof(payload), &write_boat_status_session);
    } 
    else if (is_motorboat_mode) {
        MotorboatStatusPayload payload;
        payload.latitude = position_latitude;
        payload.longitude = position_longitude;
        payload.distance_to_next_waypoint = distance_to_next_waypoint;
        payload.speed = std::hypot(velocity_vector_x, velocity_vector_y);
        payload.velocity_x = velocity_vector_x;
        payload.velocity_y = velocity_vector_y;
        payload.desired_heading = desired_heading;
        payload.heading = heading;
        payload.desired_rudder_angle = desired_rudder_angle;
        payload.current_rudder_angle = current_rudder_angle;
        payload.rudder_angle_error = desired_rudder_angle - current_rudder_angle;
        payload.current_waypoint_index = current_waypoint_index;
        payload.autopilot_mode = autopilot_mode;

        payload.rpm = vesc_rpm;
        payload.duty_cycle = vesc_duty_cycle;
        payload.amp_hours = vesc_amp_hours;
        payload.amp_hours_charged = vesc_amp_hours_charged;
        payload.current_to_vesc = vesc_current_to_vesc;
        payload.voltage_to_motor = vesc_voltage_to_motor;
        payload.voltage_to_vesc = vesc_voltage_to_vesc;
        payload.wattage_to_motor = vesc_wattage_to_motor;
        payload.motor_temperature = vesc_motor_temperature;
        payload.vesc_temperature = vesc_vesc_temperature;
        payload.time_since_vesc_startup_in_ms = vesc_time_ms;

        post_bytes(route, (const char*)&payload, sizeof(payload), &write_boat_status_session);
    }
}

void TelemetryNode::post_bytes(const std::string& route, const char* data, size_t size, cpr::Session* session) {
    std::string url = telemetry_server_url_string + "/" + route;
    std::string body_string = std::string(data, size);

    while (rclcpp::ok()) {
        cpr::Response response;
        if (session) {
            session->SetUrl(cpr::Url{url});
            session->SetBody(cpr::Body{body_string});
            session->SetHeader(cpr::Header{{"Content-Type", "application/octet-stream"}});
            session->SetTimeout(cpr::Timeout{10000});
            response = session->Post();
        } 
        else {
            response = cpr::Post(
                cpr::Url{url},
                cpr::Body{body_string},
                cpr::Header{{"Content-Type", "application/octet-stream"}},
                cpr::Timeout{10000}
            );
        }
        if (response.status_code == 200 || response.status_code == 201) return;
        RCLCPP_WARN(this->get_logger(), "Failed to send payload bytes to %s, retrying...", route.c_str());
        std::this_thread::sleep_for(1s);
    }
}

void TelemetryNode::update_waypoints() {
    json response = get_response_from_telemetry_server("waypoints/get_new/" + std::to_string(instance_id), &read_waypoints_session);
    if (!response.is_array() || response.empty()) return;

    autoboat_msgs::msg::WaypointList waypoint_list;
    current_waypoints_list.clear();
    for (const auto& waypoint : response) {
        if (waypoint.is_array() && waypoint.size() >= 2) {
            double latitude = waypoint[0].get<double>();
            double longitude = waypoint[1].get<double>();
            current_waypoints_list.push_back({latitude, longitude});

            waypoint_list.waypoints.push_back(
                sensor_msgs::msg::NavSatFix()
                .set__latitude(latitude)
                .set__longitude(longitude)
            );
        }
    }
    waypoints_list_publisher->publish(waypoint_list);
}

void TelemetryNode::update_autopilot_params() {
    json response = get_response_from_telemetry_server("autopilot_parameters/get_new/" + std::to_string(instance_id), &read_autopilot_parameters_session);
    if (response.is_object() && !response.empty()) {
        autopilot_parameters = response;
        autopilot_parameters_publisher->publish(std_msgs::msg::String().set__data(autopilot_parameters.dump()));
    }
}

json TelemetryNode::get_response_from_telemetry_server(const std::string& route, cpr::Session* session) {
    std::string url = telemetry_server_url_string + "/" + route;
    while (rclcpp::ok()) {
        cpr::Response response;
        if (session) {
            session->SetUrl(cpr::Url{url});
            session->SetTimeout(cpr::Timeout{10000});
            response = session->Get();
        } 
        else {
            response = cpr::Get(cpr::Url{url}, cpr::Timeout{10000});
        }
        if (response.status_code == 200) {
            try { 
                return json::parse(response.text); 
            }
            catch(...) {}

            return json();
        }
        RCLCPP_WARN(this->get_logger(), "Could not receive data %s, retrying... (Status: %ld)", route.c_str(), response.status_code);
        std::this_thread::sleep_for(1s);
    }
    return json();
}

void TelemetryNode::post_json_to_telemetry_server(const std::string& route, const json& j, cpr::Session* session) {
    std::string url = telemetry_server_url_string + "/" + route;
    std::string body_string = j.dump(-1, ',', false, nlohmann::detail::error_handler_t::replace);

    while (rclcpp::ok()) {
        cpr::Response response;
        if (session) {
            session->SetUrl(cpr::Url{url});
            session->SetBody(cpr::Body{body_string});
            session->SetHeader(cpr::Header{{"Content-Type", "application/json"}});
            session->SetTimeout(cpr::Timeout{10000});
            response = session->Post();
        } 
        else {
            response = cpr::Post(
                cpr::Url{url},
                cpr::Body{body_string},
                cpr::Header{{"Content-Type", "application/json"}},
                cpr::Timeout{10000}
            );
        }
        if (response.status_code == 200 || response.status_code == 201) return;
        RCLCPP_WARN(
            this->get_logger(), 
            "Could not post JSON to %s, retrying... Status: %ld, Error: %s, Response: %s", 
            route.c_str(), response.status_code, response.error.message.c_str(), response.text.c_str()
        );
        std::this_thread::sleep_for(1s);
    }
}

void TelemetryNode::post_empty_to_telemetry_server(const std::string& route, cpr::Session* session) {
    std::string url = telemetry_server_url_string + "/" + route;
    while (rclcpp::ok()) {
        cpr::Response response;
        if (session) {
            session->SetUrl(cpr::Url{url});
            session->SetBody(cpr::Body{""});
            session->SetHeader(cpr::Header{{"Content-Type", "application/json"}});
            session->SetTimeout(cpr::Timeout{10000});
            response = session->Post();
        } 
        else {
            response = cpr::Post(cpr::Url{url}, cpr::Header{{"Content-Type", "application/json"}}, cpr::Timeout{10000});
        }
        if (response.status_code == 200 || response.status_code == 201) return;
        RCLCPP_WARN(this->get_logger(), "Could not post empty to %s, retrying...", route.c_str());
        std::this_thread::sleep_for(1s);
    }
}







// Callbacks
void TelemetryNode::position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { 
    position_latitude = msg->latitude; 
    position_longitude = msg->longitude; 
}

void TelemetryNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { 
    velocity_vector_x = msg->linear.x; 
    velocity_vector_y = msg->linear.y; 
}

void TelemetryNode::heading_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
    heading = msg->data; 
}

void TelemetryNode::apparent_wind_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    apparent_wind_x = static_cast<float>(msg->x); 
    apparent_wind_y = static_cast<float>(msg->y);
    apparent_wind_speed = std::hypot(apparent_wind_x, apparent_wind_y);
    apparent_wind_angle = std::atan2(apparent_wind_y, apparent_wind_x) * 180.0f / static_cast<float>(M_PI);
}

void TelemetryNode::vesc_data_callback(const autoboat_msgs::msg::VESCTelemetryData::SharedPtr msg) {
    vesc_rpm = msg->rpm;
    vesc_duty_cycle = msg->duty_cycle;
    vesc_amp_hours = msg->amp_hours;
    vesc_amp_hours_charged = msg->amp_hours_charged;
    vesc_current_to_vesc = msg->current_to_vesc;
    vesc_voltage_to_motor = msg->voltage_to_motor;
    vesc_voltage_to_vesc = msg->voltage_to_vesc;
    vesc_wattage_to_motor = msg->wattage_to_motor;
    vesc_motor_temperature = msg->motor_temperature;
    vesc_vesc_temperature = msg->vesc_temperature;
    vesc_time_ms = msg->time_since_vesc_startup_in_ms;
}

void TelemetryNode::desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
    desired_heading = msg->data; 
}

void TelemetryNode::waypoint_index_callback(const std_msgs::msg::Int32::SharedPtr msg) { 
    current_waypoint_index = msg->data;
}

void TelemetryNode::full_autonomy_maneuver_callback(const std_msgs::msg::UInt8::SharedPtr msg) { 
    full_autonomy_maneuver = msg->data; 
}

void TelemetryNode::autopilot_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) { 
    autopilot_mode = msg->data; 
}

void TelemetryNode::desired_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
    desired_sail_angle = msg->data; 
}

void TelemetryNode::desired_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
    desired_rudder_angle = msg->data; 
}

void TelemetryNode::current_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
    current_sail_angle = msg->data; 
}

void TelemetryNode::current_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { 
    current_rudder_angle = msg->data; 
}




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
