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


using namespace std::chrono_literals;
using json = nlohmann::json;
using std::placeholders::_1;



class TelemetryNode : public rclcpp::Node {
public:
    TelemetryNode() : Node("telemetry_cpp"), telemetry_server_url_string("https://vt-autoboat-telemetry.uk") {
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
        subs.push_back(create_subscription<std_msgs::msg::String>("/full_autonomy_maneuver", sensor_qos, std::bind(&TelemetryNode::full_autonomy_maneuver_callback, this, _1)));
        subs.push_back(create_subscription<std_msgs::msg::String>("/autopilot_mode", sensor_qos, std::bind(&TelemetryNode::autopilot_mode_callback, this, _1)));
        
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

private:
    std::string sha256(const std::string& str) {
        unsigned char hash[SHA256_DIGEST_LENGTH];
        SHA256_CTX sha256;
        SHA256_Init(&sha256);
        SHA256_Update(&sha256, str.c_str(), str.size());
        SHA256_Final(hash, &sha256);
        std::stringstream ss;
        for(int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
        }
        return ss.str();
    }

    void config_path_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (params_loaded) return;
        std::string path = msg->data;
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Could not open config file: %s", path.c_str());
            return;
        }

        try {
            autopilot_parameters = json::parse(file);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "JSON Parse error: %s", e.what());
            return;
        }

        if (path.find("sailboat_default_parameters") != std::string::npos) {
            is_sailboat_mode = true;
            is_motorboat_mode = false;
        } else if (path.find("motorboat_default_parameters") != std::string::npos) {
            is_sailboat_mode = false;
            is_motorboat_mode = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unrecognized parameter config name.");
            return;
        }

        params_loaded = true;
        RCLCPP_INFO(this->get_logger(), "Loaded parameters from: %s", path.c_str());
        
        initialize_server_connection();
    }

    void initialize_server_connection() {
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
                    } else {
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

    void send_boat_status() {
        if (instance_id < 0) return;

        double tw_x = apparent_wind_x + velocity_vector_x;
        double tw_y = apparent_wind_y + velocity_vector_y;
        double r = std::hypot(tw_x, tw_y);
        double th = std::atan2(tw_y, tw_x) * 180.0 / M_PI;

        double dist_wp = 0.0;
        if (!current_waypoints_list.empty() && current_waypoint_index >= 0 && current_waypoint_index < (int)current_waypoints_list.size()) {
            double lat2 = current_waypoints_list[current_waypoint_index].first;
            double lon2 = current_waypoints_list[current_waypoint_index].second;
            dist_wp = calculate_distance(position_latitude, position_longitude, lat2, lon2);
        }

        std::string route = "boat_status/set_fast/" + std::to_string(instance_id);

        if (is_sailboat_mode) {
            SailboatStatusPayload p;
            p.latitude = position_latitude;
            p.longitude = position_longitude;
            p.distance_to_next_waypoint = dist_wp;
            p.speed = std::hypot(velocity_vector_x, velocity_vector_y);
            p.velocity_x = velocity_vector_x;
            p.velocity_y = velocity_vector_y;
            p.desired_heading = desired_heading;
            p.heading = heading;
            p.desired_rudder_angle = desired_rudder_angle;
            p.current_rudder_angle = current_rudder_angle;
            p.rudder_angle_error = desired_rudder_angle - current_rudder_angle;
            p.current_waypoint_index = current_waypoint_index;
            p.autopilot_mode = get_sailboat_mode(autopilot_mode);

            p.true_wind_speed = r;
            p.true_wind_angle = th;
            p.apparent_wind_speed = apparent_wind_speed;
            p.apparent_wind_angle = apparent_wind_angle;
            p.current_sail_angle = current_sail_angle;
            p.desired_sail_angle = desired_sail_angle;
            p.sail_angle_error = desired_sail_angle - current_sail_angle;
            p.full_autonomy_maneuver = get_sailboat_state(full_autonomy_maneuver);

            post_bytes(route, (const char*)&p, sizeof(p), &write_boat_status_session);
        } else if (is_motorboat_mode) {
            MotorboatStatusPayload p;
            p.latitude = position_latitude;
            p.longitude = position_longitude;
            p.distance_to_next_waypoint = dist_wp;
            p.speed = std::hypot(velocity_vector_x, velocity_vector_y);
            p.velocity_x = velocity_vector_x;
            p.velocity_y = velocity_vector_y;
            p.desired_heading = desired_heading;
            p.heading = heading;
            p.desired_rudder_angle = desired_rudder_angle;
            p.current_rudder_angle = current_rudder_angle;
            p.rudder_angle_error = desired_rudder_angle - current_rudder_angle;
            p.current_waypoint_index = current_waypoint_index;
            p.autopilot_mode = get_motorboat_mode(autopilot_mode);

            p.rpm = vesc_rpm;
            p.duty_cycle = vesc_duty_cycle;
            p.amp_hours = vesc_amp_hours;
            p.amp_hours_charged = vesc_amp_hours_charged;
            p.current_to_vesc = vesc_current_to_vesc;
            p.voltage_to_motor = vesc_voltage_to_motor;
            p.voltage_to_vesc = vesc_voltage_to_vesc;
            p.wattage_to_motor = vesc_wattage_to_motor;
            p.motor_temperature = vesc_motor_temperature;
            p.vesc_temperature = vesc_vesc_temperature;
            p.time_since_vesc_startup_in_ms = vesc_time_ms;

            post_bytes(route, (const char*)&p, sizeof(p), &write_boat_status_session);
        }
    }

    void post_bytes(const std::string& route, const char* data, size_t size, cpr::Session* session = nullptr) {
        std::string url = telemetry_server_url_string + "/" + route;
        std::string body_str = std::string(data, size);

        while (rclcpp::ok()) {
            cpr::Response r;
            if (session) {
                session->SetUrl(cpr::Url{url});
                session->SetBody(cpr::Body{body_str});
                session->SetHeader(cpr::Header{{"Content-Type", "application/octet-stream"}});
                session->SetTimeout(cpr::Timeout{10000});
                r = session->Post();
            } else {
                r = cpr::Post(
                    cpr::Url{url},
                    cpr::Body{body_str},
                    cpr::Header{{"Content-Type", "application/octet-stream"}},
                    cpr::Timeout{10000}
                );
            }
            if (r.status_code == 200 || r.status_code == 201) return;
            RCLCPP_WARN(this->get_logger(), "Failed to send payload bytes to %s, retrying...", route.c_str());
            std::this_thread::sleep_for(1s);
        }
    }

    void update_waypoints() {
        json resp = get_response_from_telemetry_server("waypoints/get_new/" + std::to_string(instance_id), &read_waypoints_session);
        if (!resp.is_array() || resp.empty()) return;

        autoboat_msgs::msg::WaypointList wp_list;
        current_waypoints_list.clear();
        for (const auto& w : resp) {
            if (w.is_array() && w.size() >= 2) {
                double lat = w[0].get<double>();
                double lon = w[1].get<double>();
                current_waypoints_list.push_back({lat, lon});
                sensor_msgs::msg::NavSatFix nav;
                nav.latitude = lat;
                nav.longitude = lon;
                wp_list.waypoints.push_back(nav);
            }
        }
        waypoints_list_publisher->publish(wp_list);
    }

    void update_autopilot_params() {
        json resp = get_response_from_telemetry_server("autopilot_parameters/get_new/" + std::to_string(instance_id), &read_autopilot_parameters_session);
        if (resp.is_object() && !resp.empty()) {
            autopilot_parameters = resp;
            std_msgs::msg::String msg;
            msg.data = autopilot_parameters.dump();
            autopilot_parameters_publisher->publish(msg);
        }
    }

    json get_response_from_telemetry_server(const std::string& route, cpr::Session* session = nullptr) {
        std::string url = telemetry_server_url_string + "/" + route;
        while (rclcpp::ok()) {
            cpr::Response r;
            if (session) {
                session->SetUrl(cpr::Url{url});
                session->SetTimeout(cpr::Timeout{10000});
                r = session->Get();
            } else {
                r = cpr::Get(cpr::Url{url}, cpr::Timeout{10000});
            }
            if (r.status_code == 200) {
                try { return json::parse(r.text); } catch(...) {}
                return json();
            }
            RCLCPP_WARN(this->get_logger(), "Could not receive data %s, retrying... (Status: %ld)", route.c_str(), r.status_code);
            std::this_thread::sleep_for(1s);
        }
        return json();
    }

    void post_json_to_telemetry_server(const std::string& route, const json& j, cpr::Session* session = nullptr) {
        std::string url = telemetry_server_url_string + "/" + route;
        std::string body_str = j.dump(-1, ',', false, nlohmann::detail::error_handler_t::replace);
        while (rclcpp::ok()) {
            cpr::Response r;
            if (session) {
                session->SetUrl(cpr::Url{url});
                session->SetBody(cpr::Body{body_str});
                session->SetHeader(cpr::Header{{"Content-Type", "application/json"}});
                session->SetTimeout(cpr::Timeout{10000});
                r = session->Post();
            } else {
                r = cpr::Post(
                    cpr::Url{url},
                    cpr::Body{body_str},
                    cpr::Header{{"Content-Type", "application/json"}},
                    cpr::Timeout{10000}
                );
            }
            if (r.status_code == 200 || r.status_code == 201) return;
            RCLCPP_WARN(this->get_logger(), "Could not post JSON to %s, retrying... Status: %ld, Error: %s, Response: %s", route.c_str(), r.status_code, r.error.message.c_str(), r.text.c_str());
            std::this_thread::sleep_for(1s);
        }
    }
    
    void post_empty_to_telemetry_server(const std::string& route, cpr::Session* session = nullptr) {
        std::string url = telemetry_server_url_string + "/" + route;
        while (rclcpp::ok()) {
            cpr::Response r;
            if (session) {
                session->SetUrl(cpr::Url{url});
                session->SetBody(cpr::Body{""});
                session->SetHeader(cpr::Header{{"Content-Type", "application/json"}});
                session->SetTimeout(cpr::Timeout{10000});
                r = session->Post();
            } else {
                r = cpr::Post(cpr::Url{url}, cpr::Header{{"Content-Type", "application/json"}}, cpr::Timeout{10000});
            }
            if (r.status_code == 200 || r.status_code == 201) return;
            RCLCPP_WARN(this->get_logger(), "Could not post empty to %s, retrying...", route.c_str());
            std::this_thread::sleep_for(1s);
        }
    }

    double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000.0;
        double dlat = (lat2 - lat1) * M_PI / 180.0;
        double dlon = (lon2 - lon1) * M_PI / 180.0;
        double a = sin(dlat/2)*sin(dlat/2) + cos(lat1*M_PI/180.0)*cos(lat2*M_PI/180.0)*sin(dlon/2)*sin(dlon/2);
        return R * 2 * atan2(sqrt(a), sqrt(1-a));
    }

    // Callbacks
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { position_latitude = msg->latitude; position_longitude = msg->longitude; }
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { velocity_vector_x = msg->linear.x; velocity_vector_y = msg->linear.y; }
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) { heading = msg->data; }
    void apparent_wind_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        apparent_wind_x = msg->x; apparent_wind_y = msg->y;
        apparent_wind_speed = std::hypot(msg->x, msg->y);
        apparent_wind_angle = std::atan2(msg->y, msg->x) * 180.0 / M_PI;
    }
    void vesc_data_callback(const autoboat_msgs::msg::VESCTelemetryData::SharedPtr msg) {
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
    void desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg) { desired_heading = msg->data; }
    void waypoint_index_callback(const std_msgs::msg::Int32::SharedPtr msg) { current_waypoint_index = msg->data; }
    void full_autonomy_maneuver_callback(const std_msgs::msg::String::SharedPtr msg) { full_autonomy_maneuver = msg->data; }
    void autopilot_mode_callback(const std_msgs::msg::String::SharedPtr msg) { autopilot_mode = msg->data; }
    void desired_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { desired_sail_angle = msg->data; }
    void desired_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { desired_rudder_angle = msg->data; }
    void current_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_sail_angle = msg->data; }
    void current_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_rudder_angle = msg->data; }

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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
