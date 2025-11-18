#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <autoboat_msgs/msg/waypoint_list.hpp>
#include <autoboat_msgs/msg/vesc_telemetry_data.hpp>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <thread>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <vector>
#include <optional>

using namespace std::chrono_literals;
using json = nlohmann::json;


// Helper for curl write callback
static size_t curl_write_callback_(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

class TelemetryNode : public rclcpp::Node {
public:
    explicit TelemetryNode(const std::string &telemetry_server_url = "https://vt-autoboat-telemetry.uk") : Node("telemetry_cpp"), 
    telemetry_server_url_(telemetry_server_url) {

        // default values
        current_waypoint_index_ = 0;
        position_lat_ = 0.0;
        position_lon_ = 0.0;

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



        // initialize curl
        curl_global_init(CURL_GLOBAL_DEFAULT);
        
        curl_get_raw_response_from_telemetry_server = curl_easy_init();
        curl_post_json = curl_easy_init();



        
        // create instance on server (blocking retry loop like python)
        while (rclcpp::ok()) {
            auto res = get_raw_response_from_telemetry_server("instance_manager/create");
            if (res.has_value()) {
                json j = res.value();
                if (j.is_number_integer()) {
                    instance_id_ = j.get<int>();
                    // set user (use environment USER if present)
                    const char* user_env = std::getenv("USER");
                    std::string user = user_env ? user_env : "unknown";
                    post_json(url_join("instance_manager/set_user/" + std::to_string(instance_id_) + "/" + user), json{});
                    RCLCPP_INFO(this->get_logger(), "Created new telemetry server instance with ID %d", instance_id_);
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Retrying instance creation in 500ms...");
            std::this_thread::sleep_for(500ms);
        }

        // timers
        boat_status_timer_ = this->create_wall_timer(1ms, std::bind(&TelemetryNode::update_boat_status, this));
        waypoints_timer_ = this->create_wall_timer(500ms, std::bind(&TelemetryNode::update_waypoints_from_telemetry, this));
        autopilot_params_timer_ = this->create_wall_timer(500ms, std::bind(&TelemetryNode::update_autopilot_parameters_from_telemetry, this));

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

    ~TelemetryNode() override {
        curl_global_cleanup();
    }



private:
    std::vector<std::pair<double,double>> current_waypoints_list_;
    int current_waypoint_index_;
    double position_lat_, position_lon_;

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
    std::string telemetry_server_url_;

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

    std::mutex mutex_;

    CURL* curl_get_raw_response_from_telemetry_server;
    CURL* curl_post_json;



    // --- helpers: HTTP requests using libcurl ---
    std::optional<json> get_raw_response_from_telemetry_server(const std::string &route) {
        std::string url = url_join(route);

        if (!curl_get_raw_response_from_telemetry_server) return std::nullopt;

        std::string readBuffer;
        
        curl_easy_setopt(curl_get_raw_response_from_telemetry_server, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_get_raw_response_from_telemetry_server, CURLOPT_TIMEOUT, 10L);
        curl_easy_setopt(curl_get_raw_response_from_telemetry_server, CURLOPT_WRITEFUNCTION, curl_write_callback_);
        curl_easy_setopt(curl_get_raw_response_from_telemetry_server, CURLOPT_WRITEDATA, &readBuffer);

        CURLcode res = curl_easy_perform(curl_get_raw_response_from_telemetry_server);

        
        if (res != CURLE_OK) {
            RCLCPP_WARN(this->get_logger(), "Could not connect to telemetry server route %s", route.c_str());
            return std::nullopt;
        }

        try {
            return json::parse(readBuffer);
        } 
        
        catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "JSON parse error: %s", e.what());
            return std::nullopt;
        }
    }

    bool post_json(const std::string &route, const json &j) {
        std::string url = url_join(route);
        std::string out = j.dump();

        if (!curl_post_json) return false;
        
        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl_post_json, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_post_json, CURLOPT_POSTFIELDS, out.c_str());
        curl_easy_setopt(curl_post_json, CURLOPT_POSTFIELDSIZE, (long)out.size());
        curl_easy_setopt(curl_post_json, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl_post_json, CURLOPT_TIMEOUT, 10L);

        CURLcode res = curl_easy_perform(curl_post_json);
        curl_slist_free_all(headers);

        if (res != CURLE_OK) {
            RCLCPP_WARN(this->get_logger(), "Could not POST to telemetry server route %s", route.c_str());
            return false;
        }
        return true;
    }

    std::string url_join(const std::string &route) const {
        if (route.rfind("http", 0) == 0) return route; // absolute
        
        std::string base = telemetry_server_url_;
        
        if (base.back() == '/') base.pop_back();
        
        std::string r = route;
        
        if (r.front() == '/') r.erase(0,1);
            return base + "/" + r;
    }

    // --- callbacks ---
    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        position_lat_ = msg->latitude;
        position_lon_ = msg->longitude;
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        velocity_vector_x_ = msg->linear.x;
        velocity_vector_y_ = msg->linear.y;
        speed_ = std::hypot(velocity_vector_x_, velocity_vector_y_);
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        heading_ = msg->data;
    }

    void apparent_wind_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        apparent_wind_x_ = msg->x;
        apparent_wind_y_ = msg->y;
        std::tie(apparent_wind_speed_, apparent_wind_angle_) = cartesian_vector_to_polar(apparent_wind_x_, apparent_wind_y_);
    }

    void desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        desired_heading_ = msg->data;
    }

    void vesc_telemetry_data_callback(const autoboat_msgs::msg::VESCTelemetryData::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        vesc_rpm_ = msg->rpm;
    // TODO map other fields
    }

    void current_waypoint_index_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        current_waypoint_index_ = msg->data;
    }

    void full_autonomy_maneuver_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        full_autonomy_maneuver_ = msg->data;
    }

    void autopilot_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        autopilot_mode_ = msg->data;
    }

    void desired_sail_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        desired_sail_angle_ = msg->data;
    }

    void desired_rudder_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        desired_rudder_angle_ = msg->data;
    }

    // --- utilities ---
    std::pair<double,double> cartesian_vector_to_polar(double x, double y) {
        double r = std::hypot(x,y);
        double theta = std::atan2(y,x); // radians
        double theta_deg = theta * 180.0 / M_PI;
        return {r, theta_deg};
    }

    double get_distance_between_positions(double lat1, double lon1, double lat2, double lon2) {
        // Haversine in meters
        const double R = 6371000.0;
        double dlat = (lat2 - lat1) * M_PI / 180.0;
        double dlon = (lon2 - lon1) * M_PI / 180.0;
        double a = sin(dlat/2)*sin(dlat/2) + cos(lat1*M_PI/180.0)*cos(lat2*M_PI/180.0)*sin(dlon/2)*sin(dlon/2);
        double c = 2*atan2(sqrt(a), sqrt(1-a));
        return R * c;
    }

    // --- periodic tasks ---
    void update_boat_status() {
        RCLCPP_INFO(this->get_logger(), "hi");
        // gather true wind and distance to waypoint
        double true_wind_x, true_wind_y, true_wind_speed, true_wind_angle;
        
        {
            std::lock_guard<std::mutex> lk(mutex_);
            true_wind_x = apparent_wind_x_ + velocity_vector_x_;
            true_wind_y = apparent_wind_y_ + velocity_vector_y_;
            std::tie(true_wind_speed, true_wind_angle) = cartesian_vector_to_polar(true_wind_x, true_wind_y);
        }

        RCLCPP_INFO(this->get_logger(), "hi2");

        double distance_to_next_waypoint = 0.0;
        if (!current_waypoints_list_.empty() && current_waypoint_index_ >= 0 && current_waypoint_index_ < (int)current_waypoints_list_.size()) {
            double lat2 = current_waypoints_list_[current_waypoint_index_].first;
            double lon2 = current_waypoints_list_[current_waypoint_index_].second;
            distance_to_next_waypoint = get_distance_between_positions(position_lat_, position_lon_, lat2, lon2);
        }

        RCLCPP_INFO(this->get_logger(), "hi3");

        json boat_status = json::object(); 

        {
            std::lock_guard<std::mutex> lk(mutex_);
            boat_status["position"] = {position_lat_, position_lon_};
            boat_status["state"] = autopilot_mode_;
            boat_status["full_autonomy_maneuver"] = full_autonomy_maneuver_;
            boat_status["speed"] = speed_;
            boat_status["velocity_vector"] = {velocity_vector_x_, velocity_vector_y_};
            boat_status["bearing"] = desired_heading_;
            boat_status["heading"] = heading_;
            boat_status["true_wind_speed"] = true_wind_speed;
            boat_status["true_wind_angle"] = true_wind_angle;
            boat_status["apparent_wind_speed"] = apparent_wind_speed_;
            boat_status["apparent_wind_angle"] = apparent_wind_angle_;
            boat_status["sail_angle"] = desired_sail_angle_;
            boat_status["rudder_angle"] = desired_rudder_angle_;
            boat_status["current_waypoint_index"] = current_waypoint_index_;
            boat_status["distance_to_next_waypoint"] = distance_to_next_waypoint;

            // vesc telemetry
            boat_status["vesc_telemetry_data_rpm"] = vesc_rpm_;
        }

        RCLCPP_INFO(this->get_logger(), "hi4");

        // POST to telemetry server (ignore failures)
        bool ok = post_json("boat_status/set/" + std::to_string(instance_id_), boat_status);
        if (!ok) RCLCPP_WARN(this->get_logger(), "Failed to POST boat status");

        RCLCPP_INFO(this->get_logger(), "hi5");
    }

    void update_waypoints_from_telemetry() {
        auto res = get_raw_response_from_telemetry_server("waypoints/get_new/" + std::to_string(instance_id_));
        if (!res.has_value()) return;
        json j = res.value();
        if (j.is_object() && j.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No new waypoints received from telemetry server.");
            return;
        }
        if (!j.is_array()) {
            RCLCPP_WARN(this->get_logger(), "Invalid waypoints format: expected array");
            return;
        }

        autoboat_msgs::msg::WaypointList waypoint_list_msg;
        std::vector<std::pair<double,double>> new_waypoints;

        for (const auto &w : j) {
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

    void update_autopilot_parameters_from_telemetry() {
        auto res = get_raw_response_from_telemetry_server("autopilot_parameters/get_new/" + std::to_string(instance_id_));
        if (!res.has_value()) return;
        
        json j = res.value();
        
        if (j.is_object() && j.empty()) return;

        // In this example we simply re-publish the whole dictionary as JSON string
        std_msgs::msg::String msg;
        msg.data = j.dump();
        autopilot_parameters_pub_->publish(msg);
    }

};




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
