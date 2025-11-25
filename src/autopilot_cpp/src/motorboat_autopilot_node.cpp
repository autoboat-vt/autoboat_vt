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



class MotorboatAutopilotNode : public rclcpp::Node {
    explicit MotorboatAutopilotNode() : Node("motorboat_autopilot_cpp"), {
        
    }
}





int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorboatAutopilotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


