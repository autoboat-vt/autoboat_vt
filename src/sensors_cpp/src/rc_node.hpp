#pragma once

#include "xcrsf/crossfire.h"
#include "autoboat_msgs/msg/rc_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensors_cpp/utils.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

class RCDataPublisher : public rclcpp::Node {
public:
    RCDataPublisher();

private:
    rclcpp::TimerBase::SharedPtr main_loop_timer;
    rclcpp::Publisher<autoboat_msgs::msg::RCData>::SharedPtr rc_data_publisher;

    crossfire::XCrossfire crossfire_device; 

    void main_loop();
    float normalize_joystick_input(float input, float cur_min, float cur_max);
    int parse_toggle(int toggle_state);
    void parse_multiplexed_buttons(int button_state, bool& button1_return, bool& button2_return);
};
