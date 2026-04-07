#pragma once

#include "xcrsf/crossfire.h"
#include "autoboat_msgs/msg/rc_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "utils.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

/**
 * @class RCDataPublisher
 * @brief ROS 2 node that reads Remote Control data from an RC receiver and publishes it.
 * 
 * This node processes CRSF protocol data from a serial connection to publish the state
 * of RC channels, including joysticks, toggles, and multiplexed buttons.
 */
class RCDataPublisher : public rclcpp::Node {
public:
    /**
     * @brief Construct a new RC Data Publisher object.
     * 
     * Initializes the serial crossfire device and the main processing loop.
     */
    RCDataPublisher();

private:
    rclcpp::TimerBase::SharedPtr main_loop_timer;
    rclcpp::Publisher<autoboat_msgs::msg::RCData>::SharedPtr rc_data_publisher;

    crossfire::XCrossfire crossfire_device; 

    /**
     * @brief Main loop for reading from serial and publishing RC data.
     */
    void main_loop();

    /**
     * @brief Normalizes a raw joystick input to a range of -100 to 100.
     * @param input Raw PWM/CRSF value.
     * @param min Minimum possible raw value.
     * @param max Maximum possible raw value.
     * @return float Normalized value (-100 to 100).
     */
    float normalize_joystick_input(float input, float min, float max);

    /**
     * @brief Parses a raw toggle state into 0, 1, or 2.
     * @param toggle_state Raw PWM/CRSF value.
     * @return int Toggle state (0: Up, 1: Middle, 2: Down).
     */
    int parse_toggle(int toggle_state);

    /**
     * @brief Parses multiplexed button inputs from a single channel.
     * @param button_state Raw PWM/CRSF value.
     * @param button1_return Reference to store the first button's state.
     * @param button2_return Reference to store the second button's state.
     */
    void parse_multiplexed_buttons(int button_state, bool& button1_return, bool& button2_return);
};
