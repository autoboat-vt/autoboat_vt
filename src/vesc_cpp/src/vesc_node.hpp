#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autoboat_msgs/msg/vesc_control_data.hpp>
#include <autoboat_msgs/msg/vesc_telemetry_data.hpp>

#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>

#include "vesc_protocol/vesc_protocol.hpp"
#include "utils.hpp"

/**
 * @class VescNode
 * @brief ROS 2 node that interfaces with a VESC (Vedder Electronic Speed Controller).
 * 
 * This node handles motor control commands (RPM, Current, Duty Cycle) and publishes 
 * telemetry data received from the VESC.
 */
class VescNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Vesc Node object.
     * 
     * Initializes serial communication, subscriptions, and the telemetry timer.
     */
    VescNode();

    /**
     * @brief Destroy the Vesc Node object.
     */
    ~VescNode() override;

private:
    /**
     * @brief Callback for receiving motor control commands.
     * @param msg The control data message containing desired RPM/current/duty cycle.
     */
    void control_callback(const autoboat_msgs::msg::VESCControlData::SharedPtr msg);

    /**
     * @brief Timer callback for requesting and publishing VESC telemetry data.
     */
    void timer_callback();

    rclcpp::Subscription<autoboat_msgs::msg::VESCControlData>::SharedPtr control_sub;
    rclcpp::Publisher<autoboat_msgs::msg::VESCTelemetryData>::SharedPtr telemetry_pub;
    rclcpp::TimerBase::SharedPtr timer;

    drivers::common::IoContext io_ctx;
    drivers::serial_driver::SerialDriver serial_driver;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port;

    vesc_cpp::VescProtocol protocol;
    
    int missed_measurements_in_a_row = 0;
};
