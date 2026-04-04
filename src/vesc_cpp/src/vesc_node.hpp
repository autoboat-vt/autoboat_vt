#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autoboat_msgs/msg/vesc_control_data.hpp>
#include <autoboat_msgs/msg/vesc_telemetry_data.hpp>

#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>

#include "vesc_protocol/vesc_protocol.hpp"
#include "utils.hpp"

class VescNode : public rclcpp::Node {
public:
    VescNode();
    ~VescNode() override;

private:
    void control_callback(const autoboat_msgs::msg::VESCControlData::SharedPtr msg);
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
