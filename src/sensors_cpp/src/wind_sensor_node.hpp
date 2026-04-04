#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>
#include "sensors_cpp/utils.hpp"

#include <deque>
#include <string>
#include <vector>
#include <cmath>

class WindSensorPublisher : public rclcpp::Node {
public:
    WindSensorPublisher();
    ~WindSensorPublisher() override;
    
private:
    drivers::common::IoContext io_ctx;
    drivers::serial_driver::SerialDriver serial_driver;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr apparent_wind_vector_publisher;
    rclcpp::TimerBase::SharedPtr main_loop_timer;
    std::deque<std::pair<double,double>> wind_history;

    void main_loop();
    double sum_integers(int n);
    std::pair<double, double> weighted_average(const std::deque<std::pair<double,double>> &d);
};
