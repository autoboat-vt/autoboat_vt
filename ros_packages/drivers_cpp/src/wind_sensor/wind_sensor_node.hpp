#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>
#include "utils.hpp"

#include <deque>
#include <string>
#include <vector>
#include <cmath>

/**
 * @class WindSensorPublisher
 * @brief ROS 2 node that reads wind data from a serial wind sensor and publishes it.
 * 
 * This node publishes the apparent wind velocity vector measured counter-clockwise 
 * from the centerline of the boat (x-axis: centerline, y-axis: left).
 */
class WindSensorPublisher : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Wind Sensor Publisher object.
     * 
     * Initializes serial communication and the main processing loop.
     */
    WindSensorPublisher();

    /**
     * @brief Destroy the Wind Sensor Publisher object.
     */
    ~WindSensorPublisher() override;
    
private:
    drivers::common::IoContext io_ctx;
    drivers::serial_driver::SerialDriver serial_driver;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr apparent_wind_vector_publisher;
    rclcpp::TimerBase::SharedPtr main_loop_timer;
    std::deque<std::pair<double,double>> wind_history;

    /**
     * @brief Main loop for reading from serial and publishing wind data.
     */
    void main_loop();

    /**
     * @brief Helper function to sum positive integers up to n.
     * @param n The upper bound.
     * @return double The sum.
     */
    double sum_integers(int n);

    /**
     * @brief Calculates a linear moving weighted average of the wind history.
     * @param d The history of wind vectors.
     * @return std::pair<double, double> The averaged wind vector.
     */
    std::pair<double, double> weighted_average(const std::deque<std::pair<double,double>> &d);
};
