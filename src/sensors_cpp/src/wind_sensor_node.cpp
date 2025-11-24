#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>

#include <deque>
#include <string>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::SerialPortConfig;

static constexpr uint16_t WIND_SENSOR_VID = 0x0403;
static constexpr uint16_t WIND_SENSOR_PID = 0x6001;
static const std::string WIND_SENSOR_SERIAL_NUMBER = "ABSCDYAB";

const std::string RC_DEVICE_FILE_PATH = "/dev/ttyAMA0";


std::string getPort(uint16_t vid, uint16_t pid, const std::string& serial_number)
{
    auto devices = serial::list_ports();
    for (const auto& device : devices) {
        std::cout << device.serial_number << std::endl;
        if (device.vid == vid &&
            device.pid == pid &&
            device.serial_number == serial_number)
        {
            return device.port;
        }
    }
    throw std::runtime_error("Device not found");
}



static constexpr double KNOTS_TO_METERS_PER_SECOND = 0.514444;

class WindSensorPublisher : public rclcpp::Node {
public:
    WindSensorPublisher(): Node("wind_sensor_publisher"), io_ctx_(), serial_driver_(io_ctx_) {
        apparent_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/apparent_wind_vector", rclcpp::SensorDataQoS());

        // term_sub_ = this->create_subscription<std_msgs::msg::Bool>("/should_terminate", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        //     if(msg->data) {
        //       RCLCPP_INFO(this->get_logger(), "Termination requested.");
        //       rclcpp::shutdown();
        //     }
        // });

        RCLCPP_INFO(this->get_logger(), "Initializing wind sensor node...");



        SerialPortConfig cfg(38400, drivers::serial_driver::FlowControl::NONE, drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
        

        std::string port_name = getPort(WIND_SENSOR_VID, WIND_SENSOR_PID, WIND_SENSOR_SERIAL_NUMBER);
        RCLCPP_INFO(this->get_logger(), "Opening port: %s", port_name.c_str());
        serial_driver_.init_port(port_name, cfg);
        port_ = serial_driver_.port();
        
        if (!port_) 
            throw std::runtime_error("Failed to get serial port handle");
        

        port_->open();

        timer_ = this->create_wall_timer(100ms, std::bind(&WindSensorPublisher::main_loop, this));

        RCLCPP_INFO(this->get_logger(), "Wind sensor publisher running");
    }

    ~WindSensorPublisher() override {
        if (port_ && port_->is_open()) {
          RCLCPP_INFO(this->get_logger(), "Closing serial port");
          port_->close();
        }
    }

    
private:

    drivers::common::IoContext io_ctx_;
    SerialDriver serial_driver_;
    std::shared_ptr<SerialPort> port_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr apparent_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr term_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<std::pair<double,double>> wind_history_;


    void main_loop() {
        if (!port_ || !port_->is_open())
            return;
        
        std::vector<uint8_t> buffer(256);
        size_t n = port_->receive(buffer);
        
        if (n == 0)
            return;

        std::string line(buffer.begin(), buffer.begin() + n);

        if (line.empty()) 
          return;
        

        std::vector<std::string> fields;
        std::string temp;
        for (char c : line) {
            if (c == ',') {
                fields.push_back(temp);
                temp.clear();
            } 
            else {
                temp.push_back(c);
            }
        }

        fields.push_back(temp);

        if (fields.size() != 6) {
            return;
        }

        double apparent_angle = std::stod(fields[1]);
        double apparent_speed_knots = std::stod(fields[3]);
        double speed_mps = apparent_speed_knots * KNOTS_TO_METERS_PER_SECOND;
        double angle_ccw = std::fmod(180.0 - apparent_angle + 360.0, 360.0);

        double x = speed_mps * std::cos(angle_ccw * M_PI / 180.0);
        double y = speed_mps * std::sin(angle_ccw * M_PI / 180.0);

        if (wind_history_.size() >= 15) 
            wind_history_.pop_front();
        

        wind_history_.push_back({x, y});

        auto filtered = weighted_average(wind_history_);

        geometry_msgs::msg::Vector3 msg;
        msg.x = filtered.first;
        msg.y = filtered.second;
        msg.z = 0.0;

        apparent_pub_->publish(msg);
    }


    double sum_integers(int n) {
        return (n * (n + 1)) / 2.0;
    }
    

    std::pair<double, double> weighted_average(const std::deque<std::pair<double,double>> &d) {
        double w1 = 0.0;
        double w2 = 0.0;
        int idx = 1;
        for (auto &p : d) {
            w1 += p.first * idx;
            w2 += p.second * idx;
            idx++;
        }
        double denom = sum_integers((int)d.size());
        return { w1/denom, w2/denom };
    }

};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WindSensorPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
