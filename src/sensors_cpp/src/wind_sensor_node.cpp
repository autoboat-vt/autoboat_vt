#include "wind_sensor_node.hpp"

using namespace std::chrono_literals;

static constexpr uint16_t WIND_SENSOR_VID = 0x0403;
static constexpr uint16_t WIND_SENSOR_PID = 0x6001;
static const std::string WIND_SENSOR_SERIAL_NUMBER = "ABSCDYAB";

static constexpr double KNOTS_TO_METERS_PER_SECOND = 0.514444;

WindSensorPublisher::WindSensorPublisher(): Node("wind_sensor_publisher"), io_ctx(), serial_driver(io_ctx) {
    apparent_wind_vector_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("/apparent_wind_vector", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Initializing wind sensor node...");

    drivers::serial_driver::SerialPortConfig cfg(38400, drivers::serial_driver::FlowControl::NONE, drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
    
    std::string device_filepath = get_device_filepath_from_vid_pid_and_serial_number(WIND_SENSOR_VID, WIND_SENSOR_PID, WIND_SENSOR_SERIAL_NUMBER);
    serial_driver.init_port(device_filepath, cfg);
    serial_port = serial_driver.port();
    
    if (!serial_port) 
        throw std::runtime_error("Failed to get serial port handle");
    
    serial_port->open();

    main_loop_timer = this->create_wall_timer(10ms, std::bind(&WindSensorPublisher::main_loop, this));

    RCLCPP_INFO(this->get_logger(), "Wind sensor publisher running");
}

WindSensorPublisher::~WindSensorPublisher() {
    if (serial_port && serial_port->is_open()) {
        serial_port->close();
    }
}

void WindSensorPublisher::main_loop() {
    if (!serial_port || !serial_port->is_open())
        return;
    
    std::vector<uint8_t> buffer(256);
    size_t n = serial_port->receive(buffer);
    
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

    if (wind_history.size() >= 15) 
        wind_history.pop_front();
    
    wind_history.push_back({x, y});

    auto filtered = weighted_average(wind_history);

    geometry_msgs::msg::Vector3 msg;
    msg.x = filtered.first;
    msg.y = filtered.second;
    msg.z = 0.0;

    apparent_wind_vector_publisher->publish(msg);
    print_cpu_and_ram_stats();
}

double WindSensorPublisher::sum_integers(int n) {
    return (n * (n + 1)) / 2.0;
}

std::pair<double, double> WindSensorPublisher::weighted_average(const std::deque<std::pair<double,double>> &d) {
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WindSensorPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
