#include "vesc_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

static constexpr uint16_t VESC_VID = 0x0483;
static constexpr uint16_t VESC_PID = 0x5740;
static constexpr int MOTOR_POLE_PAIRS = 7;


VescNode::VescNode() : Node("vesc_publisher"), io_ctx(), serial_driver(io_ctx) {
    RCLCPP_INFO(this->get_logger(), "Initializing VESC node...");

    // Setup Serial Port
    drivers::serial_driver::SerialPortConfig serial_port_config(
        115200, 
        drivers::serial_driver::FlowControl::NONE, 
        drivers::serial_driver::Parity::NONE, 
        drivers::serial_driver::StopBits::ONE
    );
    
    try {
        std::string device_filepath = get_device_filepath_from_vid_pid_and_serial_number(VESC_VID, VESC_PID, ""); 
        serial_driver.init_port(device_filepath, serial_port_config);
        serial_port = serial_driver.port();
        
        if (!serial_port) throw std::runtime_error("Failed to get serial port handle");
        serial_port->open();
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to VESC over serial: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // ROS 2 Comms
    vesc_control_subscriber = this->create_subscription<autoboat_msgs::msg::VESCControlData>(
        "/propeller_motor_control_struct",
        rclcpp::SensorDataQoS(),
        std::bind(&VescNode::control_callback, this, std::placeholders::_1)
    );

    vesc_telemetry_data_publisher = this->create_publisher<autoboat_msgs::msg::VESCTelemetryData>(
        "/vesc_telemetry_data",
        rclcpp::SensorDataQoS()
    );

    timer = this->create_wall_timer(50ms, std::bind(&VescNode::timer_callback, this));
}

VescNode::~VescNode() {
    if (serial_port && serial_port->is_open()) {
        serial_port->close();
    }
}

void VescNode::control_callback(const autoboat_msgs::msg::VESCControlData::SharedPtr msg) {
    if (!serial_port || !serial_port->is_open()) return;

    std::vector<uint8_t> packet;
    if (msg->control_type_for_vesc == "rpm") {
        float motorVal = msg->control_value * MOTOR_POLE_PAIRS;
        packet = protocol.generateSetRPM(motorVal);
    } 
    else if (msg->control_type_for_vesc == "duty_cycle") {
        packet = protocol.generateSetDuty(msg->control_value);
    } 
    else {
        packet = protocol.generateSetCurrent(msg->control_value);
    }

    try {
        serial_port->send(packet);
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write control data to VESC.");
    }
}

void VescNode::timer_callback() {
    if (!serial_port || !serial_port->is_open()) return;

    // Send Get Values command
    std::vector<uint8_t> request = protocol.generateGetValues();
    try {
        serial_port->send(request);
    } 
    catch (...) {
        // Disconnected
        return;
    }

    // Read Response
    std::vector<uint8_t> buffer(256);
    size_t n = 0;
    try {
        n = serial_port->receive(buffer);
    } 
    catch (...) {
        n = 0;
    }

    if (n == 0) {
        missed_measurements_in_a_row++;
        return;
    }
    
    missed_measurements_in_a_row = 0;

    buffer.resize(n);
    size_t processed = protocol.processReadPacket(buffer);
    
    if (processed > 0) {
        // We Successfully parsed a message, now publish
        float rpm = protocol.data.rpm / MOTOR_POLE_PAIRS;
        float motor_current = protocol.data.avgMotorCurrent;

        vesc_telemetry_data_publisher->publish(autoboat_msgs::msg::VESCTelemetryData()
            .set__rpm(rpm)
            .set__duty_cycle(protocol.data.dutyCycleNow)
            .set__voltage_to_vesc(protocol.data.inpVoltage)
            .set__current_to_vesc(protocol.data.avgInputCurrent)
            .set__voltage_to_motor(rpm / 180.0f)
            .set__avg_current_to_motor(motor_current)
            .set__wattage_to_motor(motor_current * rpm / 180.0f)
            .set__motor_temperature(protocol.data.tempMotor)
            .set__vesc_temperature(protocol.data.tempMosfet)
            .set__time_since_vesc_startup_in_ms(0)
            .set__amp_hours(protocol.data.ampHours)
            .set__amp_hours_charged(protocol.data.ampHoursCharged)
        );
    }

    print_cpu_and_ram_stats();
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VescNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
