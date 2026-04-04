#include "vesc_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

static constexpr uint16_t VESC_VID = 0x0483;
static constexpr uint16_t VESC_PID = 0x5740;
static constexpr int MOTOR_POLE_PAIRS = 7;

VescNode::VescNode() : Node("vesc_publisher"), io_ctx(), serial_driver(io_ctx) {
    RCLCPP_INFO(this->get_logger(), "Initializing VESC node...");

    // Setup Serial Port
    drivers::serial_driver::SerialPortConfig cfg(115200, drivers::serial_driver::FlowControl::NONE, drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
    
    try {
        std::string device_filepath = get_device_filepath_from_vid_pid_and_serial_number(VESC_VID, VESC_PID, ""); 
        serial_driver.init_port(device_filepath, cfg);
        serial_port = serial_driver.port();
        
        if (!serial_port) throw std::runtime_error("Failed to get serial port handle");
        serial_port->open();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to VESC over serial: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // ROS 2 Comms
    control_sub = this->create_subscription<autoboat_msgs::msg::VESCControlData>(
        "/propeller_motor_control_struct",
        rclcpp::SensorDataQoS(),
        std::bind(&VescNode::control_callback, this, std::placeholders::_1));

    telemetry_pub = this->create_publisher<autoboat_msgs::msg::VESCTelemetryData>(
        "/vesc_telemetry_data",
        rclcpp::SensorDataQoS());

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
        float motorVal = msg->desired_vesc_rpm * MOTOR_POLE_PAIRS;
        packet = protocol.generateSetRPM(motorVal);
    } else if (msg->control_type_for_vesc == "duty_cycle") {
        packet = protocol.generateSetDuty(msg->desired_vesc_duty_cycle);
    } else {
        packet = protocol.generateSetCurrent(msg->desired_vesc_current);
    }

    try {
        serial_port->send(packet);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write control data to VESC.");
    }
}

void VescNode::timer_callback() {
    if (!serial_port || !serial_port->is_open()) return;

    // Send Get Values command
    std::vector<uint8_t> req = protocol.generateGetValues();
    try {
        serial_port->send(req);
    } catch (...) {
        // Disconnected
        return;
    }

    // Read Response
    std::vector<uint8_t> buffer(256);
    size_t n = 0;
    try {
        n = serial_port->receive(buffer);
    } catch (...) {
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
        auto out_msg = autoboat_msgs::msg::VESCTelemetryData();
        
        float rpm = protocol.data.rpm / MOTOR_POLE_PAIRS;
        float c_motor = protocol.data.avgMotorCurrent;
        
        out_msg.rpm = rpm;
        out_msg.duty_cycle = protocol.data.dutyCycleNow;
        out_msg.voltage_to_vesc = protocol.data.inpVoltage;
        out_msg.current_to_vesc = protocol.data.avgInputCurrent;
        out_msg.voltage_to_motor = rpm / 180.0f; 
        out_msg.avg_current_to_motor = c_motor;
        out_msg.wattage_to_motor = c_motor * rpm / 180.0f;
        out_msg.motor_temperature = protocol.data.tempMotor;
        out_msg.vesc_temperature = protocol.data.tempMosfet;
        out_msg.time_since_vesc_startup_in_ms = 0; 
        out_msg.amp_hours = protocol.data.ampHours;
        out_msg.amp_hours_charged = protocol.data.ampHoursCharged;

        telemetry_pub->publish(out_msg);
        print_cpu_and_ram_stats();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VescNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
