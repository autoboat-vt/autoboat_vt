#include "rc_node.hpp"
#include <filesystem>

using namespace std::chrono_literals;

const rclcpp::QoS sensor_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

static constexpr uint16_t RC_VID = 0x0403;
static constexpr uint16_t RC_PID = 0x6001;
static const std::string RC_SERIAL_NUMBER = "A9001WL3";

namespace fs = std::filesystem;

RCDataPublisher::RCDataPublisher() : Node("rc_data_publisher"), crossfire_device(get_device_filepath_from_vid_pid_and_serial_number(RC_VID, RC_PID, RC_SERIAL_NUMBER)) {
    if (crossfire_device.open_port()) { 
        std::printf("Port opened...\n"); 
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Continuously attempt To reconnect as long as the node is still running
    // rclcpp will return false if the node is told to turn off for whatever reason (for example ctrl+c signal)
    while (!crossfire_device.is_paired() && rclcpp::ok()) {
        std::printf("Waiting for reconnect...\n");
        RCLCPP_INFO(this->get_logger(), "Waiting for reconnect...\n:");
        auto _ = crossfire_device.open_port();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    // Initialize Publishers and Timers
    rc_data_publisher = this->create_publisher<autoboat_msgs::msg::RCData>("rc_data", sensor_qos);
    main_loop_timer = this->create_wall_timer(50ms, std::bind(&RCDataPublisher::main_loop, this));
}

void RCDataPublisher::main_loop() {
    autoboat_msgs::msg::RCData message = autoboat_msgs::msg::RCData();

    if (!crossfire_device.is_paired()) {
        throw std::runtime_error("Connection to RC Receiver Lost");
    }

    const std::array<uint16_t, 16> channels = crossfire_device.get_channel_state();

    message.joystick_left_y = normalize_joystick_input(channels[0], 174, 1811);
    message.joystick_left_x = normalize_joystick_input(channels[1], 174, 1811);
    
    message.joystick_right_y = normalize_joystick_input(channels[2], 191, 1792);
    message.joystick_right_x = normalize_joystick_input(channels[3], 174, 1811);

    parse_multiplexed_buttons(channels[8], message.button_a, message.button_d);
    message.toggle_b = parse_toggle(channels[5]);
    message.toggle_c = parse_toggle(channels[6]);
    message.toggle_e = parse_toggle(channels[4]);
    message.toggle_f = parse_toggle(channels[7]);

    // something very bad happened
    if (message.toggle_b == -1 || message.toggle_c == -1 || message.toggle_e == -1 || message.toggle_f == -1) 
        return;

    rc_data_publisher->publish(message);
    print_cpu_and_ram_stats();
}

float RCDataPublisher::normalize_joystick_input(float input, float cur_min, float cur_max) {
    return (((input - cur_min) * 200) / (cur_max - cur_min)) - 100;
}

int RCDataPublisher::parse_toggle(int toggle_state) {
    if (toggle_state <= 500) 
        return 0;
    else if (toggle_state <= 1500)
        return 1;
    else if (toggle_state <= 2000)
        return 2;
    else { 
        return -1;
    }
}

void RCDataPublisher::parse_multiplexed_buttons(int button_state, bool& button1_return, bool& button2_return) {
    if (button_state < 500) {
        button1_return = false;
        button2_return = false;
    }
    else if (button_state >= 500 && button_state < 1000) {
        button1_return = true;
        button2_return = false;
    }
    else if (button_state >= 1000 && button_state < 1500) {
        button1_return = false;
        button2_return = true;
    }
    else if (button_state >= 1500) { 
        button1_return = true;
        button2_return = true;
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RCDataPublisher>());
    rclcpp::shutdown();
    return 0;
}