#include "crossfire.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "autoboat_msgs/msg/rc_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <stdexcept>

using namespace std::chrono_literals;


const rclcpp::QoS sensor_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
const std::string RC_DEVICE_FILE_PATH = "/dev/ttyAMA0";



class RCDataPublisher : public rclcpp::Node {

public:
    
    RCDataPublisher() : Node("rc_data_publisher") {

        // Connect to Crossfire Device. Crossfire is the protocol that we use to communicate to the RC receiver
        crossfire::XCrossfire crossfire_device_ = crossfire::XCrossfire(RC_DEVICE_FILE_PATH);
        crossfire_device = &crossfire_device_;

        if (crossfire_device->open_port()) { std::printf("Port opened...\n"); }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Continuously Attempt To Reconnect
        while (true) {
            while (!crossfire_device->is_paired()) {
                std::printf("Waiting for reconnect...\n");
                auto _ = crossfire_device->open_port();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }



        // Initialize Publishers and Timers
        rc_data_publisher = this->create_publisher<autoboat_msgs::msg::RCData>("rc_data", sensor_qos);

        main_loop_timer = this->create_wall_timer(10ms, std::bind(&RCDataPublisher::main_loop, this));

    }

private:

    rclcpp::TimerBase::SharedPtr main_loop_timer;
    rclcpp::Publisher<autoboat_msgs::msg::RCData>::SharedPtr rc_data_publisher;
    crossfire::XCrossfire* crossfire_device; 


    void main_loop() {
        autoboat_msgs::msg::RCData message = autoboat_msgs::msg::RCData();

        if (!crossfire_device->is_paired()) {
            throw std::runtime_error("Connection to RC Receiver Lost");
        }


        const std::array<uint16_t, 16> channels = crossfire_device->get_channel_state();
        // for (int i = 0; i < 4; i++) {
        //     if (channels.front() != 0) { std::printf("Channel %d: %u\n", i, channels[i]); }
        // }
        // const auto _ = crossfire_device->set_telemetry_battery(11.8, 25, 1200, 96);


        message.joystick_left_y = normalize_joystick_input(channels[15], 174, 1811);
        message.joystick_left_x = normalize_joystick_input(channels[14], 174, 1811);
        
        message.joystick_right_y = normalize_joystick_input(channels[13], 191, 1792);
        message.joystick_right_x = normalize_joystick_input(channels[12], 174, 1811);

        parse_multiplexed_buttons(channels[7], message.button_a, message.button_d);
        message.toggle_b = parse_toggle(channels[10]);
        message.toggle_c = parse_toggle(channels[9]);
        message.toggle_e = parse_toggle(channels[11]);
        message.toggle_f = parse_toggle(channels[8]);
        

        if (message.toggle_b == -1 || message.toggle_c == -1 || message.toggle_e == -1 || message.toggle_f == -1) 
            return


        // message.data = "Hello, world! " + std::to_string(count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        rc_data_publisher->publish(message);
    }



    float normalize_joystick_input(float input, float cur_min, float cur_max) {
        /*
        Normalizes a number between cur_min and cur_max to between -100 and 100.
        https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio 
        */
        return (((input - cur_min) * 200) / (cur_max - cur_min)) - 100;
    }


    int parse_toggle(int toggle_state) {
        // toggle data for states from rc testing: 191, 997, 1792
        if (toggle_state <= 500) 
            return 0;

        else if (toggle_state <= 1500)
            return 1;

        else if (toggle_state <= 2000)
            return 2;

        else { 
            // print(f"WARNING: Toggle state was not properly accounted for: {toggle_state}")
            return -1;
        }
    }


    void parse_multiplexed_buttons(int button_state, bool& button1_return, bool& button2_return) {
        /*
        In an effort to reduce the number of channels needed for communications (because for some reason crsf doesn't support 10 channels),
        we have decided to have a little bit of a convoluted scheme for reading button inputs to the controller

        if pwm < 500 then no buttons are pressed
        if pwm > 500 and pwm < 1000 then only button_a is pressed
        if pwm > 1000 and pwm < 1500 then only button d is pressed
        if pwm > 1500 then both button_a and button_d are pressed

        This method returns button_a, button_d in that order where not pressed is False and pressed is True
        
        This is bound onto the RC controller, meaning that if you would like to change the RC controller that we are using, you will have to program this 
        functionality into the controller itself
        */
        
        
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

};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RCDataPublisher>());
    rclcpp::shutdown();
    return 0;
}