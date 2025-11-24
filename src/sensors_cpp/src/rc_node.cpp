#include "crossfire.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "autoboat_msgs/msg/rc_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sys/sysinfo.h>



#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

// #include <serial_driver/serial_driver.hpp>
// #include <serial_driver/serial_port.hpp>
// #include <serial/serial.h>
// #include <libserialport.h>


#include <stdexcept>

using namespace std::chrono_literals;




const rclcpp::QoS sensor_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);



static constexpr uint16_t RC_VID = 0x0403;
static constexpr uint16_t RC_PID = 0x6001;
static const std::string RC_SERIAL_NUMBER = "A9001WL3";

// BAUD_RATE = 420000



// const std::string RC_DEVICE_FILE_PATH;
const std::string RC_DEVICE_FILE_PATH = "/dev/ttyUSB0";

// std::string getPort(uint16_t vid, uint16_t pid, const std::string& serial_number) {
//     auto devices = serial::list_ports();
//     for (const auto& device : devices) {
//         std::cout << device.serial_number << std::endl;
//         if (device.vid == vid &&
//             device.pid == pid &&
//             device.serial_number == serial_number)
//         {
//             return device.port;
//         }
//     }
//     throw std::runtime_error("Device not found");
// }

// std::string getPort(uint16_t vid, uint16_t pid, const std::string& serial_number)
// {
//     struct sp_port **port_list;
//     if (sp_list_ports(&port_list) != SP_OK) {
//         throw std::runtime_error("Failed to enumerate serial ports");
//     }

//     for (int i = 0; port_list[i] != nullptr; ++i) {
//         sp_port *port = port_list[i];

//         // Get USB info
//         int usb_vid = 0, usb_pid = 0;
//         char *usb_serial = nullptr;

//         sp_get_port_usb_vid_pid(port, &usb_vid, &usb_pid);
//         sp_get_port_usb_serial(port, &usb_serial);

//         std::string port_name = sp_get_port_name(port);

//         std::cout << "Port: " << port_name
//                   << " VID: " << std::hex << usb_vid
//                   << " PID: " << pid
//                   << " Serial: " << (usb_serial ? usb_serial : "NONE")
//                   << std::endl;

//         if ((uint16_t)usb_vid == vid &&
//             (uint16_t)usb_pid == pid &&
//             usb_serial &&
//             serial_number == usb_serial)
//         {
//             sp_free_port_list(port_list);
//             return port_name;  // e.g. "/dev/ttyACM0"
//         }
//     }

//     sp_free_port_list(port_list);
//     throw std::runtime_error("ROS2 serial device not found");
// }






class RCDataPublisher : public rclcpp::Node {

public:
    
    // crossfire_device() in the constructor just intializes the crossfire device and tells the compiler how to always intialize the 
    // object. To learn more see: 
    RCDataPublisher() : Node("rc_data_publisher") {

        if (crossfire_device.open_port()) { std::printf("Port opened...\n"); }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Continuously Attempt To Reconnect
        while (!crossfire_device.is_paired()) {
            std::printf("Waiting for reconnect...\n");
            RCLCPP_INFO(this->get_logger(), "Waiting for reconnect...\n:");
            auto _ = crossfire_device.open_port();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }



        // Initialize Publishers and Timers
        rc_data_publisher = this->create_publisher<autoboat_msgs::msg::RCData>("rc_data", sensor_qos);

        main_loop_timer = this->create_wall_timer(10ms, std::bind(&RCDataPublisher::main_loop, this));


    }

private:

    rclcpp::TimerBase::SharedPtr main_loop_timer;
    rclcpp::Publisher<autoboat_msgs::msg::RCData>::SharedPtr rc_data_publisher;

    // This is an example of uniform initialization which is a type of initialization
    // that guarantees the compiler that the variable will always be initialized this way
    // https://www.geeksforgeeks.org/cpp/uniform-initialization-in-c/
    crossfire::XCrossfire crossfire_device {RC_DEVICE_FILE_PATH}; 
    // const std::string RC_DEVICE_FILE_PATH = getPort(RC_VID, RC_PID, RC_SERIAL_NUMBER);


    void main_loop() {

        auto start = std::chrono::steady_clock::now();

        
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
        
        auto end = std::chrono::steady_clock::now();
    
        // Calculate and print the elapsed time in milliseconds
        std::chrono::duration<double, std::milli> elapsed_ms = end - start;
        std::cout << "Consumed time in milliseconds: " << elapsed_ms.count() << "ms\n";

        print_stats();
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


    void print_stats() {
        std::ifstream statm("/proc/self/statm");
        long size, resident, share, text, lib, data, dt;
        statm >> size >> resident >> share >> text >> lib >> data >> dt;

        long page_size_kb = sysconf(_SC_PAGESIZE) / 1024; // in KB
        std::cout << "Resident RAM: " << resident * page_size_kb << " KB\n";

        // --- CPU time ---
        std::ifstream stat("/proc/self/stat");
        std::string tmp;
        long utime_ticks, stime_ticks;
        for (int i=0; i<13; ++i) stat >> tmp; // skip first 13 fields
        stat >> utime_ticks >> stime_ticks;     // user and kernel time in ticks

        long ticks_per_sec = sysconf(_SC_CLK_TCK);
        double utime_sec = (double)utime_ticks / ticks_per_sec;
        double stime_sec = (double)stime_ticks / ticks_per_sec;

        std::cout << "User CPU time: " << utime_sec << " s\n";
        std::cout << "System CPU time: " << stime_sec << " s\n";
    }

};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RCDataPublisher>());
    rclcpp::shutdown();
    return 0;
}