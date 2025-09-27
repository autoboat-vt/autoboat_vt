#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

def getPort(vid, pid, serial_number) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid and device.serial_number == serial_number:
            return device.device
    raise OSError('Device not found')


class RCPublisher : public rclcpp::Node{
     /*
     def __init__(self):
        super().__init__("rc_publisher")
        
        self.create_timer(0.3, self.timer_callback)
        
        serial_port = getPort(RC_VID, RC_PID, RC_SERIAL_NUMBER)
        self.sensor_serial = serial.Serial(serial_port, BAUD_RATE)

        self.crsf_parser = CRSFParser(self.save_frame)
        self.crsf_frame_rc_data = None
        self.serial_stream = bytearray()
        self.callback_count = 0
                
        self.rc_data_publisher = self.create_publisher(RCData, '/rc_data', qos_profile_sensor_data)
        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)
    */
    
    public RCPublisher(): Node("rc_publisher"){
        rc_data_publisher_ = this->create_publisher<std_msgs::msg::String>("rc_data", rclcpp::SensorDataQoS);
        termination_listener_ = this->create_subscription<std_msgs::msg::Bool>("should_terminate", 10); 
        //import this later
        //serial_port = getPort()
        //sensor_serial = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
        
    }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rc_data_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr termination_listener_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        std::string serial_port;
        int sensor_serial;


}

