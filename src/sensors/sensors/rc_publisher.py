#!/usr/bin/python3

# This all uses the following library (Thank you very much for actually making a parser for this insufferable protocol with no documentation)
# https://github.com/AlessioMorale/crsf_parser/tree/master 

from sys import argv
argv = argv[1:]


import rclpy
from rclpy.qos import qos_profile_sensor_data
import time
import serial
from serial.tools import list_ports
from std_msgs.msg import Bool
from autoboat_msgs.msg import RCData
from rclpy.node import Node

from crsf_parser.payloads import PacketsTypes
from crsf_parser.frames import crsf_frame
from crsf_parser import CRSFParser, PacketValidationStatus



RC_VID = 0x0403
RC_PID = 0x6001
RC_SERIAL_NUMBER = "A9001WL3"
BAUD_RATE = 420000



def getPort(vid, pid, serial_number) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid and device.serial_number == serial_number:
            return device.device
    raise OSError('Device not found')




class RCPublisher(Node):
    """
    Reads Remote Control data from the RC receiver over a serial USB connection and then publishes that data so that the autopilot can use it
    
    This node publishes the state of all of the 16 RC channels, which are configured on the remote to mean certain things 
    """
    
    
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
        self.termination_listener = self.create_subscription(Bool, "/should_terminate", self.should_terminate_callback, 10)



    def save_frame(self, frame: crsf_frame, status: PacketValidationStatus):
        """
        this frame will come back as a crsf frame with the following declaration:
        
        crsf_frame = Struct(
        "header" / crsf_header,
        "payload"
        / Switch(
            this.header.type,
            {
                PacketsTypes.HEARTBEAT: payload_heartbeat,
                PacketsTypes.BATTERY_SENSOR: payload_battery_sensor,
                PacketsTypes.LINK_STATISTICS: payload_link_statistics,
                PacketsTypes.RC_CHANNELS_PACKED: payload_rc_channels_packed,
            },
            default=Array(this.frame_length - 2, Byte),
          ),
            "crc_offset" / Tell,
            "CRC" / Int8ub,
        )
        
        The only packets that we care about are the PacketsTypes.RC_CHANNELS_PACKED
        payload_rc_channels_packed objects are created with the following declaration:
        
        payload_rc_channels_packed= ByteSwapped(
            BitStruct("channels" / Array(16, BitsInteger(11)))
        )
        
        See this page for more information about the BitStruct: https://construct.readthedocs.io/en/latest/bitwise.html 
        """
        
        # We need to make sure the only packets we save are information about the rc inputs just to be safe
        if frame.header.type != PacketsTypes.RC_CHANNELS_PACKED or status.value != 1: return
        
        self.crsf_frame_rc_data = frame
        
        
        

    def normalize_joystick_input(self, input, cur_min, cur_max):
        """
        Normalizes a number between cur_min and cur_max to between -100 and 100.
        https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio 
        """
        return (((input - cur_min) * 200) / (cur_max - cur_min)) - 100

    def parse_toggle(self, toggle_state):
        #toggle data for states from rc testing: 191, 997, 1792
        if toggle_state <= 500: return 0
        elif toggle_state <= 1500: return 1
        elif toggle_state <= 2000: return 2
        else: 
            print(f"WARNING: Toggle state was not properly accounted for: {toggle_state}")
            return -1
    
    
    
    def parse_multiplexed_buttons(self, button_state):
        """
        In an effort to reduce the number of channels needed for communications (because for some reason crsf doesn't support 10 channels),
        we have decided to have a little bit of a convoluted scheme for reading button inputs to the controller

        if pwm < 500 then no buttons are pressed
        if pwm > 500 and pwm < 1000 then only button_a is pressed
        if pwm > 1000 and pwm < 1500 then only button d is pressed
        if pwm > 1500 then both button_a and button_d are pressed

        This method returns button_a, button_d in that order where not pressed is False and pressed is True
        
        This is bound onto the RC controller, meaning that if you would like to change the RC controller that we are using, you will have to program this 
        functionality into the controller itself
        """
        
        
        if button_state < 500: return False, False
        elif button_state >= 500 and button_state < 1000: return True, False
        elif button_state >= 1000 and button_state < 1500: return False, True
        elif button_state >= 1500: return True, True




    def process_raw_channels(self, raw_channel_array: list) -> RCData:
        """
        Processes the raw channel outputs from the receiver and returns the ros2 msg to publish
        Toggle states:
            State 0: Fully Up
            State 1: Middle
            State 2: Fully Down


        THROTTLE Up/Down: 15 (174 - 1811) 
        THROTTLE Left/Right: 14 (174 - 1811)
        STEERING Up/Down: 13 (191 - 1792)
        STEERING Left/Right: 12 (174 - 1811)
        
        A Button: Multiplexed with button d on channel 7 (see parse_multiplexed_buttons)
        B Toggle: 10 (State 1: ~191, State 2: ~997, State 3: ~1792)
        C Toggle: 9 (State 1: ~191, State 2: ~997, State 3: ~1792)
        D Button: Multiplexed with button a on channel 7 (see parse_multiplexed_buttons)
        E Toggle: 11 (State 1: ~191, State 2: ~997, State 3: ~1792)
        F Toggle: 8 (State 1: ~191, State 2: ~997, State 3: ~1792)
        
        btw "~" means approximately. Its hard to make the buttons consistent, 
        so we just take in a large swath of values where the button signal will be and call it a day
        """
        
        # tranform from [175, 1811] to [-100, 100]
        joystick_left_y = self.normalize_joystick_input(raw_channel_array[15], 174, 1811)
        joystick_left_x = self.normalize_joystick_input(raw_channel_array[14], 174, 1811)
        
        joystick_right_y = self.normalize_joystick_input(raw_channel_array[13], 191, 1792)
        joystick_right_x = self.normalize_joystick_input(raw_channel_array[12], 174, 1811)

        button_a, button_d = self.parse_multiplexed_buttons(raw_channel_array[7])
        toggle_b = self.parse_toggle(raw_channel_array[10])
        toggle_c = self.parse_toggle(raw_channel_array[9])
        toggle_e = self.parse_toggle(raw_channel_array[11])
        toggle_f = self.parse_toggle(raw_channel_array[8])
        
        if toggle_b == -1 or toggle_c == -1 or toggle_e == -1 or toggle_f == -1: return None
        
        self.get_logger().info(f"joystick right x: {joystick_right_x}")
        self.get_logger().info(f"joystick_left_y: {joystick_left_y}")
        
        
        return RCData(
            joystick_left_x=joystick_left_x, joystick_left_y=joystick_left_y,
            joystick_right_x=joystick_right_x, joystick_right_y=joystick_right_y,
            button_a=button_a, toggle_b=toggle_b, toggle_c=toggle_c, 
            button_d=button_d, toggle_e=toggle_e, toggle_f=toggle_f
        )
        
        
        
    def timer_callback(self):
        num_unread_bytes = self.sensor_serial.in_waiting
        
        if num_unread_bytes == 0: return
        
        values = self.sensor_serial.read(num_unread_bytes)
    
        self.serial_stream.extend(values)

        # parse the data into self.crsf_frame
        self.crsf_parser.parse_stream(self.serial_stream)

        if not self.crsf_frame_rc_data: return 

        rc_data = self.process_raw_channels(self.crsf_frame_rc_data.payload.channels)
        
        if rc_data == None: return
        
        self.rc_data_publisher.publish(rc_data)



    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()
        
        
    def __del__(self):
        self.get_logger().info("closing rc publisher")
        self.sensor_serial.close()
        rclpy.shutdown()    





def main(args=None):
    
    rclpy.init(args=args)
    rc_publisher = RCPublisher()
    rclpy.spin(rc_publisher)

    rc_publisher.destroy_node()
    rclpy.shutdown()

