#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from autoboat_msgs.msg import RCData
from rclpy.qos import qos_profile_sensor_data
from pynput import keyboard
import pynput

global emulated_rcdata
emulated_rcdata = RCData()



class RCEmulatorKeyboard(Node):
    def __init__(self):
        super().__init__('rc_emulator')
        self.rc_emulator_publisher = self.create_publisher(RCData, '/rc_data', qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.rc_emulator_publisher.publish(emulated_rcdata)


def on_press(key):
    global emulated_rcdata
    
        
    # left joystick (uses "WASD" keys)
    try:
        if key.char == 'w':
            emulated_rcdata.joystick_left_y = 100.
        elif key.char == 'd':
            emulated_rcdata.joystick_left_x = 100.
        elif key.char == 's':
            emulated_rcdata.joystick_left_y = -100.
        elif key.char == 'a':
            emulated_rcdata.joystick_left_x = -100.
    except:
        pass

    
    # right joystick (uses arrow keys)    
    try:
        if key == keyboard.Key.up:
            emulated_rcdata.joystick_right_y = 100.
        elif key == keyboard.Key.right:
            emulated_rcdata.joystick_right_x = 100.
        elif key == keyboard.Key.down:
            emulated_rcdata.joystick_right_y = -100.
        elif key == keyboard.Key.left:
            emulated_rcdata.joystick_right_x = -100.
    except:
        pass
    
    
    # Buttons and Toggles
    try:
        # button_a (uses "1")
        if key.char == '1':
            emulated_rcdata.button_a = not emulated_rcdata.button_a

        # toggle_b (uses "2")
        elif key.char == '2':
            emulated_rcdata.toggle_b = (emulated_rcdata.toggle_b + 1) if emulated_rcdata.toggle_b < 2 else 0

        # toggle_c (uses "3")
        elif key.char == '3':
            emulated_rcdata.toggle_c = (emulated_rcdata.toggle_c + 1) if emulated_rcdata.toggle_c < 2 else 0

        # button_d (uses "4")
        elif key.char == '4':
            emulated_rcdata.button_d = not emulated_rcdata.button_d
            
        # toggle_e (uses "5")
        elif key.char == '5':
            emulated_rcdata.toggle_e = (emulated_rcdata.toggle_e + 1) if emulated_rcdata.toggle_e < 2 else 0

        # toggle_f (uses "6")
        elif key.char == '6':
            emulated_rcdata.toggle_f = (emulated_rcdata.toggle_f + 1) if emulated_rcdata.toggle_f < 2 else 0

    except:
        pass



def on_release(key):
    global emulated_rcdata

    
    
    # left joystick (uses "WASD" keys)
    try:
        if key.char == 'w':
            emulated_rcdata.joystick_left_y = 0.
        elif key.char == 'd':
            emulated_rcdata.joystick_left_x = 0.
        elif key.char == 's':
            emulated_rcdata.joystick_left_y = 0.
        elif key.char == 'a':
            emulated_rcdata.joystick_left_x = 0.
    except:
        pass
    
    # right joystick (uses arrow keys)
    try:
        if key == keyboard.Key.up:
            emulated_rcdata.joystick_right_y = 0.
        elif key == keyboard.Key.right:
            emulated_rcdata.joystick_right_x = 0.
        elif key == keyboard.Key.down:
            emulated_rcdata.joystick_right_y = 0.
        elif key == keyboard.Key.left:
            emulated_rcdata.joystick_right_x = 0.
    except:
        pass


def start_key_listener():
    keyboard.Listener(on_press=on_press, on_release=on_release).start()






def main(args=None):
    rclpy.init(args=args)
    start_key_listener()
    rc_emulator_publisher = RCEmulatorKeyboard()
    rclpy.spin(rc_emulator_publisher)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
