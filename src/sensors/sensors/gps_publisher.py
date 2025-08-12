#!/usr/bin/python3

# ADAPTED FROM: https://github.com/FrankBu0616/ros2_zed_f9r_gps

import serial, time

from rclpy.qos import qos_profile_sensor_data
import rclpy

from serial.tools import list_ports
from rclpy.node import Node
from ublox_gps import UbloxGps
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3, Twist

import numpy as np
from collections import deque
import csv

# https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
# pg 443
# NOTE: Didn't use last year because we weren't able to figure out how to get rtcm working on our gps
RTCM_MESSAGE_CLASS = 0xF5
RTCM_MESSAGE_ID = 0x05



GPS_VID = 0x1546
GPS_PID = 0x01a8
BAUD_RATE = 38400

REFRESH_RATE = 10 # HERTZ



def getPort(vid, pid) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError('Device not found')

def sum_integers(integer):
    """Sums up all positive integers less than or equal to the number that was passed in"""
    # nothing but this formula: https://www.youtube.com/watch?app=desktop&v=bWZwF1H9YbU
    return (integer * (integer + 1))/ 2

def linear_moving_weighted_average(gps_data):
    weighted_list_1 = []
    weighted_list_2 = []
    for index, (data1, data2) in enumerate(gps_data):
        weighted_list_1.append(data1 * (index+1))
        weighted_list_2.append(data2 * (index+1))

    length = len(gps_data) 
    return (sum(weighted_list_1)/ sum_integers(length)), sum(weighted_list_2)/ sum_integers(length)






class GPSPublisher(Node):
    """
    Reads GPS data from the GPS over a serial USB connection and then publishes that data so that the autopilot can use it
    
    This node publishes the current position and velocity
    """
    
    
    def __init__(self):
        super().__init__('gps_publisher')
        
        self.position_publisher = self.create_publisher(NavSatFix, '/position', qos_profile_sensor_data)
        self.velocity_publisher = self.create_publisher(Twist, '/velocity', qos_profile_sensor_data)

        # self.csv_writer = csv.DictWriter(open("gps_data.csv", "w+"), fieldnames=["time", "SOG", "velocity_east", "velocity_north"])
        # self.csv_writer.writeheader()
        
        self.create_timer(1 / REFRESH_RATE, self.publish)

        serial_port = getPort(GPS_VID, GPS_PID)
        self.sensor_serial = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=1)
        self.gps = UbloxGps(self.sensor_serial)
            
        self.rtcm_correction_data = bytearray()
        
        self.gps_velocity_data_queue = deque(maxlen=10)
    

    
    def publish(self):

        geo = self.gps.geo_coords()
                    
        if not geo: return
        
        self.gps_velocity_data_queue.append((geo.velE, geo.velN))
        
        velE, velN = linear_moving_weighted_average(self.gps_velocity_data_queue)
        
        gps_msg = NavSatFix(longitude=geo.lon, latitude=geo.lat)
        linear_velocity_msg = Vector3(x=float(velE/1000), y=float(velN/1000))
        velocity_msg = Twist(linear=linear_velocity_msg)
        
        velE_mph = velE * 2.2369/ 1000 # mm/s to mph
        velN_mph = velN * 2.2369/ 1000 # mm/s to mph

        # self.csv_writer.writerow({"time": time.time(), "SOG": np.sqrt(velE_mph**2 + velN_mph**2), "velocity_east": velE_mph, "velocity_north": velN_mph})
        
        
        print(f"velocity vector (mph): <{float(velE_mph)}, {float(velN_mph)}>")
        print(f"SOG (mph): {np.sqrt(velE_mph**2 + velN_mph**2)}")
        print(f"DIR: {np.rad2deg(np.arctan2(velN_mph, velE_mph))}")
        print(f"Acc: {geo.sAcc}")
        print(f"Sats: {geo.numSV}")
        
        self.position_publisher.publish(gps_msg)
        self.velocity_publisher.publish(velocity_msg)


    def __del__(self):
        self.sensor_serial.close()
        self.destroy_node()
        rclpy.shutdown()






def main(args=None):
    
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    
    gps_publisher.destroy_node()
    rclpy.shutdown()
