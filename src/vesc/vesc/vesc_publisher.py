import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node

from autoboat_msgs.msg import VESCTelemetryData, VESCControlData

from pyvesc.VESC.messages import *

from pyvesc import VESC
import time
from serial.tools import list_ports
import os, signal



MOTOR_POLE_PAIRS = 7

# VESC_VID = 0x0403
# VESC_PID = 0x6001


VESC_VID = 0x0483
VESC_PID = 0x5740

# VESC_SERIAL_NUMBER = "AB7IMXEU"

def getPort(vid, pid) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError('Device not found')




class VESCPublisher(Node):

    def __init__(self):
        super().__init__('pyvesc_publisher')
        self.ser = getPort( VESC_VID, VESC_PID)
        
        # self.ser = getPorts(0x0403, 0x6001, VESC_SERIAL_NUMBER)
        try:
            self.motor = VESC(serial_port= self.ser)
        except Exception as e:
            self.get_logger().error(f"failed to connect to the motor: {e}")
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getpid(), signal.SIGTERM)

        self.motorVal = 0
        self.motorType = 0 # 1-duty cycle 2-rpm 3-current 
        self.missed_measurements_in_a_row = 0
        self.last_command_time = 0
        
        self.controlTypeSub = self.create_subscription(msg_type= VESCControlData, topic='/propeller_motor_control_struct', callback=self.receive_control_data_callback, qos_profile=qos_profile_sensor_data)
        
        self.vesc_telemetry_data_publisher = self.create_publisher(VESCTelemetryData, "/vesc_telemetry_data", qos_profile_sensor_data)
        
        
        timer_period = 0.05  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    
    def receive_control_data_callback(self, msg: VESCControlData):
        self.last_command_time = time.time()
        
        self.get_logger().info(f'{self.motorVal}')
        
        try:
            if(msg.control_type_for_vesc == "rpm"):
                self.motorVal = msg.desired_vesc_rpm * MOTOR_POLE_PAIRS
                self.motor.set_rpm(int(self.motorVal))
            elif(msg.control_type_for_vesc == "duty_cycle"):
                self.motorVal = msg.desired_vesc_duty_cycle
                self.motor.set_duty_cycle(int(self.motorVal))
            else:
                self.motorVal = msg.desired_vesc_current
                self.motor.set_current(int(self.motorVal))
        except:
            self.get_logger().error("Disconnected from the VESC")
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getpid(), signal.SIGTERM)
            
    """
    def ct_callback(self, msg):
        if msg == "DUTY_CYCLE":
            self.motorType = 1
        if msg == "RPM":
            self.motorType = 2
        if msg == "CURRENT":
            self.motorType = 3

    def cv_callback(self, msg):
        if self.motorType == 3:
            self.motorVal = msg

    def rpmv_callback(self, msg):
        if self.motorType == 2:
            self.motorVal = msg

    def dcv_callback(self, msg):
        if self.motorType == 1:
            self.motorVal = msg
    """

    def timer_callback(self):
        
        if (time.time() - self.last_command_time >= 3):
            self.motor.set_rpm(0)
        
        #get data and store in dictionary
        measurements = self.motor.get_measurements()
        # try:
        #     measurements = self.get_motor_measurements()
        # except:
        #      self.get_logger().error("Disconnected from the VESC")
        #      self.destroy_node()
        #      rclpy.shutdown()
        #      os.kill(os.getpid(), signal.SIGTERM)
        
        if not measurements:
            self.missed_measurements_in_a_row += 1
            # if (self.missed_measurements_in_a_row >= 20):
            #     self.get_logger().error("Disconnected from the VESC")
            #     self.destroy_node()
            #     rclpy.shutdown()
            #     os.kill(os.getpid(), signal.SIGTERM)
            
            return
        
        else:
            self.missed_measurements_in_a_row = 0
        
        rpm = measurements.rpm/MOTOR_POLE_PAIRS
        c_motor = measurements.avg_motor_current
        motorData = {
            "time": time.time(),
            "rpm": measurements.rpm/MOTOR_POLE_PAIRS,
            "duty_cycle": measurements.duty_cycle_now,
            "v_in": measurements.v_in,
            "c_in": measurements.avg_input_current,
            "c_motor": measurements.avg_motor_current,
            "temp_motor": measurements.temp_motor, 
            "temp_vesc": measurements.temp_fet,
            "time_ms": measurements.time_ms,
            "amp_hours": measurements.amp_hours, 
            "amp_hours_charged": measurements.amp_hours_charged,
            "motor_wattage": c_motor*rpm/180,
            "v_out": rpm/180
        }

        #write vesc data to csv file (this doesn't work with systemctl automatic startup on boot)
        # self.csv_writer.writerow(motorData)
        
        #publish vesc data to topic
        self.vesc_telemetry_data_publisher.publish(
            VESCTelemetryData(
                rpm = motorData["rpm"], duty_cycle = motorData["duty_cycle"], 
                voltage_to_vesc = motorData["v_in"], current_to_vesc = motorData["c_in"],
                voltage_to_motor = motorData["v_out"], avg_current_to_motor = motorData["c_motor"],
                wattage_to_motor = motorData["motor_wattage"], motor_temperature = motorData["temp_motor"],
                vesc_temperature = motorData["temp_vesc"], time_since_vesc_startup_in_ms = motorData["time_ms"], 
                amp_hours = motorData["amp_hours"], amp_hours_charged = motorData["amp_hours_charged"]  
            )
        )
    

    def __del__(self):
        if hasattr(self, "motor"):
            self.motor.stop_heartbeat()



def main(args=None):
    
    rclpy.init(args=args)
    vesc_publisher = VESCPublisher()
    rclpy.spin(vesc_publisher)
    
    # vesc_publisher.destroy_node()
    rclpy.shutdown()





#Pragya - Yes, you can play tetris on the phallic ice cream