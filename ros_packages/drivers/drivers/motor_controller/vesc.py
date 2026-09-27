import os
import signal
import time

import rclpy
from autoboat_msgs.msg import VESCControlData, VESCTelemetryData
from pyvesc.pyvesc import VESC
from pyvesc.pyvesc.VESC.messages import *
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from serial.tools import list_ports

MOTOR_POLE_PAIRS = 7

# VESC_VID = 0x0403
# VESC_PID = 0x6001


VESC_VID = 0x0483
VESC_PID = 0x5740


# VESC_SERIAL_NUMBER = "AB7IMXEU"


def getPort(vid: int, pid: int) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError("Device not found")




class VESCPublisher(Node):
    def __init__(self):
        super().__init__("vesc_publisher")
        self.serial_port = getPort(VESC_VID, VESC_PID)

        try:
            self.motor = VESC(serial_port=self.serial_port)
        except Exception as e:
            self.get_logger().error(f"failed to connect to the motor: {e}")
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getpid(), signal.SIGTERM)

        self.motorVal = 0
        self.motorType = 0  # 1-duty cycle 2-rpm 3-current
        self.missed_measurements_in_a_row = 0
        self.last_command_time = 0

        self.create_subscription(
            VESCControlData, "/propeller_motor_control_struct", self.receive_control_data_callback, qos_profile_sensor_data
        )
        self.vesc_telemetry_data_publisher = self.create_publisher(
            VESCTelemetryData, "/vesc_telemetry_data", qos_profile_sensor_data
        )

        timer_period = 0.05  # seconds

        self.create_timer(timer_period, self.timer_callback)


    def receive_control_data_callback(self, msg: VESCControlData):
        self.last_command_time = time.time()

        self.get_logger().info(f"{self.motorVal}")

        try:
            if msg.control_type_for_vesc == "rpm":
                self.motorVal = msg.control_value * MOTOR_POLE_PAIRS
                self.motor.set_rpm(int(self.motorVal))
                
            elif(msg.control_type_for_vesc == "duty_cycle"):
                self.motorVal = msg.control_value
                self.motor.set_duty_cycle(int(self.motorVal))
            
            else:
                self.motorVal = msg.control_value
                self.motor.set_current(int(self.motorVal))
                
        except Exception:
            self.get_logger().error("Disconnected from the VESC")
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getpid(), signal.SIGTERM)
            
            

    def timer_callback(self):
        # if (time.time() - self.last_command_time >= 3):
        #     self.motor.set_rpm(0)

        # get data and store in dictionary
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

        rpm = measurements.rpm / MOTOR_POLE_PAIRS
        c_motor = measurements.avg_motor_current
        motor_telemetry_data = {
            "time": time.time(),
            "rpm": measurements.rpm / MOTOR_POLE_PAIRS,
            "duty_cycle": measurements.duty_cycle_now,
            "v_in": measurements.v_in,
            "c_in": measurements.avg_input_current,
            "c_motor": measurements.avg_motor_current,
            "temp_motor": measurements.temp_motor,
            "temp_vesc": measurements.temp_fet,
            "time_ms": measurements.time_ms,
            "amp_hours": measurements.amp_hours,
            "amp_hours_charged": measurements.amp_hours_charged,
            "motor_wattage": c_motor * rpm / 180,
            "v_out": rpm / 180,
        }

        # write vesc data to csv file (this doesn't work with systemctl automatic startup on boot)
        # self.csv_writer.writerow(motor_telemetry_data)

        # publish vesc data to topic
        self.vesc_telemetry_data_publisher.publish(
            VESCTelemetryData(
                rpm=motor_telemetry_data["rpm"],
                duty_cycle=motor_telemetry_data["duty_cycle"],
                voltage_to_vesc=motor_telemetry_data["v_in"],
                current_to_vesc=motor_telemetry_data["c_in"],
                voltage_to_motor=motor_telemetry_data["v_out"],
                avg_current_to_motor=motor_telemetry_data["c_motor"],
                wattage_to_motor=motor_telemetry_data["motor_wattage"],
                motor_temperature=motor_telemetry_data["temp_motor"],
                vesc_temperature=motor_telemetry_data["temp_vesc"],
                time_since_vesc_startup_in_ms=motor_telemetry_data["time_ms"],
                amp_hours=motor_telemetry_data["amp_hours"],
                amp_hours_charged=motor_telemetry_data["amp_hours_charged"],
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


# Pragya - Yes, you can play tetris on the phallic ice cream
