# Samina is working on this

import serial, time
import rclpy
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from jtop import jtop


class TempPowerStatsPublisher(Node):

    def __init__(self):
        self.temp = {}
        self.power = {}
        self.j_stats = {}
        super().__init__("jetson_stats")
        self.temp_gpu_publisher = self.create_publisher(Float32, "/temp_cpu", 10)
        self.temp_cpu_publisher = self.create_publisher(Float32, "/temp_gpu", 10)
        self.temp_soc_publisher = self.create_publisher(Float32, "/temp_soc", 10)
        self.temp_tj_publisher = self.create_publisher(Float32, "/temp_tj", 10)
        self.power_cgc_publisher = self.create_publisher(Float32, "/power_cgc", 10)
        self.power_soc_publisher = self.create_publisher(Float32, "/power_soc", 10)
        self.power_total_publisher = self.create_publisher(Float32, "/power_total", 10)
        self.power_max_publisher = self.create_publisher(Float32, "/power_max", 10)
        self.create_timer(0.1, self.timer_callback)

    def jetson_stats(self):
        with jtop() as jetson:
            if jetson.ok():
                self.j_stats = jetson.stats

    def send_gpu_temp(self):
        gpu_temp = float(self.j_stats["Temp gpu"])
        self.temp_gpu_publisher.publish(Float32(data=gpu_temp))
    
    def send_cpu_temp(self):
        cpu_temp = float(self.j_stats["Temp cpu"])
        self.temp_cpu_publisher.publish(Float32(data=cpu_temp))
    
    def send_soc_temp(self):
        # average of soc0, soc1, soc2
        soc_temp = float(self.j_stats["Temp soc0"]) + float(self.j_stats["Temp soc1"]) + float(self.j_stats["Temp soc2"])
        self.temp_soc_publisher.publish(Float32(data=soc_temp))
    
    def send_tj_temp(self):
        tj_temp = float(self.j_stats["Temp tj"])
        self.temp_tj_publisher.publish(Float32(data=tj_temp))
    
    def send_cgc_power(self):
        # cgc = cpu, gpu, and cv
        cgc_power = float(self.j_stats['Power VDD_CPU_GPU_CV'])
        self.power_cgc_publisher.publish(Float32(data=cgc_power))

    
    def send_soc_power(self):
        soc_power = float(self.j_stats['Power VDD_SOC'])
        self.power_soc_publisher.publish(Float32(data=soc_power))

    
    def send_total_power(self):
        total_power = float(self.j_stats['Power TOT'])
        self.power_total_publisher.publish(Float32(data=total_power))

    
    def send_max_power(self):
        max_power = float(self.j_stats['nvp model'].rstrip("W"))
        self.power_max_publisher.publish(Float32(data=max_power))
    
    def timer_callback(self):

        self.jetson_stats()
        self.send_gpu_temp()
        self.send_cpu_temp()
        self.send_soc_temp()
        self.send_tj_temp()
        self.send_cgc_power()
        self.send_soc_power()
        self.send_total_power()
        self.send_max_power()


def main(args=None):
    rclpy.init(args=args)
    node = TempPowerStatsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()