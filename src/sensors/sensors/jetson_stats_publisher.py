import rclpy
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from autoboat_msgs.msg import JetsonStats
from jtop import jtop


# Here is an example of the result of jetson.stats:
# {
#   'time': datetime.datetime(2025, 12, 16, 17, 14, 33, 703152), 'uptime': datetime.timedelta(days=3, seconds=455, microseconds=540000), 
#   'CPU1': 11, 'CPU2': 11, 'CPU3': 19, 'CPU4': 17, 'CPU5': 6, 'CPU6': 7, 
#   'RAM': 0.2971710905063411, 'SWAP': 0.003871486339066742, 
#   'EMC': 0, 'GPU': 26.7, 'APE': 'OFF', 'NVDEC': 'OFF', 'NVJPG': 'OFF', 'NVJPG1': 'OFF', 'OFA': 'OFF', 'SE': 'OFF', 'VIC': 'OFF', 
#   'Fan pwmfan0': 21.96078431372549, 
#   'Temp cpu': 44.656, 'Temp cv0': -256, 'Temp cv1': -256, 'Temp cv2': -256, 'Temp gpu': 43.718, 
#   'Temp soc0': 42.0, 'Temp soc1': 41.937, 'Temp soc2': 43.5, 'Temp tj': 44.656, 
#   'Power VDD_CPU_GPU_CV': 881, 'Power VDD_SOC': 1121, 'Power TOT': 3852, 'jetson_clocks': 'OFF', 'nvp model': '15W
#'}


class TempPowerStatsPublisher(Node):

    def __init__(self):
        
        super().__init__("jetson_stats_publisher")
        
        self.jetson_stats_publisher = self.create_publisher(JetsonStats, "jetson_stats", qos_profile_sensor_data)
        self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        
        jetson_stats = None
        
        with jtop() as jetson:
            if jetson.ok():
                jetson_stats = jetson.stats
            
            else: return
        
        jetson_stats_message = JetsonStats()
        
        jetson_stats_message.gpu_temperature_celcius = float(jetson_stats["Temp gpu"])
        jetson_stats_message.cpu_temperature_celcius = float(jetson_stats["Temp cpu"])
        jetson_stats_message.soc_temperature_celcius = (float(jetson_stats["Temp soc0"]) + float(jetson_stats["Temp soc1"]) + float(jetson_stats["Temp soc2"])) / 3

        jetson_stats_message.soc_power_usage_watts = float(jetson_stats['Power VDD_SOC']) / 1000.
        jetson_stats_message.total_power_usage_watts = float(jetson_stats['Power TOT']) / 1000.
        jetson_stats_message.max_power_usage_watts = float(jetson_stats['nvp model'].rstrip("W"))
        
        
        # Get all CPUs that the jetson has and get the utilization percent for each of them and average that
        cpu_utilization_percents = []
        for key, value in jetson_stats.items():
            if "CPU" in key:
                try: 
                    int(key.removeprefix("CPU"))      # eg. CPU1, CPU2, etc. If it is not in this form then it isn't a cpu utilization percent
                except:
                    continue
            
            cpu_utilization_percents.append(value) 
        
            
        jetson_stats_message.cpu_utilization_percent = float(sum(cpu_utilization_percents)/len(cpu_utilization_percents))
        jetson_stats_message.ram_utilization_percent = float(jetson_stats['RAM']) * 100
        jetson_stats_message.gpu_utilization_percent = float(jetson_stats['GPU'])
        
        self.jetson_stats_publisher.publish(jetson_stats_message)
        
        
        
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TempPowerStatsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()