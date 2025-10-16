from jtop import jtop
import time

with jtop() as jetson:
    while jetson.ok():
        stats = jetson.stats
        print(f"GPU Temperature: {stats['Temp gpu']} °C") # data for jetson is givwn as a dictionary (continued)
        print(f"CPU Temperature: {stats['Temp cpu']} °C") # keys to access dictionar; "Temp cpu/gpu respectively"
        print("")
        print(f"SoC 0 Temperature: {stats['Temp soc0']} °C") # SoC == Systems on a Chip
        print(f"SoC 1 Temperature: {stats['Temp soc1']} °C") 
        print(f"SoC 2 Temperature: {stats['Temp soc2']} °C")
        print("")
        print(f"TJ Temperature: {stats['Temp tj']} °C")  # TJ = Junction Temperature
        print("")
        print(f"CPU, GPU, CV Power Output (W): {stats['Power VDD_CPU_GPU_CV']/1000}W")
        print(f"SoC Power Output (W): {stats['Power VDD_SOC']/1000}W")
        print(f"Total Power Output (W): {stats['Power TOT']/1000}W")
        print(f"Max Power Output (W): {stats['nvp model']}")
        print("")
        print("")
        time.sleep(10) # has the while loop wait for 10 seconds before it runs again

# find: soc temp, tj temp, what overall chip power should be at MAX (15W) and what it is currently 
# last two bits of info is very important bc it drains battery


def main(args=None):
    print("hi")