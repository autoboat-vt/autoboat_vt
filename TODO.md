add actual rudder and sail angles to the telemetry and not just the desired rudder and sail angles


Make the telemetry data usage better by compressing data


motorboat simulation


make it so that the telemetry server node knows what the default parameters are 

EXTREMELY HIGH PRIORITY: figure out how to handle the default parameters on the telemetry server node because right now, it only uses the sailboat default parameters



https://github.com/Gabo-Tor/gym-sailing <- make this into its own simulation node



HIGH PRIORITY: Make sure that the gps, pico, rc, and wind sensor usbs work with the udev rules and test them


Add motor config files to the VESC node so that we can have different configs for motor poles, max current, max voltage etc for each motor. We would be able to set hard limits on the data going to the VESC and also not have to update a global variable whenever we would like to update the motor pole count 


MAKE SURE THAT THE VELOCITY VECTOR IS DEFINED AS THE X COORDINATE IS IN LINE WITH THE CENTERLINE AND THE Y AXIS IS TO THE LEFT OF THE CENTERLINE


Add is it water api to autopilot for collision avoidance and navigation