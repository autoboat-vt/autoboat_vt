# Groundstation

This tool offers an intuitive interface that allows users to monitor telemetry data and set route waypoints while the boat is in operation.

## Getting Started

### Prerequisites

Ensure that you have the following software installed on your system:

- [Python](https://www.python.org/downloads) (3.10.12 to try and mimic the docker environment)
- [Bun](https://bun.sh) (whatever the latest version is at the time you're reading this)

### Installation

1. Give the start script executable permissions:

    ```bash
    chmod +x run.sh
    ```

2. Run the program

    ```bash
    ./run.sh
    ```

### Usage

- Left click on the map to add waypoints.
- Right click to remove the waypoint closest to your mouse's cursor position.
- Click on waypoint or buoy in the table to zoom to that marker on the map.

### Differential-thrust motorboat simulation

Run the ROS simulation and ground station in separate terminals from the workspace:

```bash
# Terminal 1
source install/setup.bash
ros2 launch autoboat_launch motorboat_simulation_differential_thrust.launch.py
```

```bash
# Terminal 2
cd ground_station
./run.sh
```

The ROS launch starts Gazebo, the differential-thrust autopilot, and telemetry. The
ground station displays boat status, control mode, desired heading, and measured
port/starboard propeller RPM. Build and source the workspace after changing ROS
packages so the installed launch file and autopilot executable are current.
