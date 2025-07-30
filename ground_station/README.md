# Ground Station <img src="https://github.com/user-attachments/assets/05a3d1d7-f5c2-4c9b-8a05-54f5ed727f80" alt="logo" width="50"/>

This tool offers an intuitive interface that allows users to monitor telemetry data and set route waypoints while the boat is in operation.

## Getting Started

### Prerequisites

Ensure that you have the following software installed on your system:

- [Go](https://go.dev/doc/install) (>= 1.18.1)
- [Python](https://www.python.org/downloads/) (3.10.12 to try and mimic the docker environment)

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
