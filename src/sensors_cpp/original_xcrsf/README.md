# XpressCRSF
**XpressCRSF** is a fast and lightweight `CRSF (Crossfire)` protocol parser for ExpressLRS receivers on Raspberry Pi. 
It operates at the default baud rate of 420,000, using the termios library for efficient serial communication.
XpressCRSF enables real-time parsing of telemetry and control data on Raspberry Pi.

## License:
[![License](https://img.shields.io/badge/License-MIT-blue.svg?longCache=true&style=flat)](https://github.com/Vinz1911/xcrsf/blob/master/LICENSE)

## C++ Version:
[![C++23](https://img.shields.io/badge/C++-23-blue.svg?logo=c%2B%2B&style=flat)](https://isocpp.org)

# Build and Install:

```shell
# build and install libraries
mkdir build
cd build/
cmake ..
make && sudo make install
```

# Import:

```c++
#include "xcrsf/crossfire.h"

auto crossfire = crossfire::XCrossfire("/dev/ttyAMA10");
crossfire.open_port()
```

# Usage:
## Receive Channel Information

```c++
int main() {
    // Create instance of XCrossfire
    auto crossfire = crossfire::XCrossfire("/dev/ttyAMA10");
    if (crossfire.open_port()) { std::printf("Port opened...\n"); }

    while (crossfire.is_paired()) {
        // Get channel data
        const auto channels = crossfire.get_channels();
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Get link statistics
    const auto link = crossfire.get_link_state();
    std::printf("RSSI Antenna 1 %u, Antenna 2 %u\n", link.uplink_rssi_antenna_1, link.uplink_rssi_antenna_2);
    return 0;
}
```

## Send Telemetry Information

```c++
int main() {
    // Create instance of XCrossfire
    auto crossfire = crossfire::XCrossfire("/dev/ttyAMA10");
    if (crossfire.open_port()) { std::printf("Port opened...\n"); }

    // Send Battery Telemetry
    crossfire.set_telemetry_battery(12.0, 16.0, 8500, 95);

    // Send Vario (Vertical Speed) Telemetry
    crossfire.set_telemetry_vario(100);

    // Send Attitude (Pitch, Roll, Yaw) Telemetry
    crossfire.set_telemetry_attitude(1.0, 1.0, -2.0);

    // Send GPS Telemetry
    crossfire.set_telemetry_gps(40.234, -37.365, 100, 100, 100, 2);
    return 0;
}
```