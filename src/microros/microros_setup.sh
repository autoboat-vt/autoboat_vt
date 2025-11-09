#!/bin/bash
set -e
SCRIPT_DIR="$(dirname "$(realpath "$0")")"


# Install dependencies
sudo apt install -y cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3 build-essential pkg-config libusb-1.0-0-dev
sudo apt-get update || true  # continue even if this "fails" for whatever reason
sudo apt-get upgrade -y
source /opt/ros/jazzy/setup.bash


# Install Pico SDK if it is not currently installed
if [ ! -d "/home/ws/src/microros/dependencies/pico-sdk" ]; then
  git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git /home/ws/src/microros/dependencies/pico-sdk
fi

export PICO_SDK_PATH=/home/ws/src/microros/dependencies/pico-sdk




# Install Microros Pico SDK if it is not currently installed
if [ ! -d "/home/ws/src/microros/dependencies/micro_ros_raspberrypi_pico_sdk" ]; then
  git clone -b jazzy https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git /home/ws/src/microros/dependencies/micro_ros_raspberrypi_pico_sdk
fi

export PICO_MICROROS_SDK_PATH=/home/ws/src/microros/dependencies/micro_ros_raspberrypi_pico_sdk




# Install Picotool if it is not already installed
if [ ! -d "/home/ws/src/microros/dependencies/picotool" ]; then
  git clone https://github.com/raspberrypi/picotool /home/ws/src/microros/dependencies/picotool
fi

cd /home/ws/src/microros/dependencies/picotool
export PICOTOOL_PATH=/home/ws/src/microros/dependencies/picotool
mkdir -p build && cd build
cmake .. && cmake --build . -j16




# Create and build microros workspace
if [ ! -d "/home/ws/src/microros/dependencies/micro_ros_agent/src/micro_ros_setup" ]; then
  git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git /home/ws/src/microros/dependencies/micro_ros_agent/src/micro_ros_setup
  cd /home/ws/src/microros/dependencies/micro_ros_agent/src/micro_ros_setup
  git reset --hard 5abfdaa59b0f18dc152b47b564d8e27012b05ac8 # They introduced a change that broke a lot of stuff. This is the only commit that works
fi



# Update ROS dependencies
cd /home/ws/src/microros/dependencies/micro_ros_agent
rosdep update && rosdep install --from-paths src --ignore-src -r -y


# Clean up stale symlinks 
find /home/ws/src/microros/dependencies/micro_ros_agent/build -type d -name "ament_cmake_python" -exec rm -rf {} + || true
find /home/ws/src/microros/dependencies/micro_ros_agent/install -type d -name "micro_ros_msgs" -exec rm -rf {} + || true

# Build the micro_ros_agent package
colcon build --symlink-install --parallel-workers 16
source /home/ws/src/microros/dependencies/micro_ros_agent/install/local_setup.bash


# Clean up a potentially old microros firware installation
rm -rf /home/ws/src/microros/dependencies/micro_ros_agent/firmware

# Build the microros firmware and agent
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh ros2 run



# Write sources and aliases to bashrc file
{
    echo "unset ROS_DOMAIN_ID"
    echo "export PICO_SDK_PATH=/home/ws/src/microros/dependencies/pico-sdk"
    echo "export PICO_MICROROS_SDK_PATH=/home/ws/src/microros/dependencies/micro_ros_raspberrypi_pico_sdk"
    echo "export PICOTOOL_PATH=/home/ws/src/microros/dependencies/picotool"
    echo "source /home/ws/src/microros/dependencies/micro_ros_agent/install/local_setup.bash"
    echo "alias picotool='$PICOTOOL_PATH/build/picotool'"
    echo "alias picotool_load='sudo $PICOTOOL_PATH/build/picotool load'"
    echo "alias picotool_reboot='sudo $PICOTOOL_PATH/build/picotool reboot'"
} >> ~/.bashrc




# Source bash
source ~/.bashrc

echo "Setup complete."