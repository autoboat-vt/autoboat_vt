#!/bin/bash
set -e



export DEBIAN_FRONTEND=noninteractive
export TZ=Etc/UTC

# Pre-seed tzdata answers for debconf to ensure no interactive prompts
echo "tzdata tzdata/Areas select Etc" | sudo debconf-set-selections 2>/dev/null || true
echo "tzdata tzdata/Zones/Etc select UTC" | sudo debconf-set-selections 2>/dev/null || true

# Provide /etc/timezone file which is often checked by tzdata scripts
echo "Etc/UTC" | sudo tee /etc/timezone > /dev/null

# Manual link as a secondary safety measure
sudo ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime




ARCH=$(dpkg --print-architecture)
if [[ "$ARCH" != "amd64" && "$ARCH" != "arm64" ]]; then
    echo "Error: Unsupported architecture: $ARCH. This package only supports amd64 and arm64."
    exit 1
fi

# Argument Parsing
INSTALL_BASE=false
INSTALL_SIMULATION=false
INSTALL_MICROROS_AGENT=false
INSTALL_FIRMWARE_SDK=false
SHOW_HELP=false

if [[ $# -eq 0 ]]; then
    # Default to base if no arguments provided
    INSTALL_BASE=true
else
    for arg in "$@"; do
        case $arg in
            --base) INSTALL_BASE=true ;;
            --simulation|--sim) INSTALL_SIMULATION=true ;;
            --firmware-dependencies|--firm) INSTALL_FIRMWARE_SDK=true ;;
            --microros-agent|--agent) INSTALL_MICROROS_AGENT=true ;;
            --all) INSTALL_BASE=true; INSTALL_SIMULATION=true; INSTALL_FIRMWARE_SDK=true; INSTALL_MICROROS_AGENT=true ;;
            --help|-h) SHOW_HELP=true ;;
            *) echo "Unknown option: $arg"; SHOW_HELP=true ;;
        esac
    done
fi

if [ "$SHOW_HELP" = true ]; then
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --base            Install the base autoboatvt package (default)"
    echo "  --simulation      Install the simulation package (adds Gazebo repo to the base installation)"
    echo "  --firmware-dependencies    Install the autoboat firmware SDK package"
    echo "  --microros-agent  Install only the microros agent runtime"
    echo "  --all             Install all of the above"
    echo "  --help, -h        Show this help message"
    exit 0
fi

echo "--------------------------------------------------------"
echo "Starting autoboatvt installation for $ARCH..."
echo "Requested packages:"
[ "$INSTALL_BASE" = true ] && echo " - Base system (autoboatvt)"
[ "$INSTALL_SIMULATION" = true ] && echo " - Simulation (autoboatvt-simulation)"
[ "$INSTALL_FIRMWARE_SDK" = true ] && echo " - Autoboat Firmware SDK (autoboatvt-firmware-dependencies)"
[ "$INSTALL_MICROROS_AGENT" = true ] && echo " - MicroROS Agent (autoboatvt-microros-agent)"
echo "--------------------------------------------------------"

# Add ROS 2 Repositories if missing
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "==> Configuring ROS 2 repositories..."
    sudo DEBIAN_FRONTEND=noninteractive apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y curl software-properties-common lsb-release
    sudo add-apt-repository -y universe

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    echo "==> Updating apt cache..."
    sudo DEBIAN_FRONTEND=noninteractive apt-get update
else
    echo "==> ROS 2 repositories already configured."
fi

# Add OSRF Gazebo Repositories if missing (Required for simulation)
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    echo "==> Configuring OSRF Gazebo repositories..."
    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    echo "==> Updating apt cache..."
    sudo DEBIAN_FRONTEND=noninteractive apt-get update
else
    echo "==> OSRF Gazebo repositories already configured."
fi





install_deb() {
    local PKG_NAME=$1
    local DEB_NAME=$2
    
    # Check if actually installed (avoiding false positives from 'rc' state)
    if dpkg-query -W -f='${Status}' "$PKG_NAME" 2>/dev/null | grep -q "ok installed"; then
        echo "==> Package '$PKG_NAME' is already installed. Skipping..."
        return 0
    fi

    echo "==> Installing $PKG_NAME ($DEB_NAME)..."
    DOWNLOAD_URL="https://github.com/autoboat-vt/autoboat_vt/releases/latest/download/${DEB_NAME}"
    
    if ! curl -fsSL -L -o "/tmp/${DEB_NAME}" "$DOWNLOAD_URL"; then
        echo "Error: Failed to download ${DEB_NAME} from ${DOWNLOAD_URL}"
        return 1
    fi

    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "/tmp/${DEB_NAME}"
    rm "/tmp/${DEB_NAME}"
}


# Install agent if explicitly requested or needed as a dependency
if [ "$INSTALL_MICROROS_AGENT" = true ] || [ "$INSTALL_BASE" = true ] || [ "$INSTALL_FIRMWARE_SDK" = true ]; then
    install_deb "autoboatvt-microros-agent" "autoboatvt-microros-agent-${ARCH}.deb"
fi

# Base package
if [ "$INSTALL_BASE" = true ]; then
    install_deb "autoboatvt" "autoboatvt-${ARCH}.deb"
fi

if [ "$INSTALL_SIMULATION" = true ]; then
    if [ "$ARCH" != "amd64" ]; then
        echo "WARNING: Simulation package is only available for amd64. Skipping..."
    else
        # Ensure base is installed if requested simulation
        install_deb "autoboatvt-microros-agent" "autoboatvt-microros-agent-${ARCH}.deb"
        install_deb "autoboatvt" "autoboatvt-${ARCH}.deb"
        install_deb "autoboatvt-simulation" "autoboatvt-simulation-${ARCH}.deb"
    fi
fi


if [ "$INSTALL_FIRMWARE_SDK" = true ]; then
    install_deb "autoboatvt-firmware-dependencies" "autoboatvt-firmware-dependencies-${ARCH}.deb"
fi





echo "--------------------------------------------------------"
echo "Autoboat Installation complete!"
echo "--------------------------------------------------------"
echo "Please open a NEW terminal or run:"
echo "  source ~/.bashrc"
echo "--------------------------------------------------------"
