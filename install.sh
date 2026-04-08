#!/bin/bash
set -e
# 0. Headless configuration
export DEBIAN_FRONTEND=noninteractive
export TZ=Etc/UTC

# Pre-seed tzdata answers for debconf to ensure no interactive prompts
echo "tzdata tzdata/Areas select Etc" | sudo debconf-set-selections 2>/dev/null || true
echo "tzdata tzdata/Zones/Etc select UTC" | sudo debconf-set-selections 2>/dev/null || true

# Provide /etc/timezone file which is often checked by tzdata scripts
echo "Etc/UTC" | sudo tee /etc/timezone > /dev/null

# Manual link as a secondary safety measure
sudo ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime



# 1. Detect architecture
ARCH=$(dpkg --print-architecture)
if [[ "$ARCH" != "amd64" && "$ARCH" != "arm64" ]]; then
    echo "Error: Unsupported architecture: $ARCH. This package only supports amd64 and arm64."
    exit 1
fi

# 2. Argument Parsing
INSTALL_BASE=false
INSTALL_SIMULATION=false
INSTALL_MICROROS=false
SHOW_HELP=false

if [[ $# -eq 0 ]]; then
    # Default to base if no arguments provided
    INSTALL_BASE=true
else
    for arg in "$@"; do
        case $arg in
            --base) INSTALL_BASE=true ;;
            --simulation|--sim) INSTALL_SIMULATION=true ;;
            --microros|--uC) INSTALL_MICROROS=true ;;
            --all) INSTALL_BASE=true; INSTALL_SIMULATION=true; INSTALL_MICROROS=true ;;
            --help|-h) SHOW_HELP=true ;;
            *) echo "Unknown option: $arg"; SHOW_HELP=true ;;
        esac
    done
fi

if [ "$SHOW_HELP" = true ]; then
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --base         Install the base autoboat-vt package (default)"
    echo "  --simulation   Install the simulation package (adds Gazebo repo)"
    echo "  --microros     Install the Micro-ROS SDK package"
    echo "  --all          Install all of the above"
    echo "  --help, -h     Show this help message"
    exit 0
fi

echo "--------------------------------------------------------"
echo "Starting autoboat-vt installation for $ARCH..."
echo "Requested packages:"
[ "$INSTALL_BASE" = true ] && echo " - Base system (autoboatvt)"
[ "$INSTALL_SIMULATION" = true ] && echo " - Simulation (autoboatvt-simulation)"
[ "$INSTALL_MICROROS" = true ] && echo " - Micro-ROS SDK (autoboatvt-microros)"
echo "--------------------------------------------------------"

# 3. Add ROS 2 Repositories if missing
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

# 4. Add OSRF Gazebo Repositories if missing (Required for simulation)
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    echo "==> Configuring OSRF Gazebo repositories..."
    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    echo "==> Updating apt cache..."
    sudo DEBIAN_FRONTEND=noninteractive apt-get update
else
    echo "==> OSRF Gazebo repositories already configured."
fi

# 5. Download and Install Packages
install_deb() {
    local PKG_NAME=$1
    local DEB_NAME=$2
    
    # Check if already installed
    if dpkg -s "$PKG_NAME" >/dev/null 2>&1; then
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

# Base package is a dependency for others, but we check separately
if [ "$INSTALL_BASE" = true ]; then
    install_deb "autoboatvt" "autoboat-vt-${ARCH}.deb"
fi

if [ "$INSTALL_SIMULATION" = true ]; then
    if [ "$ARCH" != "amd64" ]; then
        echo "WARNING: Simulation package is only available for amd64. Skipping..."
    else
        # Ensure base is installed if requested simulation
        install_deb "autoboatvt" "autoboat-vt-${ARCH}.deb"
        install_deb "autoboatvt-simulation" "autoboat-vt-simulation-${ARCH}.deb"
    fi
fi

if [ "$INSTALL_MICROROS" = true ]; then
    # Ensure base is installed if requested microros
    install_deb "autoboatvt" "autoboat-vt-${ARCH}.deb"
    install_deb "autoboatvt-microros" "autoboat-vt-microros-full-${ARCH}.deb"
fi


echo "--------------------------------------------------------"
echo "Autoboat Installation complete!"
echo "--------------------------------------------------------"
echo "Please open a NEW terminal or run:"
echo "  source ~/.bashrc"
echo "--------------------------------------------------------"
