#!/bin/bash
set -e
# 0. Headless configuration
export DEBIAN_FRONTEND=noninteractive


# 1. Detect architecture
ARCH=$(dpkg --print-architecture)
if [[ "$ARCH" != "amd64" && "$ARCH" != "arm64" ]]; then
    echo "Error: Unsupported architecture: $ARCH. This package only supports amd64 and arm64."
    exit 1
fi

echo "--------------------------------------------------------"
echo "Starting autoboat-vt installation for $ARCH..."
echo "--------------------------------------------------------"

# 2. Add ROS 2 Repositories if missing
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "==> Configuring ROS 2 repositories..."
    sudo DEBIAN_FRONTEND=noninteractive apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y curl software-properties-common
    sudo add-apt-repository -y universe

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    echo "==> Updating apt cache..."
    sudo DEBIAN_FRONTEND=noninteractive apt-get update
else
    echo "==> ROS 2 repositories already configured."
fi

# 3. Download the latest .deb from GitHub
# Note: This uses the 'latest' tag we configured in the build-and-release workflow.
DEB_FILE="autoboat-vt-${ARCH}.deb"
DOWNLOAD_URL="https://github.com/autoboat-vt/autoboat_vt/releases/latest/download/${DEB_FILE}"

echo "==> Downloading latest package from GitHub..."
curl -fsSL -L -o "/tmp/${DEB_FILE}" "$DOWNLOAD_URL"

# 4. Install the package and its dependencies
echo "==> Installing package and resolving dependencies..."
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "/tmp/${DEB_FILE}"

# Cleanup
rm "/tmp/${DEB_FILE}"


echo "--------------------------------------------------------"
echo "Installation complete!"
echo "--------------------------------------------------------"
echo "Please open a NEW terminal or run:"
echo "  source /etc/profile.d/autoboat-vt.sh"
echo ""
echo "Then use: ros2 run autopilot_cpp motorboat_autopilot"
echo "--------------------------------------------------------"
