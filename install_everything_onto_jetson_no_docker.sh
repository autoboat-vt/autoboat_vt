# TODO: make it so that we only have 1 ros installation script in here because its weird that there are duplicate scripts for installing ros????

# install ros2 jazzy
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade

sudo apt install ros-jazzy-ros-base && sudo apt install ros-dev-tools





# install camera

sudo apt install apt-utils
sudo pip3 install -U jetson-stats
sudo apt install nano -y

echo "installing dependencies"
sudo apt-get install libssl-dev libusb-1.0-0-dev pkg-config -y
sudo apt-get install build-essential cmake cmake-curses-gui -y
sudo apt-get install libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev -y
export PYTHON_EXECUTABLE="/usr/bin/python"
export LIBREALSENSE_DIRECTORY="$PWD/librealsense"
export NVCC_PATH="/usr/local/cuda/bin/nvcc"
export USE_CUDA=true
echo "cloning librealsense and changing to v2.55.1"
git clone https://github.com/IntelRealSense/librealsense.git
cd $LIBREALSENSE_DIRECTORY
git checkout e196cef
export CUDACXX=$NVCC_PATH
export PATH=${PATH}:/usr/local/cuda/bin
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64
mkdir build && cd build
export PYTHON_EXECUTABLE="/usr/bin/python"
echo "building"
/usr/bin/cmake ../ -DPYTHON_EXECUTABLE=$PYTHON_EXECUTABLE -DBUILD_EXAMPLES=true -DFORCE_LIBUVC=ON -DBUILD_WITH_CUDA="$USE_CUDA" -DCMAKE_BUILD_TYPE=release -DBUILD_PYTHON_BINDINGS=bool:true
sudo make install -j6
cd $LIBREALSENSE_DIRECTORY
# Copy over the udev rules so that camera can be run from user space
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
export LD_LIBRARY_PATH=""



# ros install
echo "install ros"
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade
sudo apt install ros-jazzy-ros-base
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-realsense2-*


# fix .bashrc and add all of the ROS stuff to the bashrc 
# TODO


# setup crontab (not actually needed for the jetson if we have the correct udev rules)


# setup systemctl stuff



# setup udev rule stuff for the pico and other devices
