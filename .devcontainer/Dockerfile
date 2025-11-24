# TODO and NOTE:
# this is the docker file used to build the dev container. 
# For right now, you should not modify this and if you need to add something then please ask chris. 
# In the future, I will make it so that you can modify this file and push your changes so that everyone can use it


# NOTE: The reason why we are doing rm -rf /var/lib/apt/lists/* after every apt install is because of caching reasons

FROM ros:humble

ARG USERNAME=autoboat_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV SHELL=/bin/bash
ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------
# Create user + sudo setup
# -------------------------------
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && apt-get update \
 && apt-get install -y --no-install-recommends sudo \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && rm -rf /var/lib/apt/lists/*



# -------------------------------
# Base system updates & required tools
# -------------------------------
RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y --no-install-recommends python3-pip cron usbutils udev nano git-lfs \
 && rm -rf /var/lib/apt/lists/*



# -------------------------------
# Very unsafe sudo permissions
# -------------------------------
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/bin/chmod" >> /etc/sudoers \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/service" >> /etc/sudoers



# -------------------------------
# Install RealSense Camera
# -------------------------------
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-get install curl -y \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && apt update || true \
 && apt install ros-humble-librealsense2* -y \
 && apt install ros-humble-realsense2-* -y \
 && rm /etc/apt/sources.list.d/ros-latest.list \
 && rm -rf /var/lib/apt/lists/*



# -------------------------------
# Install ROS/ Vision packages
# -------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-dev-tools python3-opencv ros-humble-vision-opencv \
 && rm -rf /var/lib/apt/lists/* \
 && rm /etc/apt/sources.list.d/ros-latest.list || true



# -------------------------------
# Golang setup
# -------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends software-properties-common \
 && add-apt-repository ppa:longsleep/golang-backports -y \
 && apt-get update \
 && apt-get install -y --no-install-recommends golang-go \
 && rm -rf /var/lib/apt/lists/*



# -------------------------------
# X11 / GUI libraries
# -------------------------------
RUN apt-get update \
 && apt-get install -y --fix-missing --no-install-recommends \
    libx11-xcb1 \
    libxcb1 \
    libxcb-util1 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-shm0 \
    libxcb-icccm4 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxcb-randr0 \
    libxcb-glx0 \
    libxkbcommon-x11-0 \
    libxrender1 \
    libasound2 \
 && rm -rf /var/lib/apt/lists/*




# -------------------------------
# Install Docker inside Docker
# -------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends docker docker.io \
 && rm -rf /var/lib/apt/lists/*


# -------------------------------
# Python requirements (as user)
# -------------------------------
USER $USERNAME
COPY required_pip_packages.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt


# C++ requirements?
# RUN sudo apt-get update \
#     && sudo apt install libcurl4-openssl-dev -y \
#     && sudo apt-get install nlohmann-json3-dev -y \
#     && sudo apt install geographiclib-tools -y \
#     && sudo apt install ros-humble-serial-driver \
#     && sudo apt install ros-humble-asio-cmake-module -y \
#     && sudo apt install libudev-dev -y
    # && sudo apt install libboost-system-dev libboost-thread-dev -y \
    # && && rm -rf /var/lib/apt/lists/*




CMD ["/bin/bash"]
