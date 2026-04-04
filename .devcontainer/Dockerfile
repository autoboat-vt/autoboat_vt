# TODO and NOTE:
# this is the docker file used to build the dev container. 
# For right now, you should not modify this and if you need to add something then please ask chris. 
# In the future, I will make it so that you can modify this file and push your changes so that everyone can use it


# NOTE: The reason why we are doing rm -rf /var/lib/apt/lists/* after every apt install is because of caching reasons


FROM ros:humble

USER root


ARG USERNAME=autoboat_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID


# stores whether or not we are on arm or x86
ARG TARGETARCH


ENV SHELL=/bin/bash
ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------
# Create user + sudo setup
# -------------------------------
RUN apt update \
   && groupadd --gid $USER_GID $USERNAME \
   && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
   && apt install -y --no-install-recommends sudo \
   && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
   && chmod 0440 /etc/sudoers.d/$USERNAME

# -------------------------------
# Base system updates & required tools
# ------------------------------- 
RUN apt install -y --no-install-recommends python3-pip cron usbutils udev nano git-lfs lsof

# -------------------------------
# Very unsafe sudo permissions
# -------------------------------
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/bin/chmod" >> /etc/sudoers \
   && echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/service" >> /etc/sudoers

# -------------------------------
# Install RealSense Camera
# -------------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
   && apt-get install curl -y \
   && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
   && apt update || true \
   && apt install ros-humble-librealsense2* -y \
   && apt install ros-humble-realsense2-* -y \
   && rm /etc/apt/sources.list.d/ros-latest.list


# -------------------------------
# Install ROS/ Vision packages
# -------------------------------
RUN apt update \
   && apt-get install -y --no-install-recommends ros-dev-tools python3-opencv ros-humble-vision-opencv \
   && rm /etc/apt/sources.list.d/ros-latest.list || true


# -------------------------------
# Golang setup
# -------------------------------
RUN apt install -y --no-install-recommends software-properties-common \
   && add-apt-repository ppa:longsleep/golang-backports -y \
   && apt update \
   && apt install -y --no-install-recommends golang-go


# -------------------------------
# X11 / GUI libraries
# -------------------------------
RUN apt install -y --fix-missing --no-install-recommends \
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
   libasound2

# -------------------------------
# Install Docker inside Docker
# -------------------------------
RUN apt install -y --no-install-recommends docker docker.io



# -------------------------------
# Ensure that qt5 is installed properly so pyqt installation works
# -------------------------------
RUN if [ "$TARGETARCH" = "arm64" ]; then \
      echo "Building for ARM64. Installing pre-compiled system packages via apt."; \
      apt-get update && apt-get install -y \
      python3-pyqt5 \
      python3-pyqt5.qtwebengine; \
   fi

USER $USERNAME

RUN if [ "$TARGETARCH" = "amd64" ]; then \
      echo "Building for x86_64. Safe to download PyQt binaries via pip."; \
      pip install PyQt5==5.15.11 PyQtWebEngine==5.15.7; \
   fi




# -------------------------------
# Python requirements (as user)
# -------------------------------

COPY .devcontainer/required_pip_packages.txt /tmp/requirements.txt
RUN pip install --upgrade pip
RUN pip install packaging
RUN pip install -r /tmp/requirements.txt

# -------------------------------
# Installation for gazebo (for simulation)
# -------------------------------
USER root
RUN if [ "$TARGETARCH" = "amd64" ]; then \
      echo "Building for x86_64. Safe to download gazebo."; \
      apt update \
      && apt install curl lsb-release gnupg \
      && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
      && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
      && apt update \
      && apt install gz-harmonic -y \
      && apt install ros-humble-ros-gzharmonic -y; \
   fi
USER $USERNAME


# Install c++ curl, json, serial driver, asyncio, udev, libboost, ninja build system, ccache, and mold 
RUN sudo apt-get update \
   && sudo apt install libcurl4-openssl-dev -y \
   && sudo apt-get install nlohmann-json3-dev -y \
   && sudo apt install ros-humble-serial-driver -y \
   && sudo apt install ros-humble-asio-cmake-module -y \
   && sudo apt install libudev-dev -y \
   && sudo apt install libboost-system-dev libboost-thread-dev -y \
   && sudo apt install ninja-build \
   && sudo apt-get install -y ccache mold \
   && sudo rm -rf /var/lib/apt/lists/*


RUN cd ~ \
   && git clone https://github.com/libcpr/cpr.git \
   && cd cpr && mkdir build && cd build \
   && cmake .. -DCPR_USE_SYSTEM_CURL=ON \
   && cmake --build . --parallel \
   && sudo cmake --install . 
   
   
CMD ["/bin/bash"]
# TODO and NOTE:
# this is the docker file used to build the dev container. 
# For right now, you should not modify this and if you need to add something then please ask chris. 
# In the future, I will make it so that you can modify this file and push your changes so that everyone can use it
