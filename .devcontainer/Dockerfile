# TODO and NOTE:
# this is the docker file used to build the dev container. 
# For right now, you should not modify this and if you need to add something then please ask chris. 
# In the future, I will make it so that you can modify this file and push your changes so that everyone can use it


# NOTE: The reason why we are doing rm -rf /var/lib/apt/lists/* after every apt install is because of caching reasons


FROM ros:humble
LABEL org.opencontainers.image.source="https://github.com/autoboat-vt/autoboat_vt"

USER root


ARG USERNAME=autoboat_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID


# stores whether or not we are on arm or x86
ARG TARGETARCH


ENV SHELL=/bin/bash
ENV DEBIAN_FRONTEND=noninteractive



RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    python3-pip \
    cron \
    usbutils \
    udev \
    nano \
    git-lfs \
    lsof \
    curl \
    lsb-release \
    gnupg \
    python3-dev \
    build-essential \
    ros-dev-tools \
    python3-opencv \
    ros-humble-vision-opencv \
    software-properties-common \
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
    docker.io \
    ccache \
    mold \
    libcurl4-openssl-dev \
    nlohmann-json3-dev \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    libudev-dev \
    libboost-system-dev \
    libboost-thread-dev \
    ninja-build \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/bin/chmod" >> /etc/sudoers \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/service" >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# Install RealSense / OSRF Gazebo / Golang (Separate due to repos)
# -------------------------------
RUN add-apt-repository ppa:longsleep/golang-backports -y \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && curl -s https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    golang-go \
    gz-harmonic ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*



# -------------------------------
# Ensure that qt5 is installed properly so pyqt installation works
# -------------------------------
RUN if [ "$TARGETARCH" = "arm64" ]; then \
      echo "Building for ARM64. Installing pre-compiled system packages via apt."; \
      apt-get update && apt-get install -y python3-pyqt5 python3-pyqt5.qtwebengine && rm -rf /var/lib/apt/lists/*; \
   fi

USER $USERNAME

RUN if [ "$TARGETARCH" = "amd64" ]; then \
      echo "Building for x86_64. Safe to download PyQt binaries via pip."; \
      pip install PyQt5==5.15.11 PyQtWebEngine==5.15.7; \
   fi





USER root
RUN cd /tmp \
   && git clone --depth 1 https://github.com/libcpr/cpr.git \
   && cd cpr \
   && mkdir build && cd build \
   && cmake .. -DCPR_USE_SYSTEM_CURL=ON \
   && cmake --build . --parallel \
   && cmake --install . \
   && cd / && rm -rf /tmp/cpr

USER $USERNAME
   


# -------------------------------
# Python requirements (as user)
# -------------------------------

COPY .devcontainer/required_pip_packages.txt /tmp/requirements.txt
RUN pip install --upgrade pip
RUN pip install packaging
RUN pip install -r /tmp/requirements.txt


   
CMD ["/bin/bash"]
# TODO and NOTE:
# this is the docker file used to build the dev container. 
# For right now, you should not modify this and if you need to add something then please ask chris. 
# In the future, I will make it so that you can modify this file and push your changes so that everyone can use it
