# TODO and NOTE:
# this is the docker file used to build the dev container. 
# For right now, you should not modify this and if you need to add something then please ask chris. 
# In the future, I will make it so that you can modify this file and push your changes so that everyone can use it


FROM ros:humble
ARG USERNAME=autoboat_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL=/bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN sudo apt install ros-dev-tools -y
RUN sudo apt install python3-opencv -y

# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/bin/chmod" >> /etc/sudoers
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/service" >> /etc/sudoers

# The following is to set up ubuntu repositories to download ros packages
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# install camera ros drivers
RUN sudo apt install ros-humble-librealsense2* -y
RUN sudo apt install ros-humble-realsense2-* -y

RUN sudo rm /etc/apt/sources.list.d/ros-latest.list
RUN sudo apt update

#Used to automate tasks
RUN sudo apt-get install cron --fix-missing

# MISC necessary things
RUN sudo apt install usbutils --fix-missing
RUN sudo apt install ros-humble-vision-opencv python3-opencv -y --fix-missing --fix-broken
RUN sudo apt install udev -y --fix-missing
RUN sudo apt install nano -y --fix-missing


# Required installations for new groundstation
RUN sudo apt update && sudo apt install -y software-properties-common
RUN sudo add-apt-repository ppa:longsleep/golang-backports
RUN sudo apt update
RUN sudo apt install -y golang-go
RUN sudo apt-get install -y --fix-missing \
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


# installations to allow the user to pull docker images from inside of this docker image
RUN sudo apt install docker -y
RUN sudo apt install docker.io -y

# Including this here so that we don't have to manually install torch everytime we rebuild the container 
# with the docker_dev_environment_setup.sh since torch installations generally take a while
# RUN pip install torch 
RUN sudo apt install git-lfs -y


USER $USERNAME
COPY required_pip_packages.txt /tmp/requirements.txt 
RUN pip install -r /tmp/requirements.txt


# [Optional] Set the default user. Omit if you want to keep the default as root.
CMD ["/bin/bash"]
