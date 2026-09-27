FROM ubuntu:16.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y lsb-release

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-get update \
    && apt-get install ros-kinetic-desktop-full ros-kinetic-control-* ros-kinetic-osg-markers ros-kinetic-move-base -y --allow-unauthenticated \
    && rosdep init \
    && rosdep update \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

RUN apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep python-wxtools python-lxml python-pathlib python-h5py python-scipy python-geolinks python-gdal -y --allow-unauthenticated \
    && apt-get install libfftw3-* libxml++2.6-* libsdl-image1.2-dev libsdl-dev -y --allow-unauthenticated

RUN source /opt/ros/kinetic/setup.bash \
    && mkdir -p ~/catkin_ws/src \
    && cd ~/catkin_ws/ \
    && catkin_make

# https://stackoverflow.com/a/46171246
RUN git config --global http.postBuffer 1048576000
RUN git config --global https.postBuffer 1048576000
RUN git config --global http.sslVerify false
RUN git config --global http.postBuffer 1048576000

RUN cd ~/catkin_ws/src \
    && git clone https://github.com/lucasmrdt/usv_sim_lsa \
    && cd usv_sim_lsa \
    && git checkout f2a49b3f \
    && git submodule init \
    && git submodule update

RUN cd ~/catkin_ws/src/usv_sim_lsa \
    && chmod +x ./install_usv_sim \
    && ./install_usv_sim

RUN cd ~/catkin_ws/ \
    && source /opt/ros/kinetic/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

RUN cd ~/catkin_ws/ \
    && source /opt/ros/kinetic/setup.bash \
    && catkin_make_isolated --install \
    && source install_isolated/setup.bash

RUN cd ~/catkin_ws/ \
    && source /opt/ros/kinetic/setup.bash \
    && source ~/catkin_ws/install_isolated/setup.bash \
    && roslaunch usv_sim sailboat_scenario3.launch parse:=true

RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py \
    && python get-pip.py \
    && rm get-pip.py \
    && python -m pip install pyzmq msgpack

RUN mkdir /root/.ssh

COPY id_rsa.pub /root/.ssh/authorized_keys

RUN chmod 700 /root/.ssh && chmod 600 /root/.ssh/authorized_keys

RUN apt-get install openssh-server -y

ARG max_step_size=0.001
ARG physics_type=ode

RUN cd ~/catkin_ws/install_isolated/share/usv_sim \
    && sed -i "s/MAX_STEP_SIZE/${max_step_size}/g" world/empty.world \
    && sed -i "s/PHYSICS_TYPE/${physics_type}/g" world/empty.world \
    && cp config/sailboat.${max_step_size}.yaml config/sailboat.yaml

WORKDIR /root/catkin_ws/

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && echo "service ssh start" >> ~/.bashrc \
    && echo "source ~/catkin_ws/install_isolated/setup.bash" >> ~/.bashrc

CMD source /opt/ros/kinetic/setup.bash \
    && source ~/catkin_ws/install_isolated/setup.bash \
    && service ssh start \
    && roslaunch usv_sim sailboat_scenario3.launch parse:=false
