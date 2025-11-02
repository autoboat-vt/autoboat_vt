#! /bin/bash

# Installs DeepStream-Yolo dependencies
# and DeepStream itself if not already installed.

# NOTE: THIS IS FOR DEEPSTREAM 7.1 ON JETPACK 6.2 ONLY. probably also work on jetpack 6.1 but untested.
# might work better on jetpack 6.1, but I don't know.
# Jetpack 6.2 might offer features that 6.1 doesn't have, and DeepStream 7.1 works with a fix.

# It might be necessary to run a gstreamer script once to initialize plugins

cd ~/
sudo -v # Prompt for sudo password at start
sudo apt update

sudo apt list | grep deepstream-7.1 -q
if [[ $? -ne 0 ]]; then
    echo -e "\n\nDeepStream not found, installing DeepStream...\n\n"
    pip3 install meson
    pip3 install ninja
    git clone https://github.com/GNOME/glib.git
    cd glib/
    git checkout 2.76.6
    meson build --prefix=/usr
    ninja -C build/
    cd build/
    sudo ninja install
    cd ~/
    sudo apt install -y \
        libssl3 \
        libssl-dev \
        libgstreamer1.0-0 \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        libgstreamer-plugins-base1.0-dev \
        libgstrtspserver-1.0-0 \
        libjansson4 \
        libyaml-cpp-dev
    sudo mkdir -p /opt/nvidia/deepstream/deepstream-7.1/
    cd /opt/nvidia/deepstream/deepstream-7.1/
    sudo wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/nvidia/deepstream/versions/7.1/files/deepstream-7.1_7.1.0-1_arm64.deb' -O deepstream-7.1_7.1.0-1_arm64.deb
    sudo apt-get install -y ./deepstream-7.1_7.1.0-1_arm64.deb
else
    echo -e "\n\nDeepStream already installed, skipping installation.\n\n"
fi

# Install DeepStream-Python
echo -e "\n\nInstalling DeepStream-Python\n\n"
cd ~/
sudo apt install -y python3-gi python3-dev python3-gst-1.0 \
    python-gi-dev git meson python3 python3-pip python3.10-dev \
    cmake g++ build-essential libglib2.0-dev libglib2.0-dev-bin \
    libgstreamer1.0-dev libtool m4 autoconf automake \
    libgirepository1.0-dev libcairo2-dev
wget https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/releases/download/v1.2.0/pyds-1.2.0-cp310-cp310-linux_aarch64.whl
pip3 install ./pyds-1.2.0-*.whl

# Compile the DeepStream-Yolo library
echo -e "\n\nCompiling DeepStream-Yolo library\n\n"
cd ~/autoboat_vt/src/object_detection/object_detection/DeepStream-Yolo/
export CUDA_VER=12.6
make -C nvdsinfer_custom_impl_Yolo clean && make -C nvdsinfer_custom_impl_Yolo

# Install ultralytics
echo -e "\n\nInstalling ultralytics\n\n"
cd ~/autoboat_vt/src/object_detection/object_detection/DeepStream-Yolo/
git clone https://github.com/ultralytics/ultralytics.git
# cd ~/autoboat_vt/src/object_detection/object_detection/DeepStream-Yolo/ultralytics/
cd ultralytics/
pip install -e .
pip install onnx onnxslim onnxruntime
pip install numpy==1.26.0
cp ../export_yolo11.py

echo -e "\n\nInstalling Utils\n\n"
sudo apt-get install -y v4l-utils