#! /bin/bash

# Installs deepstream_yolo dependencies
# and DeepStream itself if not already installed.

# NOTE: This is for x86 platforms. Untested on aarch64.

# It might be necessary to run a gstreamer script once to initialize plugins

# https://docs.nvidia.com/metropolis/deepstream/7.1/text/DS_Installation.html
install_deepstream_x86() {
    echo -e "\nInstalling deepstream for x86_64"
    # Install prerequisite packages
    sudo apt install -y \
        libssl3 \
        libssl-dev \
        libgles2-mesa-dev \
        libgstreamer1.0-0 \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        libgstreamer-plugins-base1.0-dev \
        libgstrtspserver-1.0-0 \
        libjansson4 \
        libyaml-cpp-dev \
        libjsoncpp-dev \
        protobuf-compiler \
        gcc \
        make \
        git \
        python3
    # Install CUDA Toolkit 12.6
    sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub
    sudo add-apt-repository -y "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"
    sudo apt-get update
    sudo apt-get install -y cuda-toolkit-12-6
    
    # Install TensorRT 10.3.0.26
    version="10.3.0.26-1+cuda12.5"
    sudo apt-get install libnvinfer-dev=${version} libnvinfer-dispatch-dev=${version} libnvinfer-dispatch10=${version} libnvinfer-headers-dev=${version}
    libnvinfer-headers-plugin-dev=${version} libnvinfer-lean-dev=${version} libnvinfer-lean10=${version} libnvinfer-plugin-dev=${version} libnvinfer-plugin10=${version}
    libnvinfer-vc-plugin-dev=${version} libnvinfer-vc-plugin10=${version} libnvinfer10=${version} libnvonnxparsers-dev=${version} libnvonnxparsers10=${version} tensorrt-dev=${version}
    
    sudo wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/deepstream/7.1/files?redirect=true&path=deepstream-7.1_7.1.0-1_amd64.deb' --output-document 'deepstream-7.1_7.1.0-1_amd64.deb'
    sudo apt-get install -y ./deepstream-7.1_7.1.0-1_amd64.deb
}

install_deepstream_aarch64() {
    echo -e "\nDon't use arm\n"
    exit
}

cd ~/
sudo -v # Prompt for sudo password at start
sudo apt update

sudo apt list | grep deepstream-7.1 -q
if [[ $? -ne 0 ]]; then
    echo -e "\n\nDeepStream not found, installing DeepStream...\n\n"
    sudo apt install -y software-properties-common
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
    sudo mkdir -p /opt/nvidia/deepstream/deepstream-7.1/
    cd /opt/nvidia/deepstream/deepstream-7.1/
    if uname -m | grep x86_64 -q; then
        install_deepstream_x86
    else
        install_deepstream_aarch64
    fi
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
if uname -m | grep x86_64 -q; then
    echo -e "\nInstalling pyds for x86_64"
    sudo wget https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/releases/download/v1.2.0/pyds-1.2.0-cp310-cp310-linux_x86_64.whl
else
    echo -e "\nInstalling pyds for aarch64"
    sudo wget https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/releases/download/v1.2.0/pyds-1.2.0-cp310-cp310-linux_aarch64.whl
fi
pip3 install ./pyds-1.2.0-*.whl
# echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/targets/x86_64-linux/lib/stubs:$LD_LIBRARY_PATH' >> ~/.bashrc # fix from AI to fix pyds import
# source ~/.bashrc


# Install ultralytics
echo -e "\n\nInstalling ultralytics\n\n"
pip install ultralytics
pip install onnx onnxslim onnxruntime onnxscript
pip install sahi # sahi is currently not used
pip install numpy==1.26.4

echo -e "\n\nInstalling Utils\n\n"
sudo apt-get install -y v4l-utils


# sudo v4l2-ctl --list-devices # TODO: can we change permissions so this doesn't need to be run?

# export IS_DEV_CONTAINER=true # I don't think this persists after script ends otherwise
echo -e "\n\nDeepstream setup complete!\n\n"

cd /home/ws/