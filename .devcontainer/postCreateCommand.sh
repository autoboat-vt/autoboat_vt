#!/usr/bin/env bash

# ---------------------------------------------------------------------------------------------------------
# These commands are run on the docker container right after the docker container starts
# ---------------------------------------------------------------------------------------------------------

AUTOBOAT_USER_HOME="/home/autoboat_user"

# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
echo "sudo chmod 777 /var/run/docker.sock" >> $AUTOBOAT_USER_HOME/.bashrc
echo export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/ws/build/foil_dynamics/:/home/ws/build/sail_limits/:/home/ws/build/rudder_dynamics/:/home/ws/build/wind_arrow/ >> "/home/autoboat_user/.bashrc"
# Make sure that you can just type python and you don't have to type python3 because people will get confused
echo 'alias python="python3"' >> $AUTOBOAT_USER_HOME/.bashrc
# Install all of the pip packages that we may have edited
# "pip install -e" installs the packages as "editable" which just means that we can make changes in the packages
# And you don't have to reinstall them, the changes will automatically be visible if you run another script again
pip install -e /home/ws/src/vesc/pyvesc/
pip install -e /home/ws/src/sailboat_simulation/sailboat_gym/
pip install -r /home/ws/.devcontainer/required_pip_packages.txt



# Build the ros2 workspace for the first time
# Building with symlink-install will allow us to edit python files without having to rebuild (which is super annoying!) 
source /opt/ros/humble/setup.bash

echo "source /opt/ros/humble/setup.bash" >> $AUTOBOAT_USER_HOME/.bashrc
source $AUTOBOAT_USER_HOME/.bashrc

colcon build --symlink-install

echo "source /home/ws/install/setup.bash" >> $AUTOBOAT_USER_HOME/.bashrc



# Move temporary files that were downloaded from each devcontainer variant
# The reason that we have this is to make it faster to download certain repositories and dataset for each devcontainer variant
# For instance, its far easier to just have the buoy/ boat computer vision dataset in the docker image and then just copy it
# into your workspace than for you to just download the huge dataset yourself. It just makes things slightly easier

# In some cases, we can store the entire built dependencies in /tmp like for microros where we can store a package
# of all of the dependencies that we need to work with microros

# if [[ "$DEVCONTAINER_VARIANT" == "yolo" || "$DEVCONTAINER_VARIANT" == "deepstream_and_yolo" ]]; then

#     # These devcontainers put the following data in the /tmp folder so all we need to do is move them so the user can see it 
#     mv /tmp/autoboat_weights src/object_detection/object_detection/weights
#     mv /tmp/autoboat_dataset src/object_detection/object_detection/dataset
#     mv /tmp/autoboat_hard_images src/object_detection/object_detection/hard_images

#     sudo chmod -R 777 src/object_detection/object_detection 
# fi



if [[ "$DEVCONTAINER_VARIANT" == "microros" ]]; then
    rm -rf src/microros/dependencies
    mv /tmp/src/microros/dependencies src/microros

    sudo chmod -R 777 src/microros
fi


if [[ "$DEVCONTAINER_VARIANT" == "deepstream" || "$DEVCONTAINER_VARIANT" == "deepstream_and_yolo" ]]; then
    rm -rf ~/.cache/gstreamer-1.0/
    gst-inspect-1.0 nvstreammux

fi

# TODO: DELETE THESE TEMPORARY FILES FROM THE DOCKER IMAGE IF THEY HAVE ALREADY BEEN INSTALLED


source $AUTOBOAT_USER_HOME/.bashrc
