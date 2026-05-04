#!/usr/bin/env bash

# ---------------------------------------------------------------------------------------------------------
# These commands are run on the docker container right after the docker container starts
# ---------------------------------------------------------------------------------------------------------

AUTOBOAT_USER_HOME="/home/autoboat_user"

# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
echo "sudo chmod 777 /var/run/docker.sock" >> $AUTOBOAT_USER_HOME/.bashrc
echo export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/ws/build/foil_dynamics/:/home/ws/build/sail_limits/:/home/ws/build/custom_lift_drag/:/home/ws/build/wind_arrow/ >> $AUTOBOAT_USER_HOME/.bashrc


# Make sure that you can just type python and you don't have to type python3 because people will get confused
echo 'alias python="python3"' >> $AUTOBOAT_USER_HOME/.bashrc

# Make it easy to perform the proper build command
echo 'alias build="cd /home/ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"' >> ~/.bashrc
echo 'alias build_python="build --packages-ignore-regex .*cpp"' >> ~/.bashrc

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

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-ignore-regex .*cpp

echo "source /home/ws/install/setup.bash" >> $AUTOBOAT_USER_HOME/.bashrc



# Miscellaneous things to finish setting up deepstream variant. Eventually, these should probably be added to the deepstream dockerfile.
if [[ "$DEVCONTAINER_VARIANT" == "deepstream" || "$DEVCONTAINER_VARIANT" == "deepstream_and_yolo" ]]; then
    rm -rf ~/.cache/gstreamer-1.0/
    gst-inspect-1.0 nvstreammux

fi


source $AUTOBOAT_USER_HOME/.bashrc
