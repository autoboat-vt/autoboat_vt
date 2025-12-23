# ---------------------------------------------------------------------------------------------------------
# These commands are run on the docker container right after the docker container starts
# ---------------------------------------------------------------------------------------------------------



# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
echo sudo chmod 777 /var/run/docker.sock >> "/home/autoboat_user/.bashrc"
echo sudo chmod -R 777 /home/ >> "/home/autoboat_user/.bashrc"
echo sudo chmod -R 777 /etc/udev/ >> "/home/autoboat_user/.bashrc"
sudo chmod -R 777 /etc/udev/


# Make sure that you can just type python and you don't have to type python3 because people will get confused
echo 'alias python="python3"' >> /home/autoboat_user/.bashrc



# Install all of the pip packages that we may have edited
# "pip install -e" installs the packages as "editable" which just means that we can make changes in the packages
# And you don't have to reinstall them, the changes will automatically be visible if you run another script again
pip install -e /home/ws/src/vesc/pyvesc/
pip install -e /home/ws/src/simulation/sailboat_gym/




# Build the ros2 workspace for the first time
# Building with symlink-install will allow us to edit python files without having to rebuild (which is super annoying!)
# 
source /opt/ros/humble/setup.bash
echo source /opt/ros/humble/setup.bash >> "/home/autoboat_user/.bashrc"
source /home/autoboat_user/.bashrc
colcon build --symlink-install

echo source /home/ws/install/setup.bash >> "/home/autoboat_user/.bashrc"




# Load all crontabs
# Crontabs allow us to run commands inside of the devcontainer at a fixed interval
# For instance, every 0.5 seconds we can ensure that you have the permission to every USB device
# If you would like to learn more about crontabs, you can do so here: https://www.geeksforgeeks.org/linux-unix/crontab-in-linux-with-examples/
crontab crontabs/chmod777job.txt
echo sudo service cron start >> ~/.bashrc




# Load udev rules for each device and remove any autoboat udev rules that existed before
# Udev rules basically just rename devices for us so that they are easier to access
# For example the raspberry pi pico udev rule would just make it so that the file descriptor for the raspberry pi pico device
# Would always be at the file: /dev/pico
# You can read more about udev rules here: https://opensource.com/article/18/11/udev
# if [ -f "/etc/udev/rules.d/99-autoboat-udev.rules" ]; then
#     rm -f /etc/udev/rules.d/99-autoboat-udev.rules
# fi

# sudo echo ACTION=="add", ATTRS{idVendor}=="2E8A", ATTRS{idProduct}=="0005", SYMLINK+="pico", MODE="0666" >> /etc/udev/rules.d/99-autoboat-udev.rules
# sudo echo ACTION=="add", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="gps", MODE="0666" >> /etc/udev/rules.d/99-autoboat-udev.rules
# # sudo echo ACTION=="add", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", SYMLINK+="camera", MODE="0666" >> /etc/udev/rules.d/99-autoboat-udev.rules
# sudo echo ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A9001WL3", SYMLINK+="rc", MODE="0666" >> /etc/udev/rules.d/99-autoboat-udev.rules
# sudo echo ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="ABSCDYAB", SYMLINK+="wind_sensor", MODE="0666" >> /etc/udev/rules.d/99-autoboat-udev.rules

# sudo udevadm control --reload-rules
# sudo udevadm trigger





# Move temporary files that were downloaded from each devcontainer variant
# The reason that we have this is to make it faster to download certain repositories and dataset for each devcontainer variant
# For instance, its far easier to just have the buoy/ boat computer vision dataset in the docker image and then just copy it
# into your workspace than for you to just download the huge dataset yourself. It just makes things slightly easier

# In some cases, we can store the entire built dependencies in /tmp like for microros where we can store a package
# of all of the dependencies that we need to work with microros

if [[ "$DEVCONTAINER_VARIANT" == "yolo" || "$DEVCONTAINER_VARIANT" == "deepstream_and_yolo" ]]; then

    # These devcontainers put the following data in the /tmp folder so all we need to do is move them so the user can see it 
    mv /tmp/autoboat_weights src/object_detection/object_detection/weights
    mv /tmp/autoboat_dataset src/object_detection/object_detection/dataset
    mv /tmp/autoboat_hard_images src/object_detection/object_detection/hard_images

    sudo chmod -R 777 src/object_detection/object_detection 
fi



if [[ "$DEVCONTAINER_VARIANT" == "microros" ]]; then
    rm -rf src/microros/dependencies
    mv /tmp/src/microros/dependencies src/microros

    sudo chmod -R 777 src/microros
fi


if [[ "$DEVCONTAINER_VARIANT" == "deepstream" || "$DEVCONTAINER_VARIANT" == "deepstream_and_yolo" ]]; then

    rm -rf src/object_detection/object_detection/deepstream_yolo
    sudo mv /tmp/deepstream_yolo src/object_detection/object_detection/deepstream_yolo

    sudo chmod -R 777 src/object_detection/object_detection

    rm -rf ~/.cache/gstreamer-1.0/
    gst-inspect-1.0 nvstreammux

fi

# TODO: DELETE THESE TEMPORARY FILES FROM THE DOCKER IMAGE IF THEY HAVE ALREADY BEEN INSTALLED




source ~/.bashrc
