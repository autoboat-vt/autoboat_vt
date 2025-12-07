# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
# All this does is give us the permission to edit certain files with using the sudo command
echo sudo chmod 777 /var/run/docker.sock >> "/home/autoboat_user/.bashrc"
echo sudo chmod -R 777 /home/ >> "/home/autoboat_user/.bashrc"
echo sudo chmod -R 777 /etc/udev/ >> "/home/autoboat_user/.bashrc"
sudo chmod -R 777 /etc/udev/


# Make sure that you can just type python and you don't have to type python3 because people will get confused
echo 'alias python="python3"' >> /home/autoboat_user/.bashrc

# Easier alias for a specific type of build
echo 'alias build="cd /home/ws && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"' >> /home/autoboat_user/.bashrc

# Install all of the pip packages
pip install -e /home/ws/src/vesc/pyvesc/
pip install -e /home/ws/src/simulation/sailboat_gym/


# Build the ros2 workspace for the first time
source /opt/ros/humble/setup.bash
echo source /opt/ros/humble/setup.bash >> "/home/autoboat_user/.bashrc"
source /home/autoboat_user/.bashrc
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo source /home/ws/install/setup.bash >> "/home/autoboat_user/.bashrc"


# Load all crontabs
crontab crontabs/chmod777job.txt
echo sudo service cron start >> ~/.bashrc


source ~/.bashrc
