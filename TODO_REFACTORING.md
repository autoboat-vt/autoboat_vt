Finished:

moved src to ros_packages


Still need to do:

TODO test that the second part of the postinst scripts actually work
Update all of the documentation
Rename sailboat_simulation package to deprecated_sailboat_simulation
Update output package name to autoboatvt-firmware-dependencies instead of autoboatvt-firmware-dependencies-full


Rename microros to firmware
Create drivers and autopilot and autoboat_simulation
Split all of the driver types into their own folders
Refactor all instances of rc to radio controller




ALL THINGS CHANGED THAT PEOPLE SHOULD BE NOTIFIED OF:

SENSORS and DRIVERS FOLDERS HAVE BEEN MERGED
SRC -> ROS_PACKAGES
MICROROS -> FIRMWARE
GITHUB ACTIONS HAVE BEEN MOVED AROUND
development_image_microros -> development_image_firmware
development_image -> development_image_base
sailboat_simulation -> old_sailboat_simulation