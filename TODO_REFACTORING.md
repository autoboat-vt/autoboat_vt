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




Quick announcement, but there has been another fairly major restructuring of the code on the main. We are doing this now because its probably the best time considering that there are probably less people actively working on the codebase right now because of finals. There are some names and things that are updated because of clarity that can be found below:


sensors and vesc folder have been merged into a "drivers" folder in ros_packages
src -> ros_packages (this only applies to the src folder at the root of the repository because it doesn't make sense that we have stuff like src/autopilot_cpp/src and the fact that we had multiple distinct programs in a single src folder)
microros -> firmware
the github actions folder has been moved around
development_image_microros -> development_image_firmware
development_image -> development_image_base
sailboat_simulation -> old_sailboat_simulation


all of these things should be on main and stable later today, so keep an eye out for that if you would like to work on code/ firmware over the summer.