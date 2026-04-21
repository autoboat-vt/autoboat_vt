Make sure that all of the code is fully working

Make sure that the vesc driver and all of the other sensor drivers are now fully benchmarked and tested so we know the exact ram/ cpu usages of each of the drivers


ADD TO DOCUMENTATION THAT IF THE DEVCONTAINER IS TAKING TOO MUCH RAM, TO ADD A SWAP FILE

UPDATE ALL OF THE DOCUMENTATION

UPDATE THE DOCUMENTATION ASSOCIATED WITH THE LAUNCH FILES



WHAT IS DONE:

REWROTE THE ENTIRE REPOSITORY TO CPP
NOW WE HAVE A PROPER CI/CD PIPELINE
NOW WE HAVE RELEASES AND RELEASE BINARIES
MICROROS INSTALLATION HAS BEEN OVERHAULED (NOW DEPENDENCIES HAS BEEN MOVED TO /opt)
setup_display_and_gpu HAS BEEN OVERHAULED WITH host_environment_variables
put launch files into their own ros2 package
moved the simulation sdf files to motorboat_simulation/simulation_models/models so that they can be more easily grabbed in the debian package



**ANNOUNCEMENT TO MAKE**

I just made a big PR to main with a ton of new features and changes:

- Refactored launch folder into its own ros2 package
- Rewrote most of the current autopilot drivers, autopilot, and telemetry nodes into c++. The goal of these isn't to immediately shift development into c++, we will likely still be using python for autopilot development for the forseeable future; however, eventually the autopilot and it's drivers will get into a stable enough version where it will make sense to transition development into c++. This is just meant to provide the scaffolding to make it easier to build the future c++ autopilot off of.
- Introduced a new system to easily manage devcontainer variants via the .devcontainer host_environment_variables file
- Renamed setup_display_and_gpu.sh to host_setup.sh and made the proper changes to support host_environment_variables
- Created a proper CI/CD pipeline to automatically build the devcontainer and push it to docker hub/ ghcr.io whenever something is committed to main and automatically create official releases with compiled binaries and a .deb package
- Microros installation has now been overhauled, now being easily installed with the devcontainer variants as seen here: https://autoboat-vt.github.io/documentation/ci_cd_autoboat_os_devcontainer/devcontainer/#how-to-change-the-devcontainer-variant-you-are-currently-using
- Moved the simulation sdf files to motorboat_simulation/simulation_models/models so that they can be more easily grabbed in the debian package


Please keep the following in mind when working on the following version on main:

- Whenever you merge with main or start working from main, please run `bash .devcontainer/host_setup.sh && source ~/.bashrc` in a WSL, linux, or macos terminal. Make sure you don't run this inside of a devcontainer, since that won't work. Then all you have to do is rebuild the devcontainer and you should be fully setup with everything
- If you would like to access the microros or deepstream devcontainer variants, then you should follow the steps described here: https://autoboat-vt.github.io/documentation/ci_cd_autoboat_os_devcontainer/devcontainer/#how-to-change-the-devcontainer-variant-you-are-currently-using
- Now, instead of running `ros2 launch /home/ws/src/launch/launch_filename.launch.py`, you should be able to just run `ros2 launch autoboat_launch launch_filename.launch.py`
- Instead of running `colcon build --symlink-install && source ~/.bashrc` whenever you create a new node or packages, now you can just run `build_python && source ~/.bashrc`
- If you would like to build and test all of the new c++ nodes, you can run `build` in your terminal and then run one of the "cpp" launch files