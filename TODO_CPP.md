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