#!/bin/bash

# This script is to copy images from the Jetson to this current directory.

trap 'echo -e "\nInterrupted"; exit 1' INT

if [ -d "frame_logs" ]; then
    echo -e "\e[33mWarning:\e[0m frame_logs directory already exists."
    echo -n "Overwrite? [y/n] "
    read response
    if [ "$response" != "y" ]; then
        exit 1
    fi
    rm -rf frame_logs
fi

echo -n "Wi Fi or USB? [1/2] "
read connection_type
if [ "$connection_type" == "2" ]; then
    HOSTNAME="192.168.55.1"
else 
    HOSTNAME="autoboat"
fi


echo "Connecting to Jetson..."
scp -r autoboat@$HOSTNAME:~/autoboat_vt/frame_logs . \
  && echo -e "\nSuccessfully copied frame_logs from Jetson." \
  || echo "Could not connect to Jetson."; exit 2
