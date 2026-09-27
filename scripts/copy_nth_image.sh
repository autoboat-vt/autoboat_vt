#!/bin/bash

if [ ! -d "frame_logs" ]; then
    echo "Could not find frame_logs directory. Please run scp_copy_frame_data.sh first."
    exit 1
fi

if [ -d "frame_logs/copied_frames" ]; then
    echo -e "\e[33mWarning:\e[0m frame_logs/copied_frames directory already exists."
    echo -n "Overwrite? [y/n] "
    read response
    if [ "$response" != "y" ]; then
        exit 1
    fi
    rm -rf frame_logs/copied_frames
fi

# Select one of the runs that we find in frame_logs, and copy every nth image to frame_logs/copied_frames
echo "Select a run to copy frames from:"
select run in $(ls frame_logs); do
    if [ -n "$run" ]; then
        echo "Copying frames from $run"
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

mkdir -p frame_logs/copied_frames
n=0
interval=10 # copy every nth frame
for img in frame_logs/$run/frames/*.png; do
    if [ $((n % interval)) -eq 0 ]; then
        cp "$img" frame_logs/copied_frames/
        echo "Copied $img"
    fi
    n=$((n + 1))
done

echo "Finished copying frames to frame_logs/copied_frames"