#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo -e "Usage: $0 <name_of_pt_file>"
    exit 1
fi

if [ -d "/home/ws" ]; then
    cd /home/ws/src/object_detection/object_detection/deepstream_yolo
    IS_DEV_CONTAINER=true
else
    cd /home/sailbot/autoboat_vt/src/object_detection/object_detection/deepstream_yolo
    IS_DEV_CONTAINER=false
fi

if [ ! -f "$1" ]; then
    echo -e "File $1 not found!"
    exit 1
fi

PT_FILE=$1

if [ "$IS_DEV_CONTAINER" = true ]; then
    python3 export_yolo11_dev_container.py -w $PT_FILE || exit 1
else
    python3 export_yolo11.py -w $PT_FILE || exit 1
fi

python3 modify_config_file.py $PT_FILE.onnx || exit 1

python3 make_model_engine.py || exit 1