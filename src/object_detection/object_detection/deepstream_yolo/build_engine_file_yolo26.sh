#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo -e "Usage: $0 <name_of_yolo26_model_without_file_extension>"
    exit 1
fi

if [ -d "/home/ws" ]; then
    cd /home/ws/src/object_detection/object_detection/deepstream_yolo
    IS_DEV_CONTAINER=true
else
    cd /home/sailbot/autoboat_vt/src/object_detection/object_detection/deepstream_yolo
    IS_DEV_CONTAINER=false
fi

MODEL_NAME=$1
PT_FILE="${MODEL_NAME}.pt"
ONNX_FILE="${MODEL_NAME}.pt.onnx"
ENGINE_FILE="${MODEL_NAME}_model_b1_gpu0_fp16.engine"

if [ ! -d "pt_files" ]; then
    mkdir pt_files
fi

if [ ! -f "pt_files/${PT_FILE}" ]; then
    echo -e "File ${PT_FILE} not found in deepstream_yolo/pt_files/!"
    exit 1
fi

if [ ! -d "onnx_files" ]; then
    mkdir onnx_files
fi

if [ ! -d "engine_files" ]; then
    mkdir engine_files
fi

if [ ! -d "label_files" ]; then
    mkdir label_files
fi

if [ $IS_DEV_CONTAINER = true ]; then
    python3 export_yolo26_dev_container.py -w "pt_files/${PT_FILE}" --dynamic || exit 1
else
    python3 export_yolo26.py -w "pt_files/${PT_FILE}" --dynamic || exit 1 # This is untested on jetson
fi

mv "pt_files/${MODEL_NAME}.onnx" ./onnx_files/${ONNX_FILE}
mv "labels.txt" ./label_files/"${MODEL_NAME}_labels.txt"

python3 modify_config_file.py $MODEL_NAME 26 || exit 1

python3 make_model_engine.py 26 || exit 1