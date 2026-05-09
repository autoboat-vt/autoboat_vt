#!/bin/bash

if [ "$#" -lt 2 ]; then
    echo -e "Usage: $0 <yolo_version> <name_of_yolo26_model_without_file_extension>"
    exit 1
fi

YOLO_VER=$1
MODEL_NAME=$2

if [ -d "/home/ws" ]; then
    cd /home/ws/src/object_detection/object_detection/config
    PATH_TO_AB_DIR="/home/ws"
    IS_DEV_CONTAINER=true
else
    cd ~/autoboat_vt/src/object_detection/object_detection/config
    PATH_TO_AB_DIR="~/autoboat_vt"
    IS_DEV_CONTAINER=false
fi

if [ ! -f "yolo11_config.yaml" ]; then
    cp yolo11_config_template.yaml yolo11_config.yaml
fi

if [ ! -f "yolo26_config.yaml" ]; then
    cp yolo26_config_template.yaml yolo26_config.yaml
fi

PT_FILE="${MODEL_NAME}.pt"
ONNX_FILE="${MODEL_NAME}.onnx"
ENGINE_FILE="${MODEL_NAME}_model_b1_gpu0_fp16.engine"

mkdir -p yolo11/pt_files
mkdir -p yolo26/pt_files

cd $PATH_TO_AB_DIR/scripts

if [ -f "${PT_FILE}" ]; then
    mv "${PT_FILE}" ../src/object_detection/object_detection/config/yolo${YOLO_VER}/pt_files/
fi

cd ../src/object_detection/object_detection/config

if [ ! -f "yolo${YOLO_VER}/pt_files/${PT_FILE}" ]; then
    echo -e "File ${PT_FILE} not found!"
    exit 1
fi

mkdir -p yolo11/onnx_files
mkdir -p yolo26/onnx_files

mkdir -p yolo11/engine_files
mkdir -p yolo26/engine_files

mkdir -p yolo11/label_files
mkdir -p yolo26/label_files

cd ../cv_library

# The export script is only tested in the dev container, but it should work on jetson as well. If it doesn't, we can add a separate export script for jetson.
# if [ $IS_DEV_CONTAINER = true ]; then
#     python3 export_yolo${YOLO_VER}_dev_container.py -w "pt_files/${PT_FILE}" --dynamic || exit 1
# else
#     python3 export_yolo${YOLO_VER}.py -w "pt_files/${PT_FILE}" --dynamic || exit 1 # This is untested on jetson
# fi

python3 export_yolo${YOLO_VER}_dev_container.py -w "../config/yolo${YOLO_VER}/pt_files/${PT_FILE}" --dynamic || exit 1

mv "../config/yolo${YOLO_VER}/pt_files/${ONNX_FILE}" ../config/yolo${YOLO_VER}/onnx_files/${ONNX_FILE}
mv "labels.txt" ../config/yolo${YOLO_VER}/label_files/"${MODEL_NAME}_labels.txt"

python3 modify_config_file.py $MODEL_NAME $YOLO_VER || exit 1

python3 make_model_engine.py $YOLO_VER || exit 1