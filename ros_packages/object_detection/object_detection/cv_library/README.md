# Computer Vision Library

This folder contains the following files:

- deepstream_engine.py
- export_yolo11_dev_container.py
- export_yolo11.py
- export_yolo26_dev_container.py
- make_model_engine.py
- modify_config_file.py
- triangulation.py

## deepstream_engine.py

This file runs the deepstream and computer vision pipeline. It is called by the object_detection node.

## triangulation.py

This file triangulates the positions of objects. It is called by the localization node.

## Model Generation

The following files are used in generation a model .engine file:

- export_yolo11_dev_container.py
- export_yolo11.py
- export_yolo26_dev_container.py
- make_model_engine.py
- modify_config_file.py

All of these scripts are called by `build_engine_file_yolo.sh` in `scripts/`. The model generation process is .pt -> .onnx -> .engine.

The export files are used to convert a `.pt` model file to `.onnx`. The files with `dev_container` in the name are ones that have only been tested in the dev container, but might still work on the jetson.

`modify_config_file.py` is used to modify the yolo config file to point to the newly created model .engine file.

`make_model_engine.py` is used to create the model .engine file. It is a miniature deepstream pipeline that creates the engine file.
