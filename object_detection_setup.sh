sudo apt install git-lfs

# TODO add commands for pulling the object detection code/ datasets from the huggingface repository


pip install ultralytics

# ultralytics upgrades our numpy version to 2.x.x but we don't actually want that because it will conflict with other packages, 
# specifically ros cv-bridge (https://github.com/ros-perception/vision_opencv/issues/535)
pip install numpy==1.26.4


# For a similar reason, we also need to uninstall python-opencv, since it is not compatible with ros's cv-bridge
pip uninstall python-opencv