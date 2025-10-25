sudo apt install git-lfs

git-lfs clone https://huggingface.co/datasets/Aanimated/autoboat_vt_object_detection src/object_detection


pip install ultralytics sahi

# ultralytics upgrades our numpy version to 2.x.x but we don't actually want that because it will conflict with other packages, 
# specifically ros cv-bridge (https://github.com/ros-perception/vision_opencv/issues/535)
pip install numpy==1.26.4


# For a similar reason, we also need to uninstall python-opencv, since it is not compatible with ros's cv-bridge
pip uninstall opencv-python -y


# Make sure that we rebuild, so colcon sees the object detection node as an actual ros node
colcon build --symlink-install