sudo apt install git-lfs
sudo apt install unzip


# git-lfs clone https://huggingface.co/datasets/Aanimated/autoboat_vt_object_detection src/object_detection


# Download pre trained model weights
git-lfs clone https://huggingface.co/datasets/Aanimated/autoboat_vt_models src/object_detection/object_detection/weights

# Download the hard test images
git-lfs clone https://huggingface.co/datasets/Aanimated/autoboat_vt_hard_images src/object_detection/object_detection/hard_images

# Download the dataset
cd src/object_detection/object_detection/dataset
curl -L "https://app.roboflow.com/ds/Fab44SEJ3L?key=1cCBk2tOqN" > dataset.zip
unzip dataset.zip 
rm dataset.zip
cd /home/ws



pip install ultralytics sahi

# ultralytics upgrades our numpy version to 2.x.x but we don't actually want that because it will conflict with other packages, 
# specifically ros cv-bridge (https://github.com/ros-perception/vision_opencv/issues/535)
pip install numpy==1.26.4


# For a similar reason, we also need to uninstall python-opencv, since it is not compatible with ros's cv-bridge
pip uninstall opencv-python -y