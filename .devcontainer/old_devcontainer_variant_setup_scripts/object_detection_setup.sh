sudo apt install git-lfs
sudo apt install unzip


# git-lfs clone https://huggingface.co/datasets/Aanimated/autoboat_vt_object_detection src/object_detection

mkdir src/object_detection/object_detection/weights
mkdir src/object_detection/object_detection/runs
mkdir src/object_detection/object_detection/hard_images
mkdir src/object_detection/object_detection/test_results
mkdir src/object_detection/object_detection/dataset




# Install yolo
pip install ultralytics sahi

# ultralytics upgrades our numpy version to 2.x.x but we don't actually want that because it will conflict with other packages, 
# specifically ros cv-bridge (https://github.com/ros-perception/vision_opencv/issues/535)
pip install numpy==1.26.4


# For a similar reason, we also need to uninstall python-opencv, since it is not compatible with ros's cv-bridge
pip uninstall opencv-python -y


# Download pre trained model weights from our huggingface repository
git-lfs clone https://huggingface.co/datasets/Aanimated/autoboat_vt_models src/object_detection/object_detection/weights
rm -rf src/object_detection/object_detection/weights/.git
rm src/object_detection/object_detection/weights/.gitattributes

# Download the hard test images. This is just a bunch of really difficult images to validate our model
git-lfs clone https://huggingface.co/datasets/Aanimated/autoboat_vt_hard_images src/object_detection/object_detection/hard_images
rm -rf src/object_detection/object_detection/hard_images/.git
rm src/object_detection/object_detection/hard_images/.gitattributes

# Download the training/ validation/ testing dataset for buoys and boats
cd src/object_detection/object_detection/dataset
curl -L "https://app.roboflow.com/ds/Fab44SEJ3L?key=1cCBk2tOqN" > dataset.zip
unzip dataset.zip 
rm dataset.zip


cd /home/ws