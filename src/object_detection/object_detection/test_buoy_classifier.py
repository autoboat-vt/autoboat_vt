from ultralytics import YOLO
import os

TEST_NAME = "test13"
WEIGHTS_DIRECTORY = "weights/pretty_good_11l_train32.pt"
IMAGES_DIRECTORY = "dataset/test/images"
# IMAGES_DIRECTORY = "hard_images"


if not os.path.exists(f"test_results/{TEST_NAME}"):
    os.makedirs(f"test_results/{TEST_NAME}")
    
# os.environ['WANDB_MODE'] = 'disabled'
model = YOLO(WEIGHTS_DIRECTORY)

# model.set_classes(["human", "tree", "boat", "buoy ball", "sports ball", "ball", "dock"])

for index, image_name in enumerate(os.listdir(IMAGES_DIRECTORY)):
    if index > 1000: continue
    
    results = model.predict(
        source=f"{IMAGES_DIRECTORY}/{image_name}", device="cuda:0", conf=0.2,
    )
    
    

    for result in results:
        result.save(f"test_results/{TEST_NAME}/{index}.jpg")


