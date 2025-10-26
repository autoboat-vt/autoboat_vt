from ultralytics import YOLO
import os

# os.environ['WANDB_MODE'] = 'disabled'


model = YOLO("weights/yolo11x.pt")

# I just changed the base model
print("TRAINING MODEL 1")
model.train(
    data='dataset/data.yaml', device="cuda:0", epochs=10, batch=7, imgsz=640, dropout=0.05,
    freeze=20, crop_fraction=0., warmup_epochs=1, 
    erasing=0.04, hsv_h=0.5, hsv_s=0.5, hsv_v=0.2, fliplr=0.5, degrees=45,
    translate=0.1, scale=0.5, shear=0.0, flipud=0.5, mosaic=0., close_mosaic=0,
    optimizer="adamw", lr0=0.02, lrf=0.0005
)
print("FINISHED TRAINING MODEL 1")


# model = YOLO("weights/yolo11l.pt")

# print("TRAINING MODEL 2")

# model.train(
#     data='dataset/data.yaml', device="cuda:0", epochs=15, batch=7, imgsz=640, dropout=0.05,
#     freeze=21, crop_fraction=0., warmup_epochs=1, 
#     erasing=0.04, hsv_h=0.75, hsv_s=0.5, hsv_v=0.2, fliplr=0.75, degrees=45,
#     translate=0.1, scale=0.5, shear=0.0, flipud=0.5, mosaic=0., close_mosaic=0,
#     optimizer="adamw", lr0=0.03, lrf=0.0005
# )
# print("FINISHED TRAINING MODEL 2")


# model = YOLO("weights/yolo11l.pt")

# print("TRAINING MODEL 3")

# model.train(
#     data='dataset/data.yaml', device="cuda:0", epochs=15, batch=7, imgsz=640, dropout=0.05,
#     freeze=21, crop_fraction=0., warmup_epochs=1, 
#     erasing=0.04, hsv_h=0.75, hsv_s=0.5, hsv_v=0.2, fliplr=0.75, degrees=45,
#     translate=0.1, scale=0.5, shear=0.0, flipud=0.5, mosaic=0., close_mosaic=0,
#     optimizer="adamw", lr0=0.015, lrf=0.0005
# )

# print("FINISHED TRAINING MODEL 3")
