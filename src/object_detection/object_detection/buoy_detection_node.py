#!usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
# from std_msgs.msg import Int32
# from autoboat_msgs.msg import ObjectDetectionResultsList

from cv_bridge import CvBridge

from ultralytics import YOLO
from sahi.predict import get_prediction
from sahi import AutoDetectionModel


import cv2
import numpy as np
from math import sqrt


import os

current_directory_path = os.path.dirname(os.path.realpath(__file__))

TRAINED_IMAGE_SIZE = (640, 640)  # pixel width and height of the images that the model was trained on
IMAGE_CONFIDENCE = 0.65

SHOULD_SAVE_IMAGES = False


# SAHI IS NOT YET SUPPORTED DONT CHANGE THIS UNLESS YOU KNOW WHAT YOU ARE DOING+
USING_SAHI = False


class BuoyDetectionNode(Node):
    def __init__(self):
        super().__init__("buoy_detection")

        # SAHI vs Normal YOLO

        if USING_SAHI == False:
            self.model = YOLO(f"{current_directory_path}/weights/pretty_good_11l_train32.pt")

        if USING_SAHI == True:
            self.model = AutoDetectionModel.from_pretrained(
                model_type="ultralytics",
                model_path=f"{current_directory_path}/weights/pretty_good_11l_train32.pt",
                confidence_threshold=IMAGE_CONFIDENCE,
                device="cuda:0",
            )

        self.cv_bridge = CvBridge()

        self.current_image_rgb = None
        self.depth_image = None

        self.image_to_save_index = 0  # images are saved in the format name[index].jpg so this just keeps track of the current index of the image so that we don't overwrite other images
        self.has_inferenced_current_image = False

        sensor_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.buoy_angle_pub = self.create_publisher(Float32, "/buoy_angle", 10)
        self.buoy_depth_pixel_pub = self.create_publisher(Float32, "/buoy_depth_pixel", 10)
        self.test_publisher = self.create_publisher(Float32, "/test", 10)

        # self.depth_image_listener = self.create_subscription(
        #     msg_type=Image,
        #     topic="/camera/camera/aligned_depth_to_color/image_raw",
        #     callback=self.depth_image_callback,
        #     qos_profile=sensor_qos_profile,
        # )
        # self.depth_image_listener = self.create_subscription(msg_type=Image, topic="/camera/depth/image_rect_raw", callback=self.depth_image_callback, qos_profile=sensor_qos_profile)
        self.rgb_image_listener = self.create_subscription(
            msg_type=Image, topic="/camera/camera/color/image_raw", callback=self.rgb_image_callback, qos_profile=sensor_qos_profile
        )
        # self.object_detection_results_publisher = self.create_publisher(msg_type=ObjectDetectionResults, topic="/object_detection_results", qos_profile=sensor_qos_profile)

        self.create_timer(timer_period_sec=0.001, callback=self.perform_inference)

    def depth_image_callback(self, depth_image: Image):
        self.get_logger().info("got here depth")  # print(f"hihihihi")

        # top = 0
        # bottom = smaller_size

        # TODO downscale the image so that the smallest dimension is 640p
        # TODO: crop the image properly
        # cropped_depth_image_cv = depth_image_cv[left:right, top:bottom]
        # cropped_rgb_image_cv = rgb_image_cv

        # depth_image_cv.resize(TRAINED_IMAGE_SIZE)f"The type of the cv image is
        # rgb_image_cv.resize(TRAINED_IMAGE_SIZE)

        depth_image_cv = self.cv_bridge.imgmsg_to_cv2(depth_image, desired_encoding=depth_image.encoding)

        print(f"cropped image shape: {depth_image_cv.shape}")
        # cv2.imwrite('depth_image.jpg', depth_image_cv)
        print(f"The type of the cv image is {type(depth_image_cv)}")
        self.depth_image = depth_image_cv
        # np.where(depth_image_cv > 0)

        # print(depth_image_cv)

    def rgb_image_callback(self, rgb_image: Image):
        self.get_logger().info("got here rgb")

        self.current_image_rgb = self.cv_bridge.imgmsg_to_cv2(rgb_image, "bgr8")

        self.current_image_rgb = self.current_image_rgb[80:1200, 40:680]  # crop the image to 640,640

        self.has_inferenced_current_image = False

    def perform_inference(self):
        # https://docs.ultralytics.com/modes/predict/#inference-sources
        if self.current_image_rgb is None:
            return

        if self.has_inferenced_current_image:
            return

        if USING_SAHI == False:
            results = self.model.predict(
                [
                    self.current_image_rgb,
                ],
                conf=IMAGE_CONFIDENCE,
            )  # return a list of Results objects

        if USING_SAHI == True:
            results = get_prediction(image=self.current_image_rgb, detection_model=self.model)

        self.test_publisher.publish(Float32(data=0.0))

        # Added variable for real-time inference
        DIAGONAL_FIELD_OF_VIEW = 89

        # Process results list
        print(f"The length of the results object is {len(results)}")
        result = results[0]
        boxes = result.boxes  # Boxes object for bounding box outputs
        # print(f"boxes: {boxes}")

        # masks = result.masks  # Masks object for segmentation masks outputs
        # keypoints = result.keypoints  # Keypoints object for pose outputs
        # probs = result.probs  # Probs object for classification outputs
        # obb = result.obb  # Oriented boxes object for OBB outputs

        height = result.orig_shape[0]
        width = result.orig_shape[1]
        diagonal = sqrt(height**2 + width**2)
        deg_per_pixel = DIAGONAL_FIELD_OF_VIEW / diagonal
        boxes = result.boxes

        # conf_angle = {}
        angle_list = []
        conf_list = []
        x_list = []
        y_list = []
        box_y_center = 0
        box_x_center = 0

        if boxes.shape[0] == 0:
            return

        self.get_logger().info("We are finally getting something")
        for box in boxes:
            print(box.conf.item())
            box_location = box.xywh
            # The Y stuff is only for trying to get depth image values

            # box_centerx_location = box_location[0][0].item() + box_location[0][2].item()/2
            # box_centery_location = box_location[0][1].item() + box_location[0][3].item()/2

            box_centerx_location = box_location[0][0].item()
            box_centery_location = box_location[0][1].item()

            print(f"X-coordinate: {box_centerx_location}")
            print(f"Y-coordinate: {box_centery_location}")

            # print(f"non-absolute-value-x-location: {(width/2)-box_centerx_location}")
            # print(f"non-absolute-value-y-location: {(height/2)-box_centery_location}")

            x_distance_from_center = box_centerx_location - (width / 2)
            img_angle_from_center = x_distance_from_center * deg_per_pixel

            # conf_angle[box.conf.item()]=img_angle_from_center
            angle_list.append(img_angle_from_center)

            conf_list.append(box.conf.item())
            x_list.append(box_centerx_location)
            y_list.append(box_centery_location)
        # Trying to get the maximum confidence keys for the x and y locations for the conf box dictionaries -- FAILED, kept just in case
        # Skip to after the returning of the angle to see this used

        # max_conf_x_key = min(conf_x_box, key=conf_x_box.get)
        # max_conf_y_key = min(conf_y_box, key=conf_y_box.get)
        # print(f"The keys of the x dictionary are {conf_x_box.keys()}")
        # print(f"The keys of the y dictionary are {conf_y_box.keys()}")
        # print(f"max_conf_x_key: {max_conf_x_key}")
        # print(f"max_conf_y_key: {max_conf_y_key}")

        max_conf_index = np.argmax(conf_list)

        # y_avg_distance_from_center = sum_y/count

        # print(f"The confidence angle pairs sorted are {dict(sorted(conf_angle.items(), reverse=True))}")
        # max_angle_key = max(conf_angle, key=conf_angle.get)

        # print(f"The angle of the maximum confidence box is {conf_angle[max_angle_key]}")
        max_conf_angle = angle_list[max_conf_index]
        print(f"The most confident buoy angle is: {max_conf_angle}")
        msg = Float32()
        msg.data = max_conf_angle
        self.buoy_angle_pub.publish(msg)

        # Need to get the x,y location of the buoy/center of the bounding box
        box_x_center = x_list[max_conf_index]
        box_y_center = y_list[max_conf_index]
        print(f"The type of the current image is {type(self.current_image_rgb)}")
        print(f"The value at the index is {self.current_image_rgb[int(box_y_center), int(box_x_center)]}")
        # self.current_image_rgb[int(box_x_center), int(box_y_center)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center+1), int(box_y_center)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center+2), int(box_y_center)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center-1), int(box_y_center)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center-2), int(box_y_center)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center), int(box_y_center-1)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center), int(box_y_center-2)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center), int(box_y_center+1)]= (160, 32, 240)
        # self.current_image_rgb[int(box_x_center), int(box_y_center+2)]= (160, 32, 240)

        # Then need to index into the depth image class variable saved as self.depth_image with the x,y coordinates
        # And then print out the pixel value

        ##TODO: make sure that y-center and x-center are not out of bounds (return nothing if out of bounds)
        if not self.depth_image is None:
            print(type(self.depth_image))
            print(f"The boxcenterx variable is {int(box_x_center)}")
            print(f"The boxcentery variable is {int(box_y_center)}")

            print(f"The pixel value at the box location is hopefully {self.depth_image[int(box_y_center), int(box_x_center)]}")
            # print(type(self.depth_image[int(box_x_center)][int(box_y_center)]))
            msg = Float32()
            msg.data = (self.depth_image[int(box_y_center + 50), int(box_x_center + 30)].item()) / 1000
            # msg.data = int(box_x_center)
            self.buoy_depth_pixel_pub.publish(msg)

        # generate purple boxes around the center of the detected buoy
        for x_value in range(self.current_image_rgb.shape[1]):
            for y_value in range(self.current_image_rgb.shape[0]):
                difference_vector = [abs(y_value - int(box_y_center)), abs(x_value - int(box_x_center))]
                max_value = max(difference_vector[0], difference_vector[1])
                if max_value <= 20:
                    self.current_image_rgb[y_value, x_value] = [160, 32, 240]

        # cv2.imwrite("rgb_image.jpg", self.current_image_rgb)
        # result.save("rgb_image.png")

        # For Depth Image
        # for x_value in range(self.depth_image.shape[1]):
        #     for y_value in range(self.depth_image.shape[0]):
        #         difference_vector = [abs(y_value - int(box_y_center + 50)), abs(x_value - int(box_x_center + 30))]
        #         max_value = max(difference_vector[0], difference_vector[1])
        #         if max_value <= 20:
        #             self.depth_image[y_value, x_value] = 0

        # cv2.imwrite("depth_image.jpg", self.depth_image)

        if SHOULD_SAVE_IMAGES:
            print("GOT HERE")
            result.save(f"cv_results2/result_{self.image_to_save_index}.png")  # display to screen
            self.image_to_save_index += 1

        # TODO process these results properly
        # self.object_detection_results_publisher.publish()


def main():
    rclpy.init()
    buoy_detection_node = BuoyDetectionNode()
    rclpy.spin(buoy_detection_node)


if __name__ == "__main__":
    main()
