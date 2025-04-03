#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# Import image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import necessary ROS interface types:
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import os
import numpy as np

class BeaconSearch(Node):

    def __init__(self):
        super().__init__("beacon_search")
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )

        self.declare_parameter("target_colour", "yellow")
        target_colour = self.get_parameter("target_colour").get_parameter_value().string_value
        self.get_logger().info(f"TARGET BEACON: Searching for {target_colour}.")

        if target_colour == "yellow":
            self.lower_threshold = (22, 75, 100)
            self.upper_threshold = (32, 255, 255)
        elif target_colour == "red":
            self.lower_threshold = (0, 75, 100)
            self.upper_threshold = (12, 255, 255)
        elif target_colour == "green":
            self.lower_threshold = (54, 75, 100)
            self.upper_threshold = (90, 255, 255)
        else: # blue
            self.lower_threshold = (101, 75, 100)
            self.upper_threshold = (126, 255, 255)

        # percent of the screen that contains that colour, before a beacon is "detected"
        self.percent_colour_detected = 0.1

        # most pixels of that colour detected
        self.highestm00 = 0
    
    def camera_callback(self, img_data):
        cvbridge_interface = CvBridge() 
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            self.get_logger().warning(f"{e}")

        height, width, _ = cv_img.shape

        crop_width = width - 100
        crop_height = 200
        crop_z0 = int((width / 2) - (crop_width / 2))
        crop_y0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_y0:crop_y0+crop_height, crop_z0:crop_z0+crop_width]
        
        cropped_height, cropped_width, _ = cropped_img.shape
        total_pixels = cropped_width * cropped_height

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, self.lower_threshold, self.upper_threshold)
        m = cv2.moments(mask)
        # /255.0 in order to get number of total pixels detected
        m00 = m['m00']/255.0

        # beacon is detected, and theres at least a 10% increase in pixels detected
        if m00 > total_pixels*self.percent_colour_detected and m00 > self.highestm00*1.1:
            self.highestm00 = m00
            self.get_logger().info(f"Detected a beacon with {m00} pixels.")
            self.save_image(img = cv_img, img_name="target_beacon")
            
    def save_image(self, img, img_name): 
        self.get_logger().info(f"Saving the image...")

        current_dir = os.path.dirname(os.path.realpath(__file__))
        parent_dir = os.path.dirname(current_dir)
        image_path = f"{parent_dir}/snaps/{img_name}.jpg"

        cv2.imwrite(image_path, img) 

        self.get_logger().info(
            f"\nSaved an image to '{image_path}'\n"
            f"  - image dims: {img.shape[0]}x{img.shape[1]}px"
        ) 
            
def main(args = None):
    rclpy.init(args = args)
    node = BeaconSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
