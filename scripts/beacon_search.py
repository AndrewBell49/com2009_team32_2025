#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import necessary ROS interface types:
from sensor_msgs.msg import Image

import os
from pathlib import Path

class BeaconSearch(Node):

    def __init__(self):
        super().__init__("beacon_search")
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/color/image_raw",
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

        # most pixels of that colour detected
        self.highestm00 = 0

        # percent of the edge that needs to be covered, to be "out of bounds" (too close)
        self.edge_percent = 0.2
        # pixels that count as on the edge of the camera
        self.edge_pixels = 100
    
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
        crop_height = 250
        crop_z0 = int((width / 2) - (crop_width / 2))
        crop_y0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_y0:crop_y0+crop_height, crop_z0:crop_z0+crop_width]

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, self.lower_threshold, self.upper_threshold)
        m = cv2.moments(mask)
        # /255.0 in order to get number of total pixels detected
        m00 = m['m00']/255.0

        # get the left and right hand bands of the image
        left_side = cv_img[0:height, 0:self.edge_pixels]
        hsv_left = cv2.cvtColor(left_side, cv2.COLOR_BGR2HSV)
        mask_left = cv2.inRange(hsv_left, self.lower_threshold, self.upper_threshold)
        m_left = cv2.moments(mask_left)
        m00_left = m_left['m00']/255.0

        right_side = cv_img[0:height, (width-self.edge_pixels):width]
        hsv_right = cv2.cvtColor(right_side, cv2.COLOR_BGR2HSV)
        mask_right = cv2.inRange(hsv_right, self.lower_threshold, self.upper_threshold)
        m_right = cv2.moments(mask_right)
        m00_right = m_right['m00']/255.0

        edge_height, edge_width, _ = left_side.shape
        edge_size = edge_height * edge_width

        if (m00_left > edge_size*self.edge_percent or m00_right > edge_size*self.edge_percent) and self.highestm00 > 0:
            beacon_on_edge = True
            self.get_logger().info(f"Beacon on the edge")
        else:
            beacon_on_edge = False

        self.get_logger().info(f"{m00}")

        # beacon is detected, and theres at least a 10% increase in pixels detected, and the beacon is not on the edge of the camera
        if m00 > 0 and m00 > self.highestm00*1.1 and not beacon_on_edge:
            self.highestm00 = m00
            self.get_logger().info(f"Detected a beacon with {m00} pixels.")
            self.save_image(img = cv_img, img_name="target_beacon")
        
        # cv2.imshow("camera image", cropped_img)
        # cv2.imshow("right image", right_side)
        # cv2.imshow("left image", left_side)
        cv2.waitKey(1)
            
    def save_image(self, img, img_name): 
        self.get_logger().info(f"Saving the image...")

        base_image_path = Path.home().joinpath("ros2_ws/src/com2009_team32_2025/snaps/")
        base_image_path.mkdir(parents=True, exist_ok=True) 

        full_image_path = base_image_path.joinpath(
            f"{img_name}.jpg")
        self.get_logger().info(f"{full_image_path}")

        cv2.imwrite(str(full_image_path), img) 

        self.get_logger().info(
            f"\nSaved an image to '{full_image_path}'\n"
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
