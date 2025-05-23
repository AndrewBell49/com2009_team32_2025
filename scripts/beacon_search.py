#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import necessary ROS interface types:
from sensor_msgs.msg import Image

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

        # original threshold for real robot
        # if target_colour == "yellow":
        #     self.lower_threshold = (22, 120, 0)
        #     self.upper_threshold = (32, 255, 200)
        # elif target_colour == "red":
        #     self.lower_threshold = (0, 150, 75)
        #     self.upper_threshold = (12, 255, 255)
        # elif target_colour == "green":
        #     self.lower_threshold = (75, 100, 20)
        #     self.upper_threshold = (88, 255, 255)
        # else: # blue
        #     self.lower_threshold = (101, 100, 50)
        #     self.upper_threshold = (126, 255, 255)

        if target_colour == "yellow":
            self.lower_threshold = (23, 120, 0)
            self.upper_threshold = (35, 255, 255)
        elif target_colour == "red":
            self.lower_threshold = (0, 150, 0)
            self.upper_threshold = (10, 255, 255)
        elif target_colour == "green":
            self.lower_threshold = (41, 100, 0)
            self.upper_threshold = (64, 255, 255)
        else: # blue
            self.lower_threshold = (101, 100, 0)
            self.upper_threshold = (126, 255, 255)

        # most pixels of that colour detected
        self.highestm00 = 0

        # percent of the edge that needs to be covered, to be "out of bounds" (too close)
        self.edge_percent = 0.05
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

        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

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

        if m00_left > edge_size*self.edge_percent or m00_right > edge_size*self.edge_percent:
            beacon_on_edge = True
        else:
            beacon_on_edge = False

        # beacon is detected, and theres at least a 10% increase in pixels detected, and the beacon is not on the edge of the camera
        if m00 > 0 and m00 > self.highestm00*1.1 and not beacon_on_edge:
            self.highestm00 = m00
            self.save_image(img = cv_img, img_name="target_beacon")

            # # shutdown if image is "good enough"
            # if m00 > 700.0:
            #     self.get_logger().info("Shutting down")
            #     self.destroy_node()
            #     rclpy.shutdown()
            
    def save_image(self, img, img_name):
        folder = "/home/student/ros2_ws/src/com2009_team32_2025/snaps"
        full_image_path = f"{folder}/{img_name}.jpg"

        cv2.imwrite(full_image_path, img) 
            
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
