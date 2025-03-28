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

import numpy as np

class ColourSearch(Node):

    def __init__(self):
        super().__init__("colour_search")
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/color/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )
        
        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="/cmd_vel",
            qos_profile=10
        )

        self.timer = self.create_timer(
            timer_period_sec=1/5,
            callback=self.timer_callback
        )

        self.cvbridge_interface = CvBridge()

        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.vel_cmd = Twist()
        self.vel_cmd.angular.z = self.turn_vel_fast

        self.move_rate = "" # fast, slow, stop
        self.stop_counter = 0

        self.shutdown = False

        self.total_pixels = 0
        self.percent_colour_detected = 0.1
        
        self.m00_blue = 0
        self.m00_red = 0
        self.m00_green = 0
        self.m00_yellow = 0

    def shutdown_ops(self):
        self.get_logger().info(
            "Shutting down..."
        )
        cv2.destroyAllWindows()
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"{e}")
        
        width, height, _ = cv_img.shape
        self.total_pixels = width * height

        crop_width = width - 100
        crop_height = 200
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_y0:crop_y0+crop_width]

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # /255.0 in order to get number of total pixels detected

        lower_blue = (101, 75, 100)
        upper_blue = (110, 255, 255)
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        m_blue = cv2.moments(mask_blue)
        self.m00_blue = m_blue['m00']/255.0
        self.cy_blue = m_blue['m10'] / (m_blue['m00'] + 1e-5)

        lower_red = (0, 75, 100)
        upper_red = (12, 255, 255)
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        m_red = cv2.moments(mask_red)
        self.m00_red = m_red['m00']/255.0
        self.cy_red = m_red['m10'] / (m_red['m00'] + 1e-5)

        lower_yellow = (22, 75, 100)
        upper_yellow = (28, 255, 255)
        mask_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        m_yellow = cv2.moments(mask_yellow)
        self.m00_yellow = m_yellow['m00']/255.0
        self.cy_yellow = m_yellow['m10'] / (m_yellow['m00'] + 1e-5)

        lower_green = (77, 75, 100)
        upper_green = (90, 255, 255)
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)
        m_green = cv2.moments(mask_green)
        self.m00_green = m_green['m00']/255.0
        self.cy_green = m_green['m10'] / (m_green['m00'] + 1e-5)

        if self.m00_blue > self.total_pixels*self.percent_colour_detected:
            cv2.circle(cv_img, (int(self.cy_blue), 200), 10, (0, 255, 255), 2)
        
        if self.m00_red > self.total_pixels*self.percent_colour_detected:
            cv2.circle(cv_img, (int(self.cy_red), 200), 10, (255, 255, 0), 2)

        if self.m00_yellow > self.total_pixels*self.percent_colour_detected:
            cv2.circle(cv_img, (int(self.cy_yellow), 200), 10, (0, 0, 255), 2)
        
        if self.m00_green > self.total_pixels*self.percent_colour_detected:
            cv2.circle(cv_img, (int(self.cy_green), 200), 10, (255, 0, 255), 2)
        
        cv2.imshow('cropped image', cropped_img)
        cv2.waitKey(1)
    
    def timer_callback(self):
        if self.stop_counter > 0:
            self.stop_counter -= 1

        if (self.m00_blue > self.total_pixels*self.percent_colour_detected or
            self.m00_red > self.total_pixels*self.percent_colour_detected or
            self.m00_yellow > self.total_pixels*self.percent_colour_detected or
            self.m00_green > self.total_pixels*self.percent_colour_detected):
            # blob detected
            if (self.cy_blue >= 560-100 and self.cy_blue <= 560+100 or
                self.cy_red >= 560-100 and self.cy_red <= 560+100 or
                self.cy_yellow >= 560-100 and self.cy_yellow <= 560+100 or
                self.cy_green >= 560-100 and self.cy_green <= 560+100):
                if self.move_rate == 'slow':
                    self.move_rate = 'stop'
                    self.stop_counter = 30
            else:
                self.move_rate = 'slow'
        else:
            self.move_rate = 'fast'
            
        if self.move_rate == 'fast':
            self.get_logger().info(
                "\nMOVING FAST:\n"
                "I can't see anything at the moment, scanning the area..."
            )
            self.vel_cmd.angular.z = self.turn_vel_fast
            
        elif self.move_rate == 'slow':
            m00s = [self.m00_blue, self.m00_red, self.m00_yellow, self.m00_green]
            cys = np.array([self.cy_blue, self.cy_red, self.cy_yellow, self.cy_green])
            colours = ["blue", "red", "yellow", "green"]
            maxIndex = np.argmax(m00s)
            self.get_logger().info(
                f"\nMOVING SLOW:\n"
                f"A blob of {colours[maxIndex]} of size {m00s[maxIndex]:.0f} pixels is in view at y-position: {cys[maxIndex]:.0f} pixels."
            )
            self.vel_cmd.angular.z = self.turn_vel_slow
        
        elif self.move_rate == 'stop' and self.stop_counter > 0:
            m00s = [self.m00_blue, self.m00_red, self.m00_yellow, self.m00_green]
            cys = np.array([self.cy_blue, self.cy_red, self.cy_yellow, self.cy_green])
            colours = ["blue", "red", "yellow", "green"]
            maxIndex = np.argmax(m00s)
            self.get_logger().info(
                f"\nSTOPPED:\n"
                f"The blob of {colours[maxIndex]} is now dead-ahead at y-position {cys[maxIndex]:.0f} pixels... Counting down: {self.stop_counter}"
            )
            self.vel_cmd.angular.z = 0.0
        
        else:
            m00s = [self.m00_blue, self.m00_red, self.m00_yellow, self.m00_green]
            cys = np.array([self.cy_blue, self.cy_red, self.cy_yellow, self.cy_green])
            colours = ["blue", "red", "yellow", "green"]
            maxIndex = np.argmax(m00s)
            self.get_logger().info(
                f"\nMOVING SLOW:\n"
                f"A blob of {colours[maxIndex]} of size {m00s[maxIndex]:.0f} pixels is in view at y-position: {cys[maxIndex]:.0f} pixels."
            )
            self.vel_cmd.angular.z = self.turn_vel_slow
        
        self.vel_pub.publish(self.vel_cmd)
            
def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = ColourSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"{node.get_name()} received a shutdown request (Ctrl+C)"
        )
    finally:
        node.shutdown_ops()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
