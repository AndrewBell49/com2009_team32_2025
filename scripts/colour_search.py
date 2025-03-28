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
            topic="/camera/image_raw",
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
        
        self.m00_blue = 0
        self.m00_min_blue = 10000
        self.m00_red = 0
        self.m00_min_red = 10000
        self.m00_green = 0
        self.m00_min_green = 10000
        self.m00_cyan = 0
        self.m00_min_cyan = 10000

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
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        lower_blue = (115, 224, 100)
        upper_blue = (130, 255, 255)
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        m_blue = cv2.moments(mask_blue)
        self.m00_blue = m_blue['m00']
        self.cy_blue = m_blue['m10'] / (m_blue['m00'] + 1e-5)

        lower_red = (0, 192, 100)
        upper_red = (6, 260, 255)
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        m_red = cv2.moments(mask_red)
        self.m00_red = m_red['m00']
        self.cy_red = m_red['m10'] / (m_red['m00'] + 1e-5)

        lower_cyan = (80, 137, 100)
        upper_cyan = (94, 255, 255)
        mask_cyan = cv2.inRange(hsv_img, lower_cyan, upper_cyan)
        m_cyan = cv2.moments(mask_cyan)
        self.m00_cyan = m_cyan['m00']
        self.cy_cyan = m_cyan['m10'] / (m_cyan['m00'] + 1e-5)

        lower_green = (50, 154, 100)
        upper_green = (63, 255, 255)
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)
        m_green = cv2.moments(mask_green)
        self.m00_green = m_green['m00']
        self.cy_green = m_green['m10'] / (m_green['m00'] + 1e-5)

        if self.m00_blue > self.m00_min_blue:
            cv2.circle(crop_img, (int(self.cy_blue), 200), 10, (0, 255, 255), 2)
        
        if self.m00_red > self.m00_min_red:
            cv2.circle(crop_img, (int(self.cy_red), 200), 10, (255, 255, 0), 2)

        if self.m00_cyan > self.m00_min_cyan:
            cv2.circle(crop_img, (int(self.cy_cyan), 200), 10, (0, 0, 255), 2)
        
        if self.m00_green > self.m00_min_green:
            cv2.circle(crop_img, (int(self.cy_green), 200), 10, (255, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)
    
    def timer_callback(self):
        if self.stop_counter > 0:
            self.stop_counter -= 1

        if (self.m00_blue > self.m00_min_blue or
            self.m00_red > self.m00_min_red or
            self.m00_cyan > self.m00_min_cyan or
            self.m00_green > self.m00_min_green):
            # blob detected
            if (self.cy_blue >= 560-100 and self.cy_blue <= 560+100 or
                self.cy_red >= 560-100 and self.cy_red <= 560+100 or
                self.cy_cyan >= 560-100 and self.cy_cyan <= 560+100 or
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
            m00s = [self.m00_blue, self.m00_red, self.m00_cyan, self.m00_green]
            cys = np.array([self.cy_blue, self.cy_red, self.cy_cyan, self.cy_green])
            colours = ["blue", "red", "cyan", "green"]
            maxIndex = np.argmax(m00s)
            self.get_logger().info(
                f"\nMOVING SLOW:\n"
                f"A blob of {colours[maxIndex]} of size {m00s[maxIndex]:.0f} pixels is in view at y-position: {cys[maxIndex]:.0f} pixels."
            )
            self.vel_cmd.angular.z = self.turn_vel_slow
        
        elif self.move_rate == 'stop' and self.stop_counter > 0:
            m00s = [self.m00_blue, self.m00_red, self.m00_cyan, self.m00_green]
            cys = np.array([self.cy_blue, self.cy_red, self.cy_cyan, self.cy_green])
            colours = ["blue", "red", "cyan", "green"]
            maxIndex = np.argmax(m00s)
            self.get_logger().info(
                f"\nSTOPPED:\n"
                f"The blob of {colours[maxIndex]} is now dead-ahead at y-position {cys[maxIndex]:.0f} pixels... Counting down: {self.stop_counter}"
            )
            self.vel_cmd.angular.z = 0.0
        
        else:
            m00s = [self.m00_blue, self.m00_red, self.m00_cyan, self.m00_green]
            cys = np.array([self.cy_blue, self.cy_red, self.cy_cyan, self.cy_green])
            colours = ["blue", "red", "cyan", "green"]
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
