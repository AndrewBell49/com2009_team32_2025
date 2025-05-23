#!/usr/bin/env python3
#task1 for assignment 2: Moving in a figure of 8

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 

from com2009_team32_2025_modules.tb3_tools import quaternion_to_euler 
from math import pi 

class FigureEight(Node):

    def __init__(self):
        super().__init__("task1")

        self.first_message = False
        self.turn = False 

        self.vel_msg = Twist() 
        
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.xstart = 0.0; self.ystart = 0.0; self.yawstart = 0.0

        self.second_loop = False
        self.within_start = True
        self.final_stop = False

        self.start_range = 0.1
        self.min_from_start = self.start_range

        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        )

        self.shutdown = False

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

    def on_shutdown(self):
        print("Stopping the robot...")
        self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 
        print(pose)

        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation) 

        self.x = pose.position.x 
        self.y = pose.position.y
        self.yaw = abs(yaw)

        if not self.first_message: 
            self.first_message = True
            self.xstart = self.x
            self.ystart = self.y
            self.yawstart = self.yaw

    def timer_callback(self):
        dist_from_start = euclid_dist(self.xstart, self.ystart, self.x, self.y)

        # first time it is out of the start range
        if dist_from_start > self.start_range:
            self.min_from_start = self.start_range
            self.within_start = False

            if self.second_loop:
                self.final_stop = True

        radius = 0.5 # m
        time = 55 # s
        linear_velocity = (4*pi*radius) / time # m/s
        angular_velocity = linear_velocity / radius

        if dist_from_start <= self.start_range and not self.within_start:
            # within range, but getting closer
            if dist_from_start < self.min_from_start:
                self.min_from_start = dist_from_start
            # first time the robot gets further away from start
            else:
                self.second_loop = True
                self.within_start = True

        if self.second_loop and self.within_start and self.final_stop:
            self.vel_msg = Twist()
            self.vel_pub.publish(self.vel_msg)
            self.get_logger().info(
                "Stopping", 
                throttle_duration_sec=1,
            )
        else:
            # opposite direction when first loop is completed
            if self.second_loop:
                angular_velocity *= -1

            self.vel_msg = Twist()
            self.vel_msg.linear.x = linear_velocity
            self.vel_msg.angular.z = angular_velocity

            self.vel_pub.publish(self.vel_msg)

            dAngle = self.yaw - self.yawstart
            dAngle *= (180/pi)
            self.get_logger().info(
                f"x={(self.xstart - self.x):.2f} [m], y={self.ystart - self.y:.2f} [m], yaw={dAngle:.1f} [degrees].", 
                throttle_duration_sec=1,
            )

def euclid_dist(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = FigureEight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()