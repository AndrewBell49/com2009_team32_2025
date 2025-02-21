#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 

from part2_navigation_modules.tb3_tools import quaternion_to_euler 
from math import sqrt, pow, pi 

class Square(Node):

    def __init__(self):
        super().__init__("move_square")

        self.first_message = False
        self.turn = False 

        self.vel_msg = Twist() 
        
        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0
        
        self.yaw = 0.0 
        self.displacement = 0.0 

        self.turnCount = 0

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

        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation) 

        self.x = pose.position.x 
        self.y = pose.position.y
        self.theta_z = yaw # abs(yaw) makes life much easier!!

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z

    def timer_callback(self):
        # here is where the code to control the motion of the robot 
        # goes. Add code here to make the robot move in a square of
        # dimensions 1 x 1m...

        if self.turnCount >= 8:
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0

        elif self.turn:
            self.vel_msg.angular.z = 0.2

            dTheta = min(abs(self.theta_z - self.theta_zref), 2*pi - abs(self.theta_z - self.theta_zref))

            if dTheta >= pi/2:
                # stop robot turning
                self.vel_msg.angular.z = 0.0
                self.turn = False
                #save locational reference
                self.xref = self.x
                self.yref = self.y

                self.turnCount += 1

        else:
            self.vel_msg.linear.x = 0.4

            dist = euclid_dist(self.x, self.y, self.xref, self.yref)
            if dist >= 1.0:
                # stop robot moving
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.turn = True
                # save rotation reference
                self.theta_zref = self.theta_z

        # publish whatever velocity command has been set above:
        self.vel_pub.publish(self.vel_msg)

def euclid_dist(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2) ** 0.5

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    move_square = Square()
    try:
        rclpy.spin(move_square)
    except KeyboardInterrupt:
        print(
            f"{move_square.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        move_square.on_shutdown()
        while not move_square.shutdown:
            continue
        move_square.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()