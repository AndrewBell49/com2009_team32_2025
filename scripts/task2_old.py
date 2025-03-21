#!/usr/bin/env python3
#task2 for assignment 2: Navigating around obsticles

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 

from com2009_team32_2025_modules.tb3_tools import quaternion_to_euler 
from math import pi, atan2, atan, sin, cos
import time

class Navigation(Node):

    def __init__(self):
        super().__init__("task2")

        self.first_message = False
        self.turn = False 

        self.vel_msg = Twist() 
        
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.xstart = 0.0; self.ystart = 0.0; self.yawstart = 0.0

        self.spinning = True
        self.min_from_angle = 2*pi

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

        (_, _, yaw) = quaternion_to_euler(pose.orientation) 

        self.x = pose.position.x 
        self.y = pose.position.y
        self.yaw = yaw

        if not self.first_message: 
            self.first_message = True
            self.xstart = self.x
            self.ystart = self.y
            self.yawstart = self.yaw
        
        self.yaw -= self.yawstart
        self.yaw = ((self.yaw + pi) % (2 * pi)) - pi

    def timer_callback(self):

        self.move_to_square(1)
        # dist_from_start = euclid_dist(self.xstart, self.ystart, self.x, self.y)

        # # first time it is out of the start range
        # if dist_from_start > self.stapose = msg_data.pose.pose 
        # radius = 0.5 # m
        # time = 55 # s
        # linear_velocity = (4*pi*radius) / time # m/s
        # angular_velocity = linear_velocity / radius

        # if dist_from_start <= self.start_range and not self.within_start:
        #     # within range, but getting closer
        #     if dist_from_start < self.min_from_start:
        #         self.min_from_start = dist_from_start
        #     # first time the robot gets further away from start
        #     else:
        #         self.second_loop = True
        #         self.within_start = True

        # if self.second_loop and self.within_start and self.final_stop:
        #     self.vel_msg = Twist()
        #     self.vel_pub.publish(self.vel_msg)
        #     self.get_logger().info(self.vel_msg = Twist()
        #     )
        # else:1
        #     self.vel_pub.publish(self.vel_msg)
        #     dAngle = self.yaw - self.yawstart
        #     dAngle *= (180/pi)
        #     self.get_logger().info(
        #         f"x={(self.xstartself.vel_msg = Twist() - self.x):.2f} [m], y={self.ystart - self.y:.2f} [m], yaw={dAngle:.1f} [degrees].", 
        #         throttle_duration_sec=1,
        #     )

    def move_to_square(self, square_number):        
        square_pos = get_square_pos(square_number, self.xstart, self.ystart)

        diff_x = square_pos[0] - self.x
        diff_y = square_pos[1] - self.y

        angle = -1 * atan(diff_x/diff_y)

        if square_pos[0] <= self.x and square_pos[1] >= self.y:
            angle *= -1
        elif square_pos[0] <= self.x and square_pos[1] <= self.y:
            angle = pi - angle
        elif square_pos[0] >= self.x and square_pos[1] <= self.y:
            angle -= pi

        self.vel_msg = Twist()
        
        if self.spinning:
            self.spinning = self.spin_to_angle(angle)
        else:
            dist_from_start = euclid_dist(square_pos[0], square_pos[1], self.x, self.y)
            print(f'Square: {square_pos[0]:.1f}, {square_pos[1]:.1f}')
            print(f'Self:   {self.x:.1f}, {self.y:.1f}')
            print(f'Dist:   {dist_from_start:.1f}')

            if dist_from_start > 0.1:
                self.vel_msg.linear.x = 0.2
            else:
                self.vel_msg.linear.x = 0.0

            self.vel_pub.publish(self.vel_msg)

    def spin_to_angle(self, angle):

        dTheta = angle - self.yaw
        dTheta = atan2(sin(dTheta), cos(dTheta))

        if abs(dTheta) < self.min_from_angle:
            self.min_from_angle = abs(dTheta)
            self.vel_msg.angular.z = 0.5

            # anticlockwise is the fasted way to get there 
            if dTheta < 0:
                self.vel_msg.angular.z *= -1
            
            return True
        else:
            self.vel_msg.angular.z = 0.0
            return False



def euclid_dist(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

def get_square_pos(square_number, initialX, initialY):
    coordinates = [(-1.5, 1.5), (-0.5, 1.5), (0.5, 1.5), (1.5, 1.5),
                   (1.5, 0.5), (1.5, -0.5), (1.5, -1.5), (0.5, -1.5),
                   (-0.5, -1.5), (-1.5, -1.5), (-1.5, -0.5), (-1.5, 0.5)]
    if square_number < 1 or square_number > 12:
        return (initialX, initialY)
    return (coordinates[square_number - 1][0] + initialX, coordinates[square_number - 1][1] + initialY)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = Navigation()
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
