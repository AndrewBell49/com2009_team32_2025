#!/usr/bin/env python3

# Import the core Python libraries for ROS and to implement ROS Actions:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from com2009_team32_2025_modules.tb3_tools import quaternion_to_euler 

# Import additional rclpy libraries for multithreading and shutdown
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.signals import SignalHandlerOptions

# Import our package's action interface:
from com2009_team32_2025.action import ExploreForward
# Import other key ROS interfaces that this server will use:
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Import some other useful Python Modules
from math import pi, atan2, atan, sin, cos, sqrt, pow
import numpy as np
import random
import time

class Navigation(Node):

    def __init__(self):
        super().__init__("explore_server")

        self.first_message = True

        self.vel_cmd = Twist() 

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0; self.yawstart = 0.0
        self.lidar_front = 100.0; self.lidar_right = 100.0; self.lidar_left = 100.0

        self.stop_dist = 0.4
        self.wall_distance = 0.4
        self.angle_width = 30
        self.fwd_vel = 0.26
        self.angular_vel = 1.3

        self.shutdown = False 

        self.choose_direction = True
        self.spin_direction = 1

        self.await_odom = True 
        self.await_lidar = True

        self.coordinates = [(-1.5, 1.5), (-0.5, 1.5), (0.5, 1.5), (1.5, 1.5),
                            (1.5, 0.5), (1.5, -0.5), (1.5, -1.5), (0.5, -1.5),
                            (-0.5, -1.5), (-1.5, -1.5), (-1.5, -0.5), (-1.5, 0.5)]

        self.loop_rate = self.create_rate(
            frequency=5, 
            clock=self.get_clock()
        ) 

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

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) 

        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        )

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

    def odom_callback(self, odom_msg: Odometry):
        """
        Callback for the /odom subscriber
        """
        pose = odom_msg.pose.pose

        (_, _, yaw) = quaternion_to_euler(pose.orientation) 

        self.x = pose.position.x 
        self.y = pose.position.y
        self.yaw = yaw

        # once in square, remove from list need to visit
        for coord in self.coordinates:
            if within_square((self.x, self.y), coord):
                self.coordinates.remove(coord)

        if self.first_message: 
            self.first_message = False
            self.xstart = self.x
            self.ystart = self.y
            self.yawstart = self.yaw

            # calculate center of squares, relative to starting pos
            for x in range(len(self.coordinates)):
                coords = self.coordinates[x]
                self.coordinates[x] = (coords[0] + self.xstart, coords[1] + self.ystart)

        self.await_odom = False

    def lidar_callback(self, scan_msg: LaserScan):
        """
        Callback for the /scan subscriber
        """
        # checks for wide berth at front (for avoiding obstacles)
        front = np.array(scan_msg.ranges[0:self.angle_width] + scan_msg.ranges[-self.angle_width:] )
        self.lidar_front = get_min_n_from_array(front, 2)

        # closest to right
        right = np.array(scan_msg.ranges[260:280])
        self.lidar_right = get_min_n_from_array(right, 2)

        # closest to left
        left = np.array(scan_msg.ranges[80:100])
        self.lidar_left = get_min_n_from_array(left, 2)

        self.await_lidar = False

    def cancel_callback(self, goal):
        """
        A callback to trigger cancellation of the action
        """
        self.get_logger().info('Received a cancel request...')
        return CancelResponse.ACCEPT

    def on_shutdown(self):
        """
        A method to stop the robot on shutdown
        """
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def timer_callback(self):

        if self.lidar_front <= self.stop_dist:
            self.vel_cmd.linear.x = 0.0

            if self.choose_direction:
                self.choose_direction = False
                
                left_count, right_count = count_points_left_right((self.x, self.y), self.yaw, self.coordinates)
                # add 1, so there is always a chance of turning both ways even if 0 on one side
                left_count += 1
                right_count += 1
                rand_choice = random.randint(1, left_count + right_count)

                self.spin_direction = 1
                if rand_choice <= right_count:
                    self.spin_direction = -1                    

            self.vel_cmd.angular.z = self.angular_vel * self.spin_direction

        else:
            self.choose_direction = True

            # move away from wall if too close
            dist_from_wall = min(self.lidar_left, self.lidar_right)
            if dist_from_wall < self.wall_distance:
                error = (self.wall_distance - dist_from_wall) * 2
                if self.lidar_left < self.lidar_right:
                    error *= -1

                if error > 1.82:
                    error = 1.82
                elif error < -1.82:
                    error = -1.82
            else:
                error = 0.0
            self.vel_cmd.angular.z = error
            self.vel_cmd.linear.x = self.fwd_vel

        self.vel_pub.publish(self.vel_cmd)

def get_min_n_from_array(arr, n):
    valid_data = arr[arr != float("inf")]
    valid_data = arr[arr != 0.0]
    # only get closest n obstacle degrees in range
    num_smallest = min(n, len(valid_data))
    closest = np.sort(valid_data)[:num_smallest]
    if np.shape(closest)[0] > 0: 
        return closest.mean() 
    else:
        return float("nan")

def within_square(c1, c2, radius=0.3):
    # if euclid_dist(c1[0], c1[1], c2[0], c2[1]) < radius:
    #     return True
    # return False
    if c2[0] < c1[0] - radius or c2[0] > c1[0] + radius or c2[1] < c1[1] - radius or c2[1] > c1[1] + radius:
        return False
    return True

def euclid_dist(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

def count_points_left_right(p, angle, coordinates):    
    left = 0
    right = 0

    dy = sin(angle)
    dx = cos(angle)

    # rotate pi/2 rads, in order to get a point that is definitely to the left
    ref_x, ref_y = p[0] - dy, p[1] + dx
    reference_d = (ref_x - p[0]) * dy - (ref_y - p[1]) * dx
    
    # formula from https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
    for x, y in coordinates:
        d = (x - p[0])*dy - (y - p[1])*dx

        # compare the left point, to the current point        
        if np.sign(d) == np.sign(reference_d):
            left += 1
        else:
            right += 1
    
    return left, right

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

if __name__ == '__main__':
    main()
