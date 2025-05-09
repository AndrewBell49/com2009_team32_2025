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
from nav_msgs.msg import OccupancyGrid

# Import some other useful Python Modules
from math import pi, atan2, atan, sin, cos, sqrt, pow, hypot
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
        
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile=10
        )

        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        )

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )
        
    def map_callback(self, msg):
        self.get_logger().info("Map received!")
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin
        data = np.array(msg.data).reshape((height, width))

        frontiers = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y][x] == 0:
                    neighbors = data[y-1:y+2, x-1:x+2]
                    if -1 in neighbors:
                        frontiers.append((x, y))

        self.frontiers = frontiers
        self.map_info = msg.info
        
    def grid_to_world(self, x, y):
        origin = self.map_info.origin
        resolution = self.map_info.resolution
        world_x = origin.position.x + (x + 0.5) * resolution
        world_y = origin.position.y + (y + 0.5) * resolution
        return world_x, world_y
    
    def select_closest_frontier(self):
        if not self.frontiers:
            return None

        min_dist = float('inf')
        closest = None
        for x, y in self.frontiers:
            wx, wy = self.grid_to_world(x, y)
            dist = hypot(wx - self.x, wy - self.y)
            if dist < min_dist and dist >= 1.0:
                min_dist = dist
                closest = (wx, wy)
                self.get_logger().info("closest: " + str(closest))
                
        return closest




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
        # Wait until map and lidar data are available
        if not hasattr(self, 'frontiers') or self.await_lidar or self.await_odom:
            return
        
        # If too close to a wall in front, stop and rotate
        if self.lidar_front <= self.stop_dist:
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = self.angular_vel
            self.vel_pub.publish(self.vel_cmd)
            return

        # If no more frontiers, stop
        if not self.frontiers:
            self.get_logger().info("Exploration complete - no frontiers left.")
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0
            self.vel_pub.publish(self.vel_cmd)
            return

    # Get closest frontier point
        goal = self.select_closest_frontier()
        if goal is None:
            self.get_logger().info("No reachable frontier found.")
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0
            self.vel_pub.publish(self.vel_cmd)
            return

        goal_x, goal_y = goal

        # Compute angle to goal
        angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
        angle_error = angle_to_goal - self.yaw

        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + pi) % (2 * pi) - pi

        dist_to_goal = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

        # If close to goal, stop and wait for next map update
        if dist_to_goal < 0.3:
            self.get_logger().info(f"Reached frontier at ({goal_x:.2f}, {goal_y:.2f})")

            self.frontiers = [f for f in self.frontiers if sqrt((f[0] - goal_x)**2 + (f[1] - goal_y)**2) > 0.5]

            # If there are still frontiers, select the next closest one
            if self.frontiers:
                self.get_logger().info("Selecting next closest frontier.")
                goal = self.select_closest_frontier()
                self.get_logger().info("goal: " + str(goal))
                goal_x, goal_y = goal

                # Compute angle to the next goal
                angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
                angle_error = angle_to_goal - self.yaw

                # Normalize angle error to [-pi, pi]
                angle_error = (angle_error + pi) % (2 * pi) - pi

                dist_to_goal = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

            else:
                self.get_logger().info("No more frontiers left to explore.")
                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = 0.0
                self.vel_pub.publish(self.vel_cmd)
                return

        # Obstacle avoidance still used here:
        dist_from_wall = min(self.lidar_left, self.lidar_right)
        obstacle_avoidance = 0.0
        if dist_from_wall < self.wall_distance:
            error = (self.wall_distance - dist_from_wall) * 2
            if self.lidar_left < self.lidar_right:
                error *= -1
            error = max(min(error, 1.82), -1.82)
            obstacle_avoidance = error

        # Mix angle to goal with wall avoidance
        self.vel_cmd.linear.x = min(self.fwd_vel, dist_to_goal)
        self.vel_cmd.angular.z = 0.5 * angle_error + obstacle_avoidance

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