#!/usr/bin/env python3

# Import the core Python libraries for ROS and to implement ROS Actions:
import rclpy
from rclpy.node import Node
from rclpy.action import CancelResponse

from com2009_team32_2025_modules.tb3_tools import quaternion_to_euler 

from rclpy.signals import SignalHandlerOptions

# Import other key ROS interfaces that this server will use:
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# Import some other useful Python Modules
from math import pi, atan2, sqrt, hypot
import numpy as np
import random

class Navigation(Node):

    def __init__(self):
        super().__init__("explore_server")

        self.first_message = True

        self.vel_cmd = Twist() 

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0; self.yawstart = 0.0
        self.lidar_front = 100.0; self.lidar_right = 100.0; self.lidar_left = 100.0

        self.stop_dist = 0.45
        self.wall_distance = 0.4
        self.angle_width = 30
        self.fwd_vel = 0.2
        self.angular_vel = 1.0

        self.shutdown = False

        self.await_odom = True 
        self.await_lidar = True

        self.turn_change = True
        self.turn_direction = 1

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

        # self.get_logger().info(
        #     f"The '{self.get_name()}' node is initialised."
        # )
        
    def map_callback(self, msg):
        # self.get_logger().info("Map received!")
        width = msg.info.width
        height = msg.info.height
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
                # self.get_logger().info("closest: " + str(closest))
                
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
        
        goal = self.select_closest_frontier()
        if goal:
            goal_x, goal_y = goal
            angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
            angle_error = angle_to_goal - self.yaw
            
            # Normalize angle error to [-pi, pi]
            angle_error = (angle_error + pi) % (2 * pi) - pi
            dist_to_goal = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

        
        # If too close to a wall in front, stop and rotate
        if self.lidar_front <= self.stop_dist:

            # only set direction of turn, at the start of a wall detection
            if self.turn_change:
                rand_num = random.randint(0, 2)
                if angle_error <= 0:
                    # 33% chance of going in the "wrong" directions (so it doesn't get stuck)
                    if rand_num == 0:
                        self.turn_direction = 0.5
                    else:
                        self.turn_direction = -0.5
                else:
                    # turn "wrong" direction for other way
                    if rand_num == 0:
                        self.turn_direction = -0.5
                    else:
                        self.turn_direction = 0.5
                self.turn_change = False
            
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = self.turn_direction*self.angular_vel

            self.vel_pub.publish(self.vel_cmd)
            return
        else:
            self.turn_change = True            
        
        # Move away from wall if too close
        dist_from_wall = min(self.lidar_left, self.lidar_right)
        if dist_from_wall < self.wall_distance:
            error = (self.wall_distance - dist_from_wall) * 2
            if self.lidar_left < self.lidar_right:
                self.vel_cmd.angular.z = -0.2
            else:
                self.vel_cmd.angular.z = 0.2
            self.vel_cmd.linear.x = min(self.fwd_vel, dist_to_goal*2)
            self.vel_pub.publish(self.vel_cmd)
            return

        # If no more frontiers, stop
        if not self.frontiers:
            # self.get_logger().info("Exploration complete - no frontiers left.")
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0
            self.vel_pub.publish(self.vel_cmd)
            return

        # Get closest frontier point
        if goal is None:
            # self.get_logger().info("No reachable frontier found.")
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0
            self.vel_pub.publish(self.vel_cmd)
            return

        # Compute angle to goal
        angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
        angle_error = angle_to_goal - self.yaw

        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + pi) % (2 * pi) - pi

        dist_to_goal = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

        # If close to goal, stop and wait for next map update
        if dist_to_goal < 0.3:
            # self.get_logger().info(f"Reached frontier at ({goal_x:.2f}, {goal_y:.2f})")

            self.frontiers = [f for f in self.frontiers if sqrt((f[0] - goal_x)**2 + (f[1] - goal_y)**2) > 0.5]

            # If there are still frontiers, select the next closest one
            if self.frontiers:
                # self.get_logger().info("Selecting next closest frontier.")
                goal = self.select_closest_frontier()
                # self.get_logger().info("goal: " + str(goal))
                goal_x, goal_y = goal

                # Compute angle to the next goal
                angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
                angle_error = angle_to_goal - self.yaw

                # Normalize angle error to [-pi, pi]
                angle_error = (angle_error + pi) % (2 * pi) - pi

                dist_to_goal = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

            else:
                # self.get_logger().info("No more frontiers left to explore.")
                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = 0.0
                self.vel_pub.publish(self.vel_cmd)
                return

        # Mix angle to goal with wall avoidance
        self.vel_cmd.linear.x = min(self.fwd_vel, dist_to_goal*2)
        self.vel_cmd.angular.z = 0.3 * angle_error

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