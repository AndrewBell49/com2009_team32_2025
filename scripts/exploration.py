#!/usr/bin/env python3

# Import the core Python libraries for ROS and to implement ROS Actions:
import rclpy
from rclpy.node import Node
from rclpy.action import CancelResponse
from rclpy.signals import SignalHandlerOptions

from com2009_team32_2025_modules.tb3_tools import quaternion_to_euler 

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
        self.lidar_front_wide = 100.0; self.lidar_back_wide = 100.0

        self.stop_dist = 0.4
        self.wall_distance = 0.4
        self.very_close_dist = 0.25
        self.narrow_dist = 0.7

        self.angle_width = 30
        self.narrow_angle_width = 10

        self.fwd_vel = 0.26
        self.angular_vel = 1.0
        self.close_angular_vel = 0.5

        self.shutdown = False

        self.await_odom = True 
        self.await_lidar = True

        self.turn_change = True
        self.turn_change_front_and_back = True
        self.turn_direction = 1
        self.rotating_away = False

        self.visited_frontiers = []

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
        
    def map_callback(self, msg):
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

            # skip that fronteir if it is too close to one already visited
            if any(hypot(wx - vx, wy - vy) < 0.5 for vx, vy in self.visited_frontiers):
                continue

            dist = hypot(wx - self.x, wy - self.y)
            if dist < min_dist and dist >= 1.0:
                min_dist = dist
                closest = (wx, wy)
        
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

        # checks for narrow width at front (for stopping rotating)
        front_narrow = np.array(scan_msg.ranges[0:self.narrow_angle_width] + scan_msg.ranges[-self.narrow_angle_width:] )
        self.lidar_front_narrow = get_min_n_from_array(front_narrow, 2)

        # closest to right
        right = np.array(scan_msg.ranges[260:280])
        self.lidar_right = get_min_n_from_array(right, 2)

        # closest to left
        left = np.array(scan_msg.ranges[80:100])
        self.lidar_left = get_min_n_from_array(left, 2)

        # single closest to the front and back, for very close collisions
        front_wide = np.array(scan_msg.ranges[0:50] + scan_msg.ranges[-50:] )
        self.lidar_front_wide = get_min_n_from_array(front_wide, 1)
        back_wide = np.array(scan_msg.ranges[130:230])
        self.lidar_back_wide = get_min_n_from_array(back_wide, 1)

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
        self.get_logger().info("Stopping the robot")
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def timer_callback(self):
        # Wait until map and lidar data are available
        if not hasattr(self, 'frontiers') or self.await_lidar or self.await_odom:
            return

        dist_to_goal = 0.0
        angle_error = 0.0
        
        goal = self.select_closest_frontier()
        if goal:
            goal_x, goal_y = goal
            angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
            angle_error = angle_to_goal - self.yaw
            
            # Normalize angle error to [-pi, pi]
            angle_error = (angle_error + pi) % (2 * pi) - pi
            dist_to_goal = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

        # Rotate if very close to the front and back
        if self.lidar_front_wide <= self.very_close_dist and self.lidar_back_wide <= self.very_close_dist:

            # only set direction of turn, at the start of a wall detection
            if self.turn_change_front_and_back:
                self.close_angular_vel = self.angular_vel * 0.5
                # closer to the left, turn away
                if self.lidar_right > self.lidar_left:
                    self.close_angular_vel *= -1
                
                self.turn_change_front_and_back = False

            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = self.close_angular_vel
            self.vel_pub.publish(self.vel_cmd)
            return
        else:
            self.turn_change_front_and_back = True

        # Reverse if very close to the front
        if self.lidar_front <= self.very_close_dist:

            self.vel_cmd.linear.x = -0.15
            self.vel_cmd.angular.z = 0.0
            self.vel_pub.publish(self.vel_cmd)
            return


        # If too close to a wall in front, stop and rotate
        if self.lidar_front <= self.stop_dist:

            # start rotating (if block below)
            self.rotating_away = True

            # only set direction of turn, at the start of a wall detection
            if self.turn_change:
                rand_num = random.randint(0, 3)
                if angle_error <= 0:
                    # 25% chance of going in the "wrong" directions (so it doesn't get stuck)
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
            
        else:
            self.turn_change = True
        
        if self.rotating_away:
            
            # stop rotating, if no close object in the narrow front
            if self.lidar_front_narrow > self.narrow_dist and self.lidar_front > self.stop_dist:
                self.rotating_away = False
            else:
                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = self.turn_direction*self.angular_vel

                self.vel_pub.publish(self.vel_cmd)
                return   
        
        # Move away from wall if too close, and towards the wall if slightly far away
        dist_from_wall = min(self.lidar_left, self.lidar_right)
        if dist_from_wall < self.wall_distance:

            # aim for 0.65*self.wall_distance away from wall - towards if further away, away if nearer
            angular_vel_error = ((self.wall_distance*0.65) - dist_from_wall)
            
            if self.lidar_left < self.lidar_right:
                self.vel_cmd.angular.z = -1 * angular_vel_error
            else:
                self.vel_cmd.angular.z = angular_vel_error
            self.vel_cmd.linear.x = min(self.fwd_vel, dist_to_goal)
            self.vel_pub.publish(self.vel_cmd)
            return

        # If no more frontiers, slowly forward
        if not self.frontiers:
            self.vel_cmd.linear.x = 0.2
            self.vel_cmd.angular.z = 0.0
            self.vel_pub.publish(self.vel_cmd)
            return

        # Get closest frontier point
        if goal is None:
            self.vel_cmd.linear.x = 0.2
            self.vel_cmd.angular.z = 0.0
            self.vel_pub.publish(self.vel_cmd)
            return

        # Compute angle to goal
        angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
        angle_error = angle_to_goal - self.yaw

        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + pi) % (2 * pi) - pi

        # If close to goal, stop and wait for next map update
        if dist_to_goal < 0.3:

            self.visited_frontiers.append((goal_x, goal_y))

            self.frontiers = [f for f in self.frontiers if sqrt((f[0] - goal_x)**2 + (f[1] - goal_y)**2) > 0.5]

            # If there are still frontiers, select the next closest one
            if self.frontiers:
                goal = self.select_closest_frontier()
                goal_x, goal_y = goal

                # Compute angle to the next goal
                angle_to_goal = atan2(goal_y - self.y, goal_x - self.x)
                angle_error = angle_to_goal - self.yaw

                # Normalize angle error to [-pi, pi]
                angle_error = (angle_error + pi) % (2 * pi) - pi

                dist_to_goal = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

            else:
                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = 0.0
                self.vel_pub.publish(self.vel_cmd)
                return

        # Mix angle to goal with wall avoidance
        self.vel_cmd.linear.x = min(self.fwd_vel, dist_to_goal)
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