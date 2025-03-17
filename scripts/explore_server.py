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

class ExploreForwardServer(Node):

    def __init__(self):
        super().__init__("explore_server")

        self.first_message = True

        self.vel_cmd = Twist() 

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0; self.yawstart = 0.0
        self.lidar_front_wide = 0.0
        self.lidar_right = 0.0
        self.lidar_right = 0.0
        self.lidar_front = 0.0

        self.wall_distance = 0.3

        self.min_from_angle = 2*pi

        self.shutdown = False 

        self.await_odom = True 
        self.await_lidar = True

        self.goalReached = False

        self.spinning = False
        self.atEdge = False

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

        # Creating the action server:
        self.actionserver = ActionServer(
            node=self, 
            action_type=ExploreForward,
            action_name="explore_forward",
            execute_callback=self.server_execution_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
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

        if not self.first_message: 
            self.first_message = True
            self.xstart = self.x
            self.ystart = self.y
            self.yawstart = self.yaw
        
        self.yaw -= self.yawstart
        self.yaw = ((self.yaw + pi) % (2 * pi)) - pi

        self.await_odom = False

    def lidar_callback(self, scan_msg: LaserScan):
        """
        Callback for the /scan subscriber
        """

        # checks for wide berth at front (for avoiding obstacles)
        front_wide = np.array(scan_msg.ranges[0:41] + scan_msg.ranges[-40:] )
        self.lidar_front_wide = get_min_n_from_array(front_wide, 2)

        # narrower beam for front, because of closeness to edge
        front = np.array(scan_msg.ranges[0:20] + scan_msg.ranges[-20:])
        self.lidar_front = get_min_n_from_array(front, 10)

        # closest to right
        right = np.array(scan_msg.ranges[260:280])
        self.lidar_right = get_min_n_from_array(right, 2)

        self.await_lidar = False

    def goal_callback(self, goal: ExploreForward.Goal):
        """
        A callback to check that the goal inputs are valid        
        """
        goal_ok = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            goal_ok = False

        if goal.stopping_distance < 0.2:
            goal_ok = False

        return GoalResponse.ACCEPT if goal_ok else GoalResponse.REJECT

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

    def server_execution_callback(self, goal):
        result = ExploreForward.Result()
        feedback = ExploreForward.Feedback()
        fwd_vel = goal.request.fwd_velocity
        stop_dist = goal.request.stopping_distance

        self.get_logger().info(
            f"\n#####\n"
            f"The '{self.get_name()}' has been called.\n"
            f"Goal:\n"
            f"  - explore at {fwd_vel:.2f} m/s\n"
            f"  - stop {stop_dist:.2f} m in front of something\n" 
            f"Here we go..."
            f"\n#####\n")

        # set the robot's velocity (based on the request):
        self.vel_cmd.linear.x = fwd_vel

        # Get the robot's current position:
        while self.await_odom or self.await_lidar:
            continue
            
        ref_posx = self.x
        ref_posy = self.y
        dist_travelled = 0.0

        while not self.goalReached:

            # first, get to the edge
            if not self.atEdge:

                if (self.x >= (ref_posx + 2 - stop_dist) or
                    self.x <= (ref_posx - 2 + stop_dist) or 
                    self.y >= (ref_posy + 2 - stop_dist) or
                    self.y <= (ref_posy - 2 + stop_dist)):

                    self.atEdge = True

                # start spinning if obsticle detected
                if self.lidar_front_wide <= stop_dist:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 0.5
                # move forwawrd if
                else:
                    self.vel_cmd.linear.x = fwd_vel
                    self.vel_cmd.angular.z = 0.0

            # going round the edge
            else:
                print(f"{self.lidar_right:.2f}, {self.lidar_front_wide:.2f}")
                if self.lidar_front_wide <= stop_dist:
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 0.3
                elif self.lidar_right == float("nan"):
                    self.vel_cmd.linear.x = fwd_vel
                    self.vel_cmd.angular.z = 0.0
                else:
                    # error, in distance to wall (aim to keep even distance)
                    error = self.wall_distance - self.lidar_right
                    if error > 0.3:
                        error = 0.3

                    self.vel_cmd.linear.x = fwd_vel
                    self.vel_cmd.angular.z = error
            
            self.vel_pub.publish(self.vel_cmd)

            # check if there has been a request to cancel the action:
            if goal.is_cancel_requested:
                # stop the robot:
                for i in range(5):
                    self.vel_pub.publish(Twist())
                goal.canceled()
                self.get_logger().info(
                    f"Cancelled."
                )
                result.total_distance_travelled = dist_travelled
                result.closest_obstacle = float(self.lidar_front_wide)
                return result

            dist_travelled = sqrt((ref_posx - self.x)**2 + (ref_posy - self.y)**2)

            feedback.current_distance_travelled = dist_travelled
            goal.publish_feedback(feedback)

            self.loop_rate.sleep() 

        for i in range(5):
            self.vel_pub.publish(Twist())

        self.get_logger().info(
            f"{self.get_name()} complete."
        )
        goal.succeed()
        
        result.total_distance_travelled = sqrt((ref_posx - self.x)**2 + (ref_posy - self.y)**2)
        result.closest_obstacle = float(self.lidar_front_wide)
        return result

def get_min_n_from_array(arr, n):
    valid_data = arr[arr != float("inf")] 
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
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = ExploreForwardServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info(
            "Starting the Server (shut down with Ctrl+C)"
        )
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            "Server shut down with Ctrl+C"
        )
    finally:
        node.shutdown = True

if __name__ == '__main__':
    main()
