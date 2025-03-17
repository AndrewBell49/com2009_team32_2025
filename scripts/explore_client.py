#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 
from rclpy.signals import SignalHandlerOptions

from com2009_team32_2025.action import ExploreForward

class ExploreForwardClient(Node):

    def __init__(self):
        super().__init__("explore_client") 
        
        self.actionclient = ActionClient(
            node=self, 
            action_type=ExploreForward, 
            action_name="explore_forward"
        ) 
        self.goal_succeeded = False
        self.goal_cancelled = False
        self.stop = False

    def send_goal(self, fwd_velocity=0.0, stopping_distance=0.0): 
        goal = ExploreForward.Goal()
        goal.fwd_velocity = float(fwd_velocity)
        goal.stopping_distance = float(stopping_distance)

        self.actionclient.wait_for_server()

        self.send_goal_future = self.actionclient.send_goal_async(
            goal=goal, 
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("The goal was rejected by the server.")
            return

        self.get_logger().info("The goal was accepted by the server.")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        self._goal_handle = goal_handle
    
    def get_result_callback(self, future):
        self.goal_succeeded = True
        result = future.result().result
        self.get_logger().info(
            f"The action has completed.\n"
            f"Result:\n"
            f"  - Total Distance   = {result.total_distance_travelled:.2f}\n"
            f"  - Closest Obstacle = {result.closest_obstacle:.2f}"
        )
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        fdbk_current_distance_travelled = feedback.current_distance_travelled
        self.get_logger().info(
            f"\nFEEDBACK:\n"
            f"  - Current distance travelled = {fdbk_current_distance_travelled:.2f} m."
        )
        if self.stop:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_goal)
            
    def cancel_goal(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.goal_cancelled = True
        else:
            self.get_logger().info('Goal failed to cancel')

def main(args=None): 
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    action_client = ExploreForwardClient()
    future = action_client.send_goal(0.1, 0.4)
    while not action_client.goal_succeeded:
        try:
            rclpy.spin_once(action_client)
            if action_client.goal_cancelled:
                break
        except KeyboardInterrupt:
            print("Ctrl+C")
            action_client.stop = True

if __name__ == '__main__':
    main()
