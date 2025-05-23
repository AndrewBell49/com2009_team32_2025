#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid

import os

class MapSaverClient(Node):
    def __init__(self):
        super().__init__('map_saver_client')
        
        self.map_received = False
        
        # Set the exact maps directory path and map name
        self.maps_dir = "/home/student/ros2_ws/src/com2009_team32_2025/maps"
        self.map_path = f"{self.maps_dir}/arena_map"
        
        os.makedirs(self.maps_dir, exist_ok=True)
        
        # Subscribe to map topic to verify data is available
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1
        )
        
        # Create the service client
        self.client = self.create_client(SaveMap, '/map_saver/save_map')
        
        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map saver service not available, waiting...')
        
        # Create a timer to check if we've received map data
        self.init_timer = self.create_timer(1.0, self.init_timer_callback)
        self.save_timer = None
        
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            
    def init_timer_callback(self):
        if self.map_received:
            self.init_timer.cancel()  # Stop the initialization timer
            # Start the periodic save timer
            self.save_timer = self.create_timer(10.0, self.save_map)
        
    def save_map(self):
        # Create the request
        request = SaveMap.Request()
        request.map_topic = "/map"
        request.map_url = self.map_path
        request.image_format = "png"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        
        # Send the async request
        future = self.client.call_async(request)
        future.add_done_callback(self.save_callback)
        
    def save_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main():
    rclpy.init()
    node = MapSaverClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main() 