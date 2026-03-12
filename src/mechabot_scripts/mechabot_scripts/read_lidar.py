#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Front: combine 340-360 and 0-20 (0°)
        front = np.concatenate([ranges[-20:], ranges[:20]])


        # Replace inf and nan with 15.0
        front_ranges = np.nan_to_num(front, nan=12.0, posinf=12.0, neginf=12.0)

        front = np.min(front_ranges)
        
        # Left: 70-110 (90°)
        left = ranges[70:110]

        left_ranges = np.nan_to_num(left, nan=12.0, posinf=12.0, neginf=12.0)
        left = np.min(left_ranges)


        # Back: 170-190 (180°)
        back = ranges[170:190]
        back_ranges = np.nan_to_num(back, nan=12.0, posinf=12.0, neginf=12.0)
        back = np.min(back_ranges)
        
        # Right: 250-290 (270°)
        right = ranges[250:290]
        right_ranges = np.nan_to_num(right, nan=12.0, posinf=12.0, neginf=12.0)
        right = np.min(right_ranges)
        
        print(f"Front: {front:.2f}m | Left: {left:.2f}m | Right: {right:.2f}m | Back: {back:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()