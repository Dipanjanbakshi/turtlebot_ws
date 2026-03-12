#!/usr/bin/env python3
from tokenize import String

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class Wall_follower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(TwistStamped, 'wheel_controller/cmd_vel', 10)  # Assuming String for simplicity

        self.timer = self.create_timer(0.1, self.move)  # Move every 100ms
        self.front_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.back_distance = 0.0
        self.wall_distance= 0.3

        self.distance_tolerance = 0.1 
        self.angle_tolerance = 0.1

         # Speed limits
        self.max_linear_speed = 2.0 # m/s speed limit
        self.max_angular_speed = 2.0 # rad/s speed limit
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Front: combine 340-360 and 0-20 (0°)
        front = np.concatenate([ranges[-20:], ranges[:20]])
        # Replace inf and nan with 15.0
        front_ranges = np.nan_to_num(front, nan=12.0, posinf=12.0, neginf=12.0)
        front = np.min(front_ranges)
        self.front_distance = front
        
        # Left: 70-110 (90°)
        left = ranges[70:110]

        left_ranges = np.nan_to_num(left, nan=12.0, posinf=12.0, neginf=12.0)
        left = np.min(left_ranges)
        self.left_distance = left


        # Back: 170-190 (180°)
        back = ranges[170:190]
        back_ranges = np.nan_to_num(back, nan=12.0, posinf=12.0, neginf=12.0)
        back = np.min(back_ranges)
        self.back_distance = back
        
        # Right: 250-290 (270°)
        right = ranges[250:290]
        right_ranges = np.nan_to_num(right, nan=12.0, posinf=12.0, neginf=12.0)
        right = np.min(right_ranges)
        self.right_distance = right
        
        print(f"Front: {front:.2f}m | Left: {left:.2f}m | Right: {right:.2f}m | Back: {back:.2f}m")

    def move(self):
        twistStamped = TwistStamped()
        twistStamped.header.stamp = self.get_clock().now().to_msg()
        twistStamped.twist.linear.x = 0.4
        twistStamped.twist.angular.z = 0.0

        if self.right_distance > self.wall_distance and self.front_distance > 1.0:
            error = self.right_distance - self.wall_distance
            if self.right_distance > 1.0:
                twistStamped.twist.angular.z = 0.0
                twistStamped.twist.linear.x = 0.0
                self.publisher.publish(twistStamped)
            elif error <= self.distance_tolerance:
                twistStamped.twist.angular.z = 0.0  # Move straight
            else:
                speed = -1.5 * error  # Proportional control to turn left
                twistStamped.twist.angular.z  = min(speed, self.max_angular_speed)
        elif self.front_distance < 1.0 and self.right_distance > self.wall_distance:
            twistStamped.twist.angular.z = 0.8# Turn rightt
        elif self.front_distance < 1.0 and self.right_distance < self.wall_distance:
            twistStamped.twist.angular.z = 0.0 # Turn left    

        self.publisher.publish(twistStamped) 



def main(args=None):
    rclpy.init(args=args)
    node = Wall_follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()