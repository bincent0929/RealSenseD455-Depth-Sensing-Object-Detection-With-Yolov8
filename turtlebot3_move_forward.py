#!/usr/bin/env python3
"""
TurtleBot3 Burger - Move Forward 1 Meter Test
This script makes the robot move forward 1 meter using velocity commands.
"""

import os
import time

# Set ROS domain ID before importing rclpy
os.environ['ROS_DOMAIN_ID'] = '55'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class TurtleBot3MoveForward(Node):
    def __init__(self):
        super().__init__('turtlebot3_move_forward')
        
        # QoS profile for better compatibility
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        
        # Subscriber for odometry to track distance traveled
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )
        
        # Movement parameters
        self.linear_speed = 0.21  # m/s (safe speed for TurtleBot3 Burger)
        self.target_distance = 1.0  # meters
        
        # Position tracking
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_traveled = 0.0
        self.odom_received = False
        
        self.get_logger().info('TurtleBot3 Move Forward Node initialized')
        self.get_logger().info(f'Target: Move forward {self.target_distance}m at {self.linear_speed}m/s')
        
    def odom_callback(self, msg):
        """Track robot position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f'Start position: x={self.start_x:.3f}, y={self.start_y:.3f}')
        
        # Calculate distance traveled
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        self.distance_traveled = math.sqrt(dx*dx + dy*dy)
        self.odom_received = True
        
    def stop_robot(self):
        """Send stop command to robot."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        # Publish stop multiple times to ensure it's received
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)
        self.get_logger().info('Robot stopped')
        
    def move_forward(self):
        """Move the robot forward 1 meter."""
        # Create velocity message
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = 0.0
        
        self.get_logger().info('Waiting for odometry data...')
        
        # Wait for first odometry message
        timeout = 5.0
        start_wait = time.time()
        while not self.odom_received and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not self.odom_received:
            self.get_logger().warn('No odometry received! Moving based on time instead.')
            self.move_forward_timed()
            return
            
        self.get_logger().info('Starting movement...')
        
        # Move until target distance reached
        while self.distance_traveled < self.target_distance:
            self.cmd_vel_pub.publish(vel_msg)
            self.get_logger().info(f'Distance: {self.distance_traveled:.3f}m / {self.target_distance}m')
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
            
        # Stop the robot
        self.stop_robot()
        self.get_logger().info(f'Movement complete! Total distance: {self.distance_traveled:.3f}m')
        
    def move_forward_timed(self):
        """Fallback: Move forward based on time if no odometry available."""
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = 0.0
        
        # Calculate duration based on distance and speed
        duration = self.target_distance / self.linear_speed
        
        self.get_logger().info(f'Moving forward for {duration:.2f} seconds (time-based)')
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(vel_msg)
            elapsed = time.time() - start_time
            self.get_logger().info(f'Time: {elapsed:.2f}s / {duration:.2f}s')
            time.sleep(0.1)
            
        self.stop_robot()
        self.get_logger().info('Time-based movement complete!')


def main():
    print('What model of TurtleBot3 are you using?')
    model = input().strip()
    os.environ['TURTLEBOT3_MODEL'] = model

    print('=' * 50)
    print(f'TurtleBot3 {model} - Move Forward 1 Meter')
    print(f'ROS_DOMAIN_ID: {os.environ.get("ROS_DOMAIN_ID", "not set")}')
    print(f'TURTLEBOT3_MODEL: {os.environ.get("TURTLEBOT3_MODEL", "not set")}')
    print('=' * 50)
    
    rclpy.init()
    
    node = TurtleBot3MoveForward()
    
    try:
        # Give time for publisher to be discovered
        time.sleep(1.0)
        node.get_logger().info('Starting in 2 seconds...')
        time.sleep(2.0)
        
        # Move forward
        node.move_forward()
        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        print('Done!')


if __name__ == '__main__':
    main()
