#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move_forward(self, speed=0.3, duration=3.0):
        """Move robot forward"""
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration} seconds...')
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        
        self.stop()
        
    def turn_left(self, angular_speed=0.5, duration=2.0):
        """Turn robot left"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed
        
        self.get_logger().info(f'Turning left at {angular_speed} rad/s for {duration} seconds...')
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        
        self.stop()
        
    def turn_right(self, angular_speed=0.5, duration=2.0):
        """Turn robot right"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -angular_speed
        
        self.get_logger().info(f'Turning right at {angular_speed} rad/s for {duration} seconds...')
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        
        self.stop()
    
    def stop(self):
        """Stop the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Stopped')

def main():
    rclpy.init()
    robot = RobotMover()
    
    try:
        print("\n=== Robot Hareket Demo ===")
        print("Robot dairede gezecek...\n")
        
        time.sleep(2)  # Wait for connections
        
        # Demo hareket: İleri git, sola dön, tekrar ileri git
        robot.move_forward(speed=0.5, duration=3.0)
        time.sleep(1)
        
        robot.turn_left(angular_speed=0.8, duration=1.5)
        time.sleep(1)
        
        robot.move_forward(speed=0.5, duration=3.0)
        time.sleep(1)
        
        robot.turn_right(angular_speed=0.8, duration=1.5)
        time.sleep(1)
        
        robot.move_forward(speed=0.5, duration=2.0)
        
        print("\n✅ Hareket tamamlandı!")
        
    except KeyboardInterrupt:
        print("\nDurduruldu!")
    finally:
        robot.stop()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
