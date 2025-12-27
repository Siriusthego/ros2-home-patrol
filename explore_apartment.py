#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ApartmentExplorer(Node):
    def __init__(self):
        super().__init__('apartment_explorer')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Move robot with given velocities for duration"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        
        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    
    def explore_apartment(self):
        """Navigate through all rooms in the apartment for SLAM"""
        self.get_logger().info('🗺️ Starting apartment exploration for mapping...')
        
        time.sleep(3)  # Wait for SLAM to initialize
        
        # Start from hallway, explore systematically
        movements = [
            # Move forward into hallway
            ("Moving into hallway", 0.3, 0.0, 3.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Turn left to face living room door
            ("Turning to living room", 0.0, 0.5, 2.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Enter living room
            ("Entering living room", 0.3, 0.0, 4.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Scan living room - rotate
            ("Scanning living room", 0.0, 0.4, 6.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Move deeper into living room
            ("Exploring living room", 0.3, 0.0, 2.0),
            ("Rotate", 0.0, 0.4, 6.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Return to hallway
            ("Returning to hallway", -0.3, 0.0, 4.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Turn to face WC
            ("Turning to WC", 0.0, 0.5, 2.0),
            ("Move to WC", 0.3, 0.0, 2.5),
            ("Scan WC", 0.0, 0.4, 4.0),
            ("Back from WC", -0.3, 0.0, 2.5),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Go to kitchen door
            ("Moving to kitchen", 0.3, 0.0, 3.0),
            ("Turn to kitchen", 0.0, -0.5, 2.0),
            ("Enter kitchen", 0.3, 0.0, 3.0),
            ("Scan kitchen", 0.0, 0.4, 6.0),
            ("Explore kitchen", 0.3, 0.0, 2.0),
            ("Rotate in kitchen", 0.0, 0.4, 6.0),
            
            # Return to hallway
            ("Exit kitchen", -0.3, 0.0, 3.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Move to bedroom 1
            ("Turn to hallway", 0.0, 0.5, 2.0),
            ("Move in hallway", 0.3, 0.0, 4.0),
            ("Turn to bedroom 1", 0.0, 0.5, 2.0),
            ("Enter bedroom 1", 0.3, 0.0, 4.0),
            ("Scan bedroom 1", 0.0, 0.4, 7.0),
            ("Move in bedroom 1", 0.3, 0.0, 2.0),
            ("Final scan bedroom 1", 0.0, 0.4, 6.0),
            
            # Return to hallway
            ("Exit bedroom 1", -0.3, 0.0, 4.0),
            ("Pause", 0.0, 0.0, 1.0),
            
            # Go to bedroom 2
            ("Turn to bedroom 2", 0.0, -0.5, 4.0),
            ("Move to bedroom 2", 0.3, 0.0, 5.0),
            ("Turn to bedroom 2 door", 0.0, -0.5, 2.0),
            ("Enter bedroom 2", 0.3, 0.0, 4.0),
            ("Scan bedroom 2", 0.0, 0.4, 7.0),
            ("Explore bedroom 2", 0.3, 0.0, 2.0),
            ("Final scan bedroom 2", 0.0, 0.4, 6.0),
            
            # Final return to center
            ("Returning to center", -0.3, 0.0, 4.0),
            ("Final orientation", 0.0, 0.5, 3.0),
            ("Complete", 0.0, 0.0, 2.0),
        ]
        
        for i, (description, linear, angular, duration) in enumerate(movements, 1):
            self.get_logger().info(f'Step {i}/{len(movements)}: {description}')
            self.move(linear, angular, duration)
        
        self.get_logger().info('✅ Apartment exploration complete!')

def main():
    rclpy.init()
    explorer = ApartmentExplorer()
    
    try:
        explorer.explore_apartment()
        time.sleep(2)
    except KeyboardInterrupt:
        explorer.get_logger().info('Exploration interrupted')
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
