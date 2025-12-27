#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import time

class NavInitializer(Node):
    def __init__(self):
        super().__init__('nav_initializer')
        
        # Publisher for initial pose (AMCL)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Publisher for navigation goal
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
    def set_initial_pose(self, x=-2.0, y=0.0, yaw=0.0):
        """Set robot's initial pose for AMCL localization"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        import math
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Set covariance (confidence)
        msg.pose.covariance[0] = 0.25   # x variance
        msg.pose.covariance[7] = 0.25   # y variance
        msg.pose.covariance[35] = 0.06  # yaw variance
        
        self.get_logger().info(f'Setting initial pose: x={x}, y={y}, yaw={yaw}')
        
        # Publish multiple times to ensure AMCL receives it
        for i in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.2)
        
        self.get_logger().info('✅ Initial pose set!')
    
    def send_goal(self, x, y, yaw=0.0):
        """Send navigation goal to Nav2"""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        import math
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'🎯 Sending goal: x={x}, y={y}, yaw={yaw}')
        
        # Publish multiple times
        for i in range(5):
            self.goal_pub.publish(msg)
            time.sleep(0.2)
        
        self.get_logger().info('Goal sent to Nav2!')

def main():
    rclpy.init()
    nav_init = NavInitializer()
    
    try:
        # Wait for Nav2 to be ready
        print("⏳ Waiting for Nav2 to initialize...")
        time.sleep(3)
        
        # Set initial pose (robot starts at -2.0, 0.0 in hallway)
        print("\n📍 Setting initial pose for AMCL localization...")
        nav_init.set_initial_pose(x=-2.0, y=0.0, yaw=0.0)
        
        # Wait for localization
        print("⏳ Waiting for AMCL to localize...")
        time.sleep(5)
        
        # Send test goal to living room (approximately x=-4.0, y=2.0)
        print("\n🎯 Sending navigation goal to living room...")
        nav_init.send_goal(x=-4.0, y=2.0, yaw=0.0)
        
        print("\n✅ Navigation initialized!")
        print("💡 Watch Gazebo - robot should start moving to the goal!")
        
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        nav_init.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
