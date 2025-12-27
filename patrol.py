#!/usr/bin/env python3
"""
Autonomous Patrol Script for 2+1 Apartment
Uses Nav2 Simple Commander API to visit all rooms sequentially
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def create_pose(navigator, x, y, yaw=0.0):
    """Create a PoseStamped message for navigation"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    import math
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    return pose

def main():
    rclpy.init()
    
    # Initialize the navigator
    navigator = BasicNavigator()
    
    # Define waypoints for apartment patrol
    # User-specified coordinates for each room
    # Format: (name, x, y, yaw)
    waypoints = [
        ("Salon (Living Room)", -4.5, 2.0, 1.0),
        ("Mutfak (Kitchen)", -4.5, -3.0, 1.0),
        ("Yatak Odası 1 (Bedroom Top)", 4.5, 2.0, 1.0),
        ("Yatak Odası 2 (Bedroom Bottom)", 4.5, -3.0, 1.0),
        ("Koridor (Return Home)", -2.0, 0.0, 1.0),
    ]
    
    print("\n" + "="*60)
    print("🏠 APARTMENT PATROL STARTED")
    print("="*60)
    print(f"📍 Total waypoints: {len(waypoints)}")
    print("🤖 Robot will visit all rooms sequentially")
    print("="*60 + "\n")
    
    # Wait for Nav2 to be ready
    print("⏳ Waiting for Nav2 to be fully active...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 is ready!\n")
    
    # Set initial pose (robot starts at -2.0, 0.0)
    initial_pose = create_pose(navigator, -2.0, 0.0, 0.0)
    navigator.setInitialPose(initial_pose)
    print("📍 Initial pose set: Hallway (x=-2.0, y=0.0)\n")
    
    time.sleep(2)  # Give AMCL time to localize
    
    # Patrol loop
    patrol_count = 1
    try:
        while True:
            print(f"\n{'🔄 PATROL CYCLE #' + str(patrol_count):=^60}\n")
            
            for i, (name, x, y, yaw) in enumerate(waypoints, 1):
                print(f"[{i}/{len(waypoints)}] 🎯 Going to: {name}")
                print(f"           Coordinates: (x={x}, y={y})")
                
                # Create goal pose
                goal_pose = create_pose(navigator, x, y, yaw)
                
                # Send goal
                navigator.goToPose(goal_pose)
                
                # Wait for navigation to complete
                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    if feedback:
                        distance = feedback.distance_remaining
                        if distance > 0.1:  # Only print if meaningful distance
                            print(f"           Distance remaining: {distance:.2f}m", end='\r')
                    time.sleep(0.5)
                
                # Check result
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f"\n           ✅ Arrived at {name}!")
                elif result == TaskResult.CANCELED:
                    print(f"\n           ⚠️  Navigation to {name} was canceled")
                elif result == TaskResult.FAILED:
                    print(f"\n           ❌ Failed to reach {name}")
                
                # Brief pause at each waypoint
                time.sleep(2)
            
            print(f"\n{'✅ PATROL CYCLE #' + str(patrol_count) + ' COMPLETED':=^60}\n")
            patrol_count += 1
            
            # Ask if user wants to continue (comment out for infinite loop)
            print("🔄 Starting next patrol cycle in 5 seconds...")
            print("   (Press Ctrl+C to stop)\n")
            time.sleep(5)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Patrol interrupted by user")
        navigator.cancelTask()
    
    finally:
        print("\n" + "="*60)
        print(f"📊 PATROL SUMMARY")
        print("="*60)
        print(f"Total cycles completed: {patrol_count - 1}")
        print(f"Total waypoints visited: {(patrol_count - 1) * len(waypoints)}")
        print("="*60 + "\n")
        
        navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
