#!/usr/bin/env python3
"""
Autonomous Apartment Patrol Script - SIMULATION TIME ENFORCED
Uses Nav2 Simple Commander API with strict sim time synchronization
"""

import rclpy
from rclpy.parameter import Parameter
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def create_pose_stamped(navigator, x, y, yaw=0.0):
    """
    Create a PoseStamped message for navigation goal with SIMULATION TIME
    
    Args:
        navigator: BasicNavigator instance (must use sim time)
        x: X coordinate in map frame
        y: Y coordinate in map frame
        yaw: Orientation in radians (default 0.0)
    
    Returns:
        PoseStamped message with simulation timestamp
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    
    # CRITICAL: Use navigator's clock (simulation time)
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
    # Initialize ROS 2 with SIMULATION TIME parameter
    rclpy.init()
    
    # Create the navigator with EXPLICIT sim time setting
    print("🕐 Initializing BasicNavigator with use_sim_time=True...")
    navigator = BasicNavigator()
    
    # CRITICAL: Verify simulation time parameter (already declared by BasicNavigator)
    use_sim_time = navigator.get_parameter('use_sim_time').value
    print(f"✓ use_sim_time parameter: {use_sim_time}")
    
    if not use_sim_time:
        print("❌ ERROR: use_sim_time is False! This will cause timestamp issues.")
        print("   Make sure to run: export ROS_DOMAIN_ID=0")
        print("   And start with: ros2 run ... --ros-args -p use_sim_time:=true")
        return
    
    # Define waypoints for the 2+1 apartment
    # Format: (room_name, x, y, yaw)
    waypoints = [
        ("Salon (Living Room)", -4.5, 2.0, 0.0),
        ("Mutfak (Kitchen)", -4.5, -3.0, 0.0),
        ("Yatak Odasi 1 (Bedroom 1)", 4.5, 2.0, 0.0),
        ("Yatak Odasi 2 (Bedroom 2)", 4.5, -3.0, 0.0),
        # TODO: Update Tuvalet coordinates with actual map values
        ("Tuvalet (Bathroom)", 0.0, 0.0, 0.0),  # Placeholder coordinates
        ("Koridor (Hallway - Home)", -2.0, 0.0, 0.0),
    ]
    
    print("\n" + "="*70)
    print("🏠 AUTONOMOUS APARTMENT PATROL")
    print("="*70)
    print(f"📍 Waypoints: {len(waypoints)} locations")
    print("🤖 Robot will visit each room sequentially")
    print("="*70 + "\n")
    
    # Wait for Nav2 to be ready
    print("⏳ Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 is ready!\n")
    
    # Set initial pose using simulation time
    initial_pose = create_pose_stamped(navigator, -2.0, 0.0, 0.0)
    print(f"🕐 Initial pose timestamp: {initial_pose.header.stamp.sec}.{initial_pose.header.stamp.nanosec}")
    navigator.setInitialPose(initial_pose)
    print("📍 Initial pose set: Koridor (x=-2.0, y=0.0)\n")
    
    # Wait for localization
    time.sleep(2)
    
    try:
        patrol_cycle = 1
        
        while True:  # Infinite patrol loop
            print(f"\n{'=' * 70}")
            print(f"🔄 PATROL CYCLE #{patrol_cycle}")
            print(f"{'=' * 70}\n")
            
            for i, (room_name, x, y, yaw) in enumerate(waypoints, 1):
                print(f"[{i}/{len(waypoints)}] 🎯 Going to: {room_name}")
                print(f"             Coordinates: (x={x}, y={y}, yaw={yaw})")
                
                # Create the goal pose with SIMULATION TIME
                goal_pose = create_pose_stamped(navigator, x, y, yaw)
                
                # Debug: Print timestamp
                print(f"             🕐 Goal timestamp: {goal_pose.header.stamp.sec}.{goal_pose.header.stamp.nanosec}")
                
                # Send the goal to Nav2
                navigator.goToPose(goal_pose)
                
                # Monitor progress
                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    if feedback:
                        distance_remaining = feedback.distance_remaining
                        if distance_remaining > 0.1:
                            print(f"             Distance remaining: {distance_remaining:.2f}m", end='\r')
                    time.sleep(0.5)
                
                # Check the result
                result = navigator.getResult()
                
                # DEBUG: Print detailed result
                print(f"\n             🔍 Navigation result: {result}")
                
                if result == TaskResult.SUCCEEDED:
                    print(f"             ✅ Arrived at {room_name}!")
                elif result == TaskResult.CANCELED:
                    print(f"             ⚠️  Navigation to {room_name} was CANCELED")
                    print(f"             Reason: User canceled or preempted the goal")
                elif result == TaskResult.FAILED:
                    print(f"             ❌ FAILED to reach {room_name}")
                    print(f"             Reason: Controller could not find a valid path or execution failed")
                elif result == TaskResult.UNKNOWN:
                    print(f"             ❓ UNKNOWN result for {room_name}")
                    print(f"             Reason: Goal may have been rejected or server error")
                else:
                    print(f"             ⚠️  Unexpected result: {result}")
                
                # Wait 2 seconds at the location
                print(f"             ⏸️  Waiting 2 seconds at {room_name}...")
                time.sleep(2)
            
            print(f"\n{'=' * 70}")
            print(f"✅ PATROL CYCLE #{patrol_cycle} COMPLETED")
            print(f"{'=' * 70}")
            print("\n🔄 Starting next cycle in 3 seconds...")
            print("   (Press Ctrl+C to stop)\n")
            time.sleep(3)
            
            patrol_cycle += 1
    
    except KeyboardInterrupt:
        print("\n\n🛑 Patrol stopped by user")
        navigator.cancelTask()
    
    finally:
        print("\n" + "="*70)
        print(f"📊 PATROL SUMMARY")
        print("="*70)
        print(f"Total cycles completed: {patrol_cycle - 1}")
        print(f"Total waypoints visited: {(patrol_cycle - 1) * len(waypoints)}")
        print("="*70 + "\n")
        
        # Cleanup
        navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
