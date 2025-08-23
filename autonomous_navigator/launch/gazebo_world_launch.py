#!/usr/bin/env python3

"""
TurtleBot3 Waypoint Navigator using Nav2
=====================================

This node sends navigation goals to Nav2 stack for autonomous waypoint navigation.
The robot will continuously cycle through predefined waypoints while avoiding obstacles.

Author: Your Name
License: MIT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import math


class WaypointNavigator(Node):
    """
    Autonomous waypoint navigation node for TurtleBot3
    
    This node manages a list of waypoints and sends navigation goals
    to the Nav2 stack for autonomous navigation with obstacle avoidance.
    """
    
    def __init__(self):
        """Initialize the waypoint navigator node."""
        super().__init__('waypoint_navigator')
        
        # Create action client for Nav2 NavigateToPose action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Wait for action server to be available
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server is ready!')
        
        # Define waypoints - SMALLER coordinates that fit within turtlebot3_world
        # These coordinates are relative to the map origin
        # Format: (x, y, yaw_radians)
        self.waypoints = [
            (0.5, 0.5, 0.0),      # Point 1: Small move forward and right
            (0.5, -0.5, -1.57),   # Point 2: Move down, face south
            (-0.5, -0.5, 3.14),   # Point 3: Move left, face west  
            (-0.5, 0.5, 1.57),    # Point 4: Move up, face north
            (0.0, 0.0, 0.0),      # Point 5: Return to start, face east
        ]
        
        self.current_waypoint_index = 0
        self.navigation_complete = False
        
        # Timer to check navigation status and send next waypoint
        self.timer = self.create_timer(2.0, self.navigation_timer_callback)
        
        self.get_logger().info(f'Waypoint Navigator initialized with {len(self.waypoints)} waypoints')
        self.get_logger().info('MAKE SURE GAZEBO IS RUNNING IN TERMINAL 1!')
        
        # Start navigation immediately
        self.send_next_goal()

    def navigation_timer_callback(self):
        """
        Timer callback to check navigation status and send next waypoint.
        
        This callback runs every 2 seconds to monitor the navigation progress
        and automatically send the next waypoint when current navigation completes.
        """
        if self.navigation_complete:
            self.get_logger().info('Navigation to current waypoint completed!')
            time.sleep(1)  # Brief pause between waypoints
            self.send_next_goal()

    def create_pose_stamped(self, x, y, yaw):
        """
        Create a PoseStamped message for navigation goal.
        
        Args:
            x (float): X coordinate in map frame
            y (float): Y coordinate in map frame  
            yaw (float): Orientation in radians
            
        Returns:
            PoseStamped: Formatted pose for Nav2
        """
        pose = PoseStamped()
        
        # Set header information
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion for orientation
        # Formula for yaw-only rotation: q = [0, 0, sin(yaw/2), cos(yaw/2)]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def send_next_goal(self):
        """
        Send the next waypoint goal to Nav2.
        
        Cycles through the waypoint list continuously, sending each
        waypoint as a navigation goal to the Nav2 action server.
        """
        if not self.waypoints:
            self.get_logger().error('No waypoints defined!')
            return
            
        # Get current waypoint coordinates
        x, y, yaw = self.waypoints[self.current_waypoint_index]
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, yaw)
        
        self.get_logger().info(
            f'Sending goal {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad'
        )
        
        # Reset navigation completion flag
        self.navigation_complete = False
        
        # Send goal to Nav2 action server
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Move to next waypoint (cycle back to 0 when reaching the end)
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def goal_response_callback(self, future):
        """
        Handle the response from Nav2 action server.
        
        Args:
            future: Future object containing the goal response
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected by Nav2!')
            return
            
        self.get_logger().info('Navigation goal accepted by Nav2')
        
        # Get result when navigation completes
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from Nav2 during navigation.
        
        Args:
            feedback_msg: Feedback message from Nav2 with navigation progress
        """
        # Extract current pose and distance remaining from feedback
        current_pose = feedback_msg.feedback.current_pose.pose
        distance_remaining = feedback_msg.feedback.distance_remaining
        
        # Log navigation progress every few feedback messages
        if hasattr(self, '_feedback_counter'):
            self._feedback_counter += 1
        else:
            self._feedback_counter = 0
            
        # Log progress every 10th feedback message to avoid spam
        if self._feedback_counter % 10 == 0:
            self.get_logger().info(
                f'Navigation progress: {distance_remaining:.2f}m remaining, '
                f'current position: x={current_pose.position.x:.2f}, '
                f'y={current_pose.position.y:.2f}'
            )

    def get_result_callback(self, future):
        """
        Handle the final result from Nav2 navigation.
        
        Args:
            future: Future object containing the navigation result
        """
        result = future.result().result
        
        # Check if navigation was successful
        if result:
            self.get_logger().info('Navigation completed successfully!')
            self.navigation_complete = True
        else:
            self.get_logger().warn('Navigation failed! Will retry with next waypoint.')
            self.navigation_complete = True  # Continue to next waypoint anyway


def main(args=None):
    """
    Main function to initialize and run the waypoint navigator.
    
    Args:
        args: Command line arguments (unused)
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and run the waypoint navigator
        navigator = WaypointNavigator()
        
        print("\n" + "="*60)
        print("TurtleBot3 Autonomous Waypoint Navigator")
        print("="*60)
        print("IMPORTANT: Make sure Terminal 1 has Gazebo running!")
        print("The robot will navigate through small waypoints.")
        print("Press Ctrl+C to stop the navigation.")
        print("="*60 + "\n")
        
        # Spin the node to keep it running
        rclpy.spin(navigator)
        
    except KeyboardInterrupt:
        print("\nNavigation stopped by user.")
        
    except Exception as e:
        print(f"Error occurred: {e}")
        
    finally:
        # Clean shutdown
        if 'navigator' in locals():
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Waypoint Navigator shut down successfully.")


if __name__ == '__main__':
    main()