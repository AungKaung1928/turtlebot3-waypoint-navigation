#!/usr/bin/env python3
"""
TurtleBot3 Waypoint Navigator using Nav2
=====================================
This node sends navigation goals to Nav2 stack for autonomous waypoint navigation.
The robot will continuously cycle through predefined waypoints while avoiding obstacles.
Fixed version with better waypoints and recovery mechanisms.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav2_msgs.srv import ClearEntireCostmap
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
        
        # Create service client for clearing costmap
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        
        # Wait for action server to be available
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server is ready!')
        
        # Define waypoints with safer distances from obstacles for TurtleBot3 world
        # These coordinates are designed to maintain safe clearance from obstacles
        # Format: (x, y, yaw_radians)
        self.waypoints = [
            (1.8, 0.0, 0.0),      # Point 1: Move forward (safer distance from center obstacles)
            (1.8, 1.8, 1.57),     # Point 2: Move up, face north (safer corner position)
            (0.0, 1.8, 3.14),     # Point 3: Move left, face west (safe from top obstacles)
            (-1.8, 1.8, 3.14),    # Point 4: Continue left (safe distance)
            (-1.8, 0.0, -1.57),   # Point 5: Move down, face south (safe from side obstacles)
            (-1.8, -1.8, -1.57),  # Point 6: Continue down (safe corner position)
            (0.0, -1.8, 0.0),     # Point 7: Move right, face east (safe from bottom obstacles)
            (1.8, -1.8, 0.0),     # Point 8: Continue right (safe distance)
            (1.0, 0.0, 1.57),     # Point 9: Move to intermediate safe position
            (0.0, 0.0, 0.0),      # Point 10: Return to origin, face east
        ]
        
        self.current_waypoint_index = 0
        self.navigation_complete = False
        self.failed_attempts = 0
        self.max_failed_attempts = 3
        self.last_goal_time = None
        self.goal_timeout = 60.0  # 60 seconds timeout for each goal
        
        # Timer to check navigation status and send next waypoint
        self.timer = self.create_timer(3.0, self.navigation_timer_callback)
        
        self.get_logger().info(f'Waypoint Navigator initialized with {len(self.waypoints)} waypoints')
        self.get_logger().info('Using safer waypoints with increased clearance from obstacles')
        
        # Wait a bit before starting navigation
        time.sleep(2.0)
        self.send_next_goal()

    def navigation_timer_callback(self):
        """
        Timer callback to check navigation status and handle timeouts.
        """
        # Check for timeout
        if (self.last_goal_time and 
            time.time() - self.last_goal_time > self.goal_timeout):
            self.get_logger().warn('Goal timeout! Clearing costmap and trying next waypoint.')
            self.clear_costmap()
            self.navigation_complete = True
            self.failed_attempts += 1
            
        if self.navigation_complete:
            self.get_logger().info('Navigation to current waypoint completed!')
            
            # If too many failures, skip problematic waypoint
            if self.failed_attempts >= self.max_failed_attempts:
                self.get_logger().warn(f'Skipping waypoint {self.current_waypoint_index} after {self.failed_attempts} failures')
                self.failed_attempts = 0
            
            time.sleep(2)  # Brief pause between waypoints
            self.send_next_goal()

    def clear_costmap(self):
        """Clear the local costmap to help with recovery."""
        try:
            if self._clear_costmap_client.wait_for_service(timeout_sec=2.0):
                request = ClearEntireCostmap.Request()
                future = self._clear_costmap_client.call_async(request)
                self.get_logger().info('Clearing local costmap for recovery')
        except Exception as e:
            self.get_logger().warn(f'Failed to clear costmap: {e}')

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
        
        # Reset navigation completion flag and set timeout
        self.navigation_complete = False
        self.last_goal_time = time.time()
        
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
            self.navigation_complete = True
            self.failed_attempts += 1
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
        navigation_time = feedback_msg.feedback.navigation_time
        
        # Log navigation progress every few feedback messages
        if hasattr(self, '_feedback_counter'):
            self._feedback_counter += 1
        else:
            self._feedback_counter = 0
            
        # Log progress every 15th feedback message to avoid spam
        if self._feedback_counter % 15 == 0:
            self.get_logger().info(
                f'Navigation progress: {distance_remaining:.2f}m remaining, '
                f'time: {navigation_time.sec}s, '
                f'current position: x={current_pose.position.x:.2f}, '
                f'y={current_pose.position.y:.2f}'
            )
            
        # Check if robot seems stuck (very low progress for extended time)
        if (hasattr(self, '_last_distance') and 
            abs(self._last_distance - distance_remaining) < 0.05 and 
            navigation_time.sec > 30):
            self.get_logger().warn('Robot may be stuck, clearing costmap')
            self.clear_costmap()
            
        self._last_distance = distance_remaining

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
            self.failed_attempts = 0  # Reset failure counter on success
        else:
            self.get_logger().warn('Navigation failed! Will try next waypoint.')
            self.navigation_complete = True
            self.failed_attempts += 1

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
        print("TurtleBot3 Autonomous Waypoint Navigator - FIXED VERSION")
        print("="*60)
        print("The robot will navigate through larger waypoints to avoid obstacles.")
        print("Improved recovery mechanisms and timeout handling included.")
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
        rclpy.shutdown()
        print("Waypoint Navigator shut down successfully.")

if __name__ == '__main__':
    main()