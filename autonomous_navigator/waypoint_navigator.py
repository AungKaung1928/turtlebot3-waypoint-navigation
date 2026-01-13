#!/usr/bin/env python3
"""
TurtleBot3 Waypoint Navigator using Nav2
Sends navigation goals to Nav2 stack for autonomous waypoint navigation
Continuous loop through predefined waypoints
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
    Manages waypoint list and sends navigation goals to Nav2 in continuous loop
    """
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Action client for Nav2 navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Service client for clearing costmap
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        
        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server is ready!')
        
        # Waypoint list (x, y, yaw_radians)
        self.waypoints = [
            (2.0, 0.5, 0.0),
            (2.0, -1.5, -1.57),
            (0.5, -2.0, 3.14),
            (-1.5, -1.5, 2.36),
            (-2.0, 0.5, 1.57),
            (-1.0, 2.0, 0.79),
            (1.0, 2.0, 0.0),
            (0.0, 0.0, 0.0),
        ]
        
        self.current_waypoint_index = 0
        self.navigation_complete = False
        self.failed_attempts = 0
        self.max_failed_attempts = 2
        self.last_goal_time = None
        self.goal_timeout = 60.0
        self.loop_count = 0
        
        # Check navigation status every 3 seconds
        self.timer = self.create_timer(3.0, self.navigation_timer_callback)
        
        self.get_logger().info(f'Waypoint Navigator initialized with {len(self.waypoints)} waypoints')
        self.get_logger().info('INFINITE LOOP MODE - Robot will never stop')
        
        # Initial delay before starting
        time.sleep(2.0)
        self.send_next_goal()

    def navigation_timer_callback(self):
        """Check navigation status and handle timeouts"""
        # Check for timeout
        if (self.last_goal_time and 
            time.time() - self.last_goal_time > self.goal_timeout):
            self.get_logger().warn('Goal timeout! Clearing costmap and trying next waypoint.')
            self.clear_costmap()
            self.navigation_complete = True
            self.failed_attempts += 1
            
        if self.navigation_complete:
            self.get_logger().info('Navigation to current waypoint completed!')
            
            # Skip waypoint after multiple failures
            if self.failed_attempts >= self.max_failed_attempts:
                self.get_logger().warn(f'Skipping waypoint after {self.failed_attempts} failures')
                self.failed_attempts = 0
            
            time.sleep(2)
            self.send_next_goal()

    def clear_costmap(self):
        """Clear local costmap for recovery"""
        try:
            if self._clear_costmap_client.wait_for_service(timeout_sec=2.0):
                request = ClearEntireCostmap.Request()
                future = self._clear_costmap_client.call_async(request)
                self.get_logger().info('Clearing local costmap for recovery')
        except Exception as e:
            self.get_logger().warn(f'Failed to clear costmap: {e}')

    def create_pose_stamped(self, x, y, yaw):
        """
        Create PoseStamped message for navigation goal
        
        Args:
            x: X coordinate in map frame
            y: Y coordinate in map frame
            yaw: Orientation in radians
            
        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def send_next_goal(self):
        """
        Send next waypoint goal to Nav2
        Cycles through waypoints continuously, restarting from beginning after completion
        """
        # Reset to start after completing all waypoints
        if self.current_waypoint_index >= len(self.waypoints):
            self.loop_count += 1
            self.current_waypoint_index = 0
            self.get_logger().info(f'=== LOOP {self.loop_count} COMPLETE - RESTARTING ===')
            
        # Get current waypoint
        x, y, yaw = self.waypoints[self.current_waypoint_index]
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, yaw)
        
        self.get_logger().info(
            f'→ Goal {self.current_waypoint_index + 1}/{len(self.waypoints)} [Loop {self.loop_count}]: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad'
        )
        
        # Reset flags and set timeout
        self.navigation_complete = False
        self.last_goal_time = time.time()
        
        # Send goal to Nav2
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Advance to next waypoint
        self.current_waypoint_index += 1

    def goal_response_callback(self, future):
        """Handle goal response from Nav2"""
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
        """Handle navigation feedback from Nav2"""
        current_pose = feedback_msg.feedback.current_pose.pose
        distance_remaining = feedback_msg.feedback.distance_remaining
        navigation_time = feedback_msg.feedback.navigation_time
        
        # Log progress periodically
        if hasattr(self, '_feedback_counter'):
            self._feedback_counter += 1
        else:
            self._feedback_counter = 0
            
        if self._feedback_counter % 15 == 0:
            self.get_logger().info(
                f'Navigation progress: {distance_remaining:.2f}m remaining, '
                f'time: {navigation_time.sec}s, '
                f'current position: x={current_pose.position.x:.2f}, '
                f'y={current_pose.position.y:.2f}'
            )

    def get_result_callback(self, future):
        """Handle final navigation result"""
        result = future.result().result
        
        if result:
            self.get_logger().info('✓ Navigation completed successfully!')
            self.navigation_complete = True
            self.failed_attempts = 0
        else:
            self.get_logger().warn('✗ Navigation failed! Will try next waypoint.')
            self.navigation_complete = True
            self.failed_attempts += 1


def main(args=None):
    """Initialize and run waypoint navigator"""
    rclpy.init(args=args)
    
    try:
        navigator = WaypointNavigator()
        
        print("\n" + "="*60)
        print("TurtleBot3 Autonomous Waypoint Navigator - INFINITE LOOP")
        print("="*60)
        print("8 waypoints | Speed: 0.8 m/s | NEVER STOPS")
        print("Robot will loop through waypoints forever")
        print("Press Ctrl+C to stop the navigation.")
        print("="*60 + "\n")
        
        rclpy.spin(navigator)
        
    except KeyboardInterrupt:
        print("\nNavigation stopped by user.")
        
    except Exception as e:
        print(f"Error occurred: {e}")
        
    finally:
        if 'navigator' in locals():
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Waypoint Navigator shut down successfully.")


if __name__ == '__main__':
    main()