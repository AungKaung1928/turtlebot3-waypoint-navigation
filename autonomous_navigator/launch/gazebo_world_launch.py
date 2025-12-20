#!/usr/bin/env python3

"""
TurtleBot3 Waypoint Navigator using Nav2
Sends navigation goals to Nav2 stack for autonomous waypoint navigation
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
    Manages waypoint list and sends navigation goals to Nav2
    """
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Action client for Nav2 navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server is ready!')
        
        # Waypoint list (x, y, yaw_radians)
        self.waypoints = [
            (0.5, 0.5, 0.0),
            (0.5, -0.5, -1.57),
            (-0.5, -0.5, 3.14),
            (-0.5, 0.5, 1.57),
            (0.0, 0.0, 0.0),
        ]
        
        self.current_waypoint_index = 0
        self.navigation_complete = False
        
        # Check navigation status every 2 seconds
        self.timer = self.create_timer(2.0, self.navigation_timer_callback)
        
        self.get_logger().info(f'Waypoint Navigator initialized with {len(self.waypoints)} waypoints')
        self.get_logger().info('MAKE SURE GAZEBO IS RUNNING IN TERMINAL 1!')
        
        # Start navigation
        self.send_next_goal()

    def navigation_timer_callback(self):
        """Check navigation status and send next waypoint when complete"""
        if self.navigation_complete:
            self.get_logger().info('Navigation to current waypoint completed!')
            time.sleep(1)
            self.send_next_goal()

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
        """Send next waypoint goal to Nav2"""
        if not self.waypoints:
            self.get_logger().error('No waypoints defined!')
            return
            
        # Get current waypoint
        x, y, yaw = self.waypoints[self.current_waypoint_index]
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, yaw)
        
        self.get_logger().info(
            f'Sending goal {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad'
        )
        
        self.navigation_complete = False
        
        # Send goal to Nav2
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Cycle to next waypoint
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def goal_response_callback(self, future):
        """Handle goal response from Nav2"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected by Nav2!')
            return
            
        self.get_logger().info('Navigation goal accepted by Nav2')
        
        # Get result when navigation completes
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback from Nav2"""
        current_pose = feedback_msg.feedback.current_pose.pose
        distance_remaining = feedback_msg.feedback.distance_remaining
        
        # Log progress periodically
        if hasattr(self, '_feedback_counter'):
            self._feedback_counter += 1
        else:
            self._feedback_counter = 0
            
        if self._feedback_counter % 10 == 0:
            self.get_logger().info(
                f'Navigation progress: {distance_remaining:.2f}m remaining, '
                f'current position: x={current_pose.position.x:.2f}, '
                f'y={current_pose.position.y:.2f}'
            )

    def get_result_callback(self, future):
        """Handle final navigation result"""
        result = future.result().result
        
        if result:
            self.get_logger().info('Navigation completed successfully!')
            self.navigation_complete = True
        else:
            self.get_logger().warn('Navigation failed! Will retry with next waypoint.')
            self.navigation_complete = True


def main(args=None):
    """Initialize and run waypoint navigator"""
    rclpy.init(args=args)
    
    try:
        navigator = WaypointNavigator()
        
        print("\n" + "="*60)
        print("TurtleBot3 Autonomous Waypoint Navigator")
        print("="*60)
        print("IMPORTANT: Make sure Terminal 1 has Gazebo running!")
        print("The robot will navigate through small waypoints.")
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