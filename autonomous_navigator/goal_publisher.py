#!/usr/bin/env python3

"""
Simple Goal Publisher for Nav2
=============================

Alternative goal publisher that sends single navigation goals.
Useful for testing and manual waypoint navigation.

Author: Your Name
License: MIT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class GoalPublisher(Node):
    """Simple goal publisher for testing Nav2 navigation."""
    
    def __init__(self):
        """Initialize the goal publisher node."""
        super().__init__('goal_publisher')
        
        # Create publisher for Nav2 goals
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Timer to publish goals periodically
        self.timer = self.create_timer(10.0, self.publish_goal)
        
        # Test goal coordinates
        self.test_goals = [
            (2.0, 1.0, 0.0),
            (1.0, -1.0, 1.57),
            (0.0, 0.0, 0.0)
        ]
        
        self.current_goal_index = 0
        
        self.get_logger().info('Goal Publisher initialized')

    def publish_goal(self):
        """Publish a test navigation goal."""
        if not self.test_goals:
            return
            
        x, y, yaw = self.test_goals[self.current_goal_index]
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_publisher.publish(goal)
        
        self.get_logger().info(f'Published goal: x={x}, y={y}, yaw={yaw}')
        
        # Move to next goal
        self.current_goal_index = (self.current_goal_index + 1) % len(self.test_goals)


def main(args=None):
    """Main function for goal publisher."""
    rclpy.init(args=args)
    
    try:
        publisher = GoalPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()