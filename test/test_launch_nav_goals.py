"""
Launch test: the installed waypoint_navigator against a stand-in Nav2 action server.

The test process serves navigate_to_pose itself: goal 1 succeeds, goal 2 is aborted.
It checks the goals arrive in waypoint order in the map frame, and that the node
reports the aborted goal as a failure.
Run: launch_test test/test_launch_nav_goals.py (after sourcing the workspace).
"""
import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

FIRST_WAYPOINTS = [(2.0, 0.5, 0.0), (2.0, -1.5, -1.57)]  # waypoint_navigator.py, self.waypoints


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='autonomous_navigator', executable='waypoint_navigator', output='screen')
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest(), node]), {'node': node}


class TestNavGoals(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_goals_in_order_and_abort_reported(self, proc_output, node):
        fake = rclpy.create_node('fake_nav2')
        goals = []

        def execute(goal_handle):
            goals.append(goal_handle.request.pose)
            if len(goals) == 2:
                goal_handle.abort()
            else:
                goal_handle.succeed()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute)
        executor = MultiThreadedExecutor()
        executor.add_node(fake)
        try:
            start = time.monotonic()
            while len(goals) < 2 and time.monotonic() - start < 40.0:
                executor.spin_once(timeout_sec=0.1)
            self.assertEqual(len(goals), 2, 'the node did not send two goals within 40 s')
            for pose, (x, y, yaw) in zip(goals, FIRST_WAYPOINTS):
                self.assertEqual(pose.header.frame_id, 'map')
                self.assertAlmostEqual(pose.pose.position.x, x)
                self.assertAlmostEqual(pose.pose.position.y, y)
                self.assertAlmostEqual(pose.pose.orientation.z, math.sin(yaw / 2.0), places=6)
                self.assertAlmostEqual(pose.pose.orientation.w, math.cos(yaw / 2.0), places=6)
            # Keep serving while the node handles the aborted result.
            end = time.monotonic() + 5.0
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.1)
        finally:
            server.destroy()
            executor.shutdown()
            fake.destroy_node()
        proc_output.assertWaitFor('Navigation failed', process=node, timeout=5)
