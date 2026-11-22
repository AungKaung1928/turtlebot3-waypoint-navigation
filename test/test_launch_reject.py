"""Launch test: rejected goals. The stand-in navigate_to_pose server rejects every goal; the node must
count each as a failure and move on through the waypoints in order (no stall, no resend).

Setup as in test_launch_nav_goals.py:

The test process serves navigate_to_pose itself: goal 1 succeeds, goal 2 is aborted.
It checks the goals arrive in waypoint order in the map frame, and that the node
reports the aborted goal as a failure.
Run: launch_test test/test_launch_nav_goals.py (after sourcing the workspace).
"""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor



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

    def test_rejected_goals_move_on_in_order(self, proc_output, node):
        fake = rclpy.create_node('fake_nav2')
        goals = []

        def on_goal(request):
            p = request.pose.pose.position
            goals.append((round(p.x, 3), round(p.y, 3)))
            return GoalResponse.REJECT

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose',
                              lambda gh: NavigateToPose.Result(), goal_callback=on_goal)
        executor = MultiThreadedExecutor()
        executor.add_node(fake)
        try:
            start = time.monotonic()  # one goal per ~5 s: 3 s status timer + 2 s pause
            while len(goals) < 3 and time.monotonic() - start < 45.0:
                executor.spin_once(timeout_sec=0.1)
        finally:
            server.destroy()
            executor.shutdown()
            fake.destroy_node()
        self.assertEqual(goals[:3], [(2.0, 0.5), (2.0, -1.5), (0.5, -2.0)])  # waypoint_navigator.py, self.waypoints
        proc_output.assertWaitFor('rejected', process=node, timeout=5)
