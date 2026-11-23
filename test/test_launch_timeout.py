"""Launch test: a goal that times out must not cost the next waypoint.

The stand-in navigate_to_pose server never finishes goal 1, so the node's goal_timeout (4 s here)
moves it on to goal 2. Like Nav2, the server then aborts goal 1. That late abort belongs to a
superseded goal: the node must keep waiting for goal 2 (succeeds after 3.5 s) and only then send
goal 3. Before the fix the late abort marked the current goal done and goal 3 arrived while goal 2
was still running.
Run: launch_test test/test_launch_timeout.py (after sourcing the workspace).
"""
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='autonomous_navigator', executable='waypoint_navigator', output='screen',
        parameters=[{'goal_timeout': 4.0}])
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest(), node]), {'node': node}


class TestTimeout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_late_abort_does_not_skip_a_waypoint(self, proc_output, node):
        fake = rclpy.create_node('fake_nav2')
        arrived = []        # time each goal arrived, in order
        goal2_done = []     # time goal 2 succeeded
        stop = threading.Event()

        def execute(goal_handle):
            i = len(arrived)
            arrived.append(time.monotonic())
            if i == 0:      # hangs until superseded, then aborted as Nav2 does on preemption
                while len(arrived) < 2 and not stop.is_set():
                    time.sleep(0.05)
                goal_handle.abort()
            elif i == 1:
                time.sleep(3.5)
                goal_handle.succeed()
                goal2_done.append(time.monotonic())
            else:
                while not stop.is_set():
                    time.sleep(0.05)
                goal_handle.abort()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute,
                              callback_group=ReentrantCallbackGroup())
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(fake)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()
        try:
            start = time.monotonic()
            while len(arrived) < 3 and time.monotonic() - start < 40.0:
                time.sleep(0.1)
        finally:
            stop.set()
            time.sleep(0.3)
            executor.shutdown()
            server.destroy()
            fake.destroy_node()
        self.assertEqual(len(arrived), 3, 'goal 3 never arrived')
        self.assertTrue(goal2_done, 'goal 2 never finished')
        self.assertGreater(arrived[2], goal2_done[0], 'goal 3 sent while goal 2 was still running')
