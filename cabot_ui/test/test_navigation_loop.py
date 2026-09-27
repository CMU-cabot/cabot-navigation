#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: MIT
"""Regression for a completion timer stranded between two ROS executors."""

import os
import threading
import time
import unittest

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from cabot_ui.navigation import Navigation


class NavigationLoopTest(unittest.TestCase):
    def test_arrival_loop_resumes_after_action_callback(self):
        if os.environ.get('ROS_DOMAIN_ID') != '93':
            self.skipTest('Run in isolated ROS_DOMAIN_ID=93')
        context = Context()
        rclpy.init(context=context)
        nav_node = Node('test_navigation_loop', context=context)
        action_node = Node('test_navigation_actions', context=context)
        nav_executor = MultiThreadedExecutor(num_threads=2, context=context)
        action_executor = SingleThreadedExecutor(context=context)
        nav_executor.add_node(nav_node)
        action_executor.add_node(action_node)
        group = MutuallyExclusiveCallbackGroup()
        entered = threading.Event()
        release = threading.Event()
        checked = threading.Event()

        def action_callback():
            action_timer.cancel()
            entered.set()
            release.wait(5)

        action_timer = action_node.create_timer(0.01, action_callback, callback_group=group)
        navigation = Navigation.__new__(Navigation)
        navigation._node = nav_node
        navigation._action_node = action_node
        navigation._logger = nav_node.get_logger()
        navigation.lock = threading.Lock()
        navigation._main_callback_group = group
        navigation._loop_handle = None
        navigation._check_loop = checked.set
        threads = [threading.Thread(target=e.spin) for e in (nav_executor, action_executor)]
        try:
            for thread in threads:
                thread.start()
            self.assertTrue(entered.wait(2), 'Action callback did not start')
            navigation._start_loop()
            # The timer becomes due while its callback group is occupied. An
            # unrelated executor gets no wakeup when that group is released.
            time.sleep(0.3)
            self.assertFalse(checked.is_set(), 'Action and arrival checks must remain serialized')
            release.set()
            self.assertTrue(checked.wait(2), 'Arrival loop did not resume after the action callback')
            timer = navigation._loop_handle
            navigation._start_loop()
            self.assertIs(navigation._loop_handle, timer)
        finally:
            release.set()
            nav_executor.shutdown(timeout_sec=2)
            action_executor.shutdown(timeout_sec=2)
            for thread in threads:
                thread.join(timeout=2)
            nav_node.destroy_node()
            action_node.destroy_node()
            rclpy.shutdown(context=context)


if __name__ == '__main__':
    unittest.main()
