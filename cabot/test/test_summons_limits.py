#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: MIT
"""Exercise the real touch and downstream speed nodes in isolated ROS domain 93."""
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import Float32, Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import yaml


class SummonsLimitsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '93':
            raise unittest.SkipTest('Use isolated ROS_DOMAIN_ID=93')
        from ament_index_python.packages import get_package_prefix
        binary_dir = Path(os.environ.get('CABOT_TEST_BIN_DIR', Path(get_package_prefix('cabot')) / 'lib/cabot'))
        cls.temp = tempfile.TemporaryDirectory(prefix='summons-limits-')
        params = {'speed_control_node': {'ros__parameters': {
            'cmd_vel_input': '/test_velocity', 'cmd_vel_output': '/test_limited_velocity',
            'speed_input': ['/test_user_speed', '/test_obstacle_speed', '/touch_speed_switched'],
            'speed_limit': [2.0, 2.0, 0.0], 'speed_timeout': [-1.0, -1.0, 0.5],
            'complete_stop': [False, True, True], 'configurable': [True, False, False],
        }}}
        param_path = Path(cls.temp.name) / 'limits.yaml'
        param_path.write_text(yaml.safe_dump(params))
        cls.logs = []
        cls.processes = []
        for binary in ['touch_speed_control_node', 'speed_control_node']:
            log = open(Path(cls.temp.name) / (binary + '.log'), 'w+')
            cls.logs.append(log)
            cls.processes.append(subprocess.Popen(
                [str(binary_dir / binary), '--ros-args', '--params-file', str(param_path)],
                stdout=log, stderr=subprocess.STDOUT))
        rclpy.init()
        cls.node = Node('test_summons_speed_limits')
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.touch = cls.node.create_publisher(Int16, '/touch', 10)
        cls.user = cls.node.create_publisher(Float32, '/test_user_speed', transient)
        cls.obstacle = cls.node.create_publisher(Float32, '/test_obstacle_speed', transient)
        cls.velocity = cls.node.create_publisher(Twist, '/test_velocity', 10)
        cls.latest = None
        cls.node.create_subscription(Twist, '/test_limited_velocity', cls.output, 10)
        cls.mode = cls.node.create_client(SetBool, '/set_touch_speed_active_mode')
        cls.enable_user = cls.node.create_client(SetBool, '/test_user_speed_enabled')
        cls.parameters = cls.node.create_client(SetParametersAtomically, '/touch_speed_control_node/set_parameters_atomically')
        for client in [cls.mode, cls.enable_user, cls.parameters]:
            if not client.wait_for_service(timeout_sec=15):
                cls.tearDownClass()
                raise RuntimeError('Test node did not start: ' + client.srv_name)

    @classmethod
    def output(cls, msg):
        cls.latest = (time.monotonic(), msg.linear.x, msg.angular.z)

    @classmethod
    def tearDownClass(cls):
        for process in cls.processes:
            process.terminate()
        for process in cls.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in cls.logs:
            log.seek(0)
            text = log.read()
            if any(p.returncode not in (0, -15) for p in cls.processes):
                print(text)
            log.close()
        if hasattr(cls, 'node'):
            cls.node.destroy_node()
            rclpy.shutdown()
        cls.temp.cleanup()

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)
        self.assertTrue(future.done())
        return future.result()

    def set_limit(self, value):
        parameter = Parameter('touch_speed_max_speed_inactive', value=value).to_parameter_msg()
        return self.call(self.parameters, SetParametersAtomically.Request(parameters=[parameter])).result

    def setUp(self):
        self.assertTrue(self.call(self.mode, SetBool.Request(data=False)).success)
        self.assertTrue(self.call(self.enable_user, SetBool.Request(data=True)).success)
        self.assertTrue(self.set_limit(1.0).successful)

    def expect_speed(self, expected, user=2.0, obstacle=2.0, touch=0):
        start = time.monotonic()
        while time.monotonic() - start < 4:
            if touch is not None:
                self.touch.publish(Int16(data=touch))
            self.user.publish(Float32(data=user))
            self.obstacle.publish(Float32(data=obstacle))
            cmd = Twist()
            cmd.linear.x = 2.0
            cmd.angular.z = 0.4
            self.velocity.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if (self.latest and self.latest[0] - start > 0.7
                    and abs(self.latest[1] - expected) < 1e-5):
                if touch == 1 or touch is None:
                    self.assertAlmostEqual(self.latest[2], 0.0)
                return
        self.fail(f'Expected {expected} m/s, last output={self.latest}')

    def test_summons_ceiling_is_one_meter_per_second(self):
        self.expect_speed(1.0)

    def test_user_speed_remains_a_lower_ceiling(self):
        self.expect_speed(0.4, user=0.4)
        self.expect_speed(0.8, user=0.8)

    def test_obstacles_can_lower_the_ceiling_further(self):
        self.expect_speed(0.25, user=0.8, obstacle=0.25)

    def test_touch_stops_translation_and_rotation(self):
        self.expect_speed(1.0)
        self.expect_speed(0.0, touch=1)

    def test_missing_touch_messages_stop_motion(self):
        self.expect_speed(1.0)
        self.expect_speed(0.0, touch=None)

    def test_limit_updates_and_invalid_values_are_rejected(self):
        self.assertTrue(self.set_limit(0.6).successful)
        self.expect_speed(0.6)
        self.assertFalse(self.set_limit(-1.0).successful)
        self.assertFalse(self.set_limit(float('nan')).successful)
        self.expect_speed(0.6)
        self.expect_speed(0.0, touch=1)


if __name__ == '__main__':
    unittest.main()
