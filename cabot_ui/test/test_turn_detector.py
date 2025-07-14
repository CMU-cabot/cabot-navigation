#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
"""Test cabot_ui.turn_detector module"""

import os
import unittest
import yaml

import cabot_ui.geoutil
from cabot_ui.turn_detector import TurnDetector
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import rclpy
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
import time


def parsePath(path_data):
    path_msg = Path()
    # Populate header
    path_msg.header = Header(
        stamp=Time(
            sec=path_data['header']['stamp']['sec'],
            nanosec=path_data['header']['stamp']['nanosec']
        ),
        frame_id=path_data['header']['frame_id']
    )
    path_msg.poses = [
        PoseStamped(
            header=Header(
                stamp=Time(
                    sec=pose['header']['stamp']['sec'],
                    nanosec=pose['header']['stamp']['nanosec']
                ),
                frame_id=pose['header']['frame_id']
            ),
            pose=Pose(
                position=Point(
                    x=pose['pose']['position']['x'],
                    y=pose['pose']['position']['y'],
                    z=pose['pose']['position']['z']
                ),
                orientation=Quaternion(
                    x=pose['pose']['orientation']['x'],
                    y=pose['pose']['orientation']['y'],
                    z=pose['pose']['orientation']['z'],
                    w=pose['pose']['orientation']['w']
                )
            )
        ) for pose in path_data['poses']
    ]
    return path_msg


class TestEvent(unittest.TestCase):
    """Test class"""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.node.Node("test_node")
        CaBotRclpyUtil.initialize(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()
        return super().tearDown()

    def _test_basic(self):
        current_pose = cabot_ui.geoutil.Pose(x=0, y=0, r=0)
        for i in range(0, 1):
            path = Path()
            for j in range(0, i):
                pose = PoseStamped()
                pose.pose.position.x = j * 0.1
                pose.pose.position.y = j * 0.1
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            _ = TurnDetector.detects(path, current_pose=current_pose)

    def test_plan_msg1(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))

        with open(dir_path+"/data/plan_msg1.yaml") as plan_file:
            path_data = yaml.safe_load(plan_file)
            path = parsePath(path_data)
            pose = path.poses[0]
            current_pose = cabot_ui.geoutil.Pose(x=pose.pose.position.x, y=pose.pose.position.y, r=0)

            start = time.time()
            for i in range(100):
                TurnDetector.detects(path, current_pose=current_pose)
            end = time.time()
            print(f"Total time taken for 100 iterations: {(end-start)/100:.4f}")
