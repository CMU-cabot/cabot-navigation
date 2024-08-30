#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2024  IBM Corporation
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

import csv

import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import traceback


class TrackedPoseListener:
    def __init__(self, node: Node, throttle_duration_sec: float = 10.0):
        self.node = node
        self.data_list = []
        self.throttle_duration_sec = throttle_duration_sec
        self.tracked_pose_subscriber = self.node.create_subscription(PoseStamped, "/tracked_pose", self.tracked_pose_callback, 10)

    def tracked_pose_callback(self, msg: PoseStamped):
        timestamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)*1.0e-9
        frame_id = msg.header.frame_id
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        data = [timestamp, frame_id, x, y, z, qx, qy, qz, qw]
        self.data_list.append(data)

        freq = self.data_list
        if len(self.data_list) > 1:
            t0 = self.data_list[0][0]
            t1 = self.data_list[-1][0]
            freq = len(self.data_list)/(t1 - t0)
            self.node.get_logger().info(F"tracked_pose messages received at {freq:.2f} Hz (total count=  {len(self.data_list)}). current tracked_pose = {data}", throttle_duration_sec=self.throttle_duration_sec)


def main():
    rclpy.init()
    node = rclpy.create_node("tracked_pose_listener")
    output = node.declare_parameter("output", "").value
    tracked_pose_listner = TrackedPoseListener(node)

    def shutdown_hook():
        if output is not None and 0 < len(tracked_pose_listner.data_list):
            print("wirinting to output")
            with open(output, "w") as f:
                writer = csv.writer(f, lineterminator="\n")
                writer.writerow(["timestamp", "frame_id", "x", "y", "z", "qx", "qy", "qz", "qw"])
                writer.writerows(tracked_pose_listner.data_list)

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        traceback.print_exc()
        shutdown_hook()


if __name__ == "__main__":
    main()
