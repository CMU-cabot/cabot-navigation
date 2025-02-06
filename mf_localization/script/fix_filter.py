#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022  IBM Corporation
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

import numpy as np
import rclpy

from sensor_msgs.msg import NavSatFix


class FixFilterNode:
    def __init__(self, node, status_threshold, stdev_threshold, overwrite_time=True):
        self._node = node
        self._clock = node.get_clock()
        self._status_threshold = status_threshold
        self._stdev_threshold = stdev_threshold
        self._overwrite_time = overwrite_time
        self._fix_sub = self._node.create_subscription(NavSatFix, "fix", self.fix_callback, 10)
        self._fix_pub = self._node.create_publisher(NavSatFix, "fix_filtered", 10)

    def fix_callback(self, msg: NavSatFix):
        navsat_status = msg.status
        status = navsat_status.status
        position_covariance = np.reshape(msg.position_covariance, (3, 3))
        stdev = np.sqrt(position_covariance[0, 0])

        if self._status_threshold <= status and stdev <= self._stdev_threshold:
            if self._overwrite_time:
                msg.header.stamp = self._clock.now().to_msg()
            msg.altitude = 0.0
            self._fix_pub.publish(msg)


def main():
    rclpy.init()
    node = rclpy.create_node("fix_filter")
    status_threshold = node.declare_parameter("status_threshold", 2).value
    stdev_threshold = node.declare_parameter("stdev_threshold", 0.1).value
    overwrite_time = node.declare_parameter("overwrite_time", False).value

    FixFilterNode(node, status_threshold, stdev_threshold, overwrite_time)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
