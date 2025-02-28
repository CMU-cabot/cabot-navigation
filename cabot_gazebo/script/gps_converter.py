#!/usr/bin/env python3
# ******************************************************************************
#  Copyright (c) 2025  Miraikan - The National Museum of Emerging Science and Innovation
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# ******************************************************************************

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped


class Vector3StampedToTwistCovarianceStamped(Node):
    def __init__(self, name):
        super().__init__(name)
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/in',
            self.callback,
            10)
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/out',
            10)

    def callback(self, msg):
        twist_msg = TwistWithCovarianceStamped()

        twist_msg.header = msg.header

        twist_msg.twist.twist.linear.x = msg.vector.x
        twist_msg.twist.twist.linear.y = msg.vector.y
        twist_msg.twist.twist.linear.z = msg.vector.z

        twist_msg.twist.covariance = [0.0] * 36

        self.publisher.publish(twist_msg)
        self.get_logger().info(f'Publishing: {twist_msg}')


def main():
    rclpy.init()
    converter = Vector3StampedToTwistCovarianceStamped('vector3_stamped_to_twist_covariance_stamped')
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
