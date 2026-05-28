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
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_msgs.msg import NavSAT, NavSATSV


class GpsConverter(Node):
    def __init__(self, name):
        super().__init__(name)

        # Gazebo provides position and velocity only. These defaults add the
        # u-blox quality fields expected by localization as a high-quality
        # simulated GNSS receiver, not as physical receiver statistics.
        self.fix_covariance = self.declare_parameter('fix_covariance', 0.002).value
        self.navsat_num_svs = self.declare_parameter('navsat_num_svs', 20).value
        self.navsat_cno = self.declare_parameter('navsat_cno', 40).value
        self.navsat_elev = self.declare_parameter('navsat_elev', 60).value
        self.navsat_rate = self.declare_parameter('navsat_rate', 1.0).value

        self.fix_sub = self.create_subscription(
            NavSatFix,
            '/fix_in',
            self.fix_callback,
            10)
        self.velocity_sub = self.create_subscription(
            Vector3Stamped,
            '/velocity_in',
            self.velocity_callback,
            10)
        self.fix_pub = self.create_publisher(
            NavSatFix,
            '/fix_out',
            10)
        self.fix_velocity_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/fix_velocity_out',
            10)
        self.navsat_pub = self.create_publisher(
            NavSAT,
            '/navsat_out',
            10)

        self.navsat_timer = self.create_timer(1.0 / self.navsat_rate, self.publish_navsat)

    def fix_callback(self, msg):
        fix_msg = NavSatFix()
        fix_msg.header = msg.header
        # Treat simulator GNSS as a high-quality fix so the same quality gates
        # used on the robot can run in simulation.
        fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = msg.latitude
        fix_msg.longitude = msg.longitude
        fix_msg.altitude = msg.altitude
        fix_msg.position_covariance = [
            self.fix_covariance, 0.0, 0.0,
            0.0, self.fix_covariance, 0.0,
            0.0, 0.0, self.fix_covariance,
        ]
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.fix_pub.publish(fix_msg)
        self.get_logger().debug(f'Publishing fix: {fix_msg}')

    def velocity_callback(self, msg):
        twist_msg = TwistWithCovarianceStamped()

        twist_msg.header = msg.header

        twist_msg.twist.twist.linear.x = msg.vector.x
        twist_msg.twist.twist.linear.y = msg.vector.y
        twist_msg.twist.twist.linear.z = msg.vector.z

        twist_msg.twist.covariance = [0.0] * 36

        self.fix_velocity_pub.publish(twist_msg)
        self.get_logger().debug(f'Publishing velocity: {twist_msg}')

    def publish_navsat(self):
        navsat_msg = NavSAT()
        navsat_msg.i_tow = self.gps_time_of_week_ms()
        navsat_msg.version = 1
        navsat_msg.num_svs = self.navsat_num_svs
        navsat_msg.reserved0 = [0, 0]
        navsat_msg.sv = [self.create_navsat_sv(i) for i in range(self.navsat_num_svs)]

        self.navsat_pub.publish(navsat_msg)
        self.get_logger().debug(f'Publishing navsat: {navsat_msg}')

    def gps_time_of_week_ms(self):
        return int((self.get_clock().now().nanoseconds // 1000000) % 604800000)

    def create_navsat_sv(self, index):
        sv = NavSATSV()
        sv.gnss_id = 0
        sv.sv_id = index + 1
        sv.cno = self.navsat_cno
        sv.elev = self.navsat_elev
        sv.azim = int(index * 360 / max(self.navsat_num_svs, 1))
        sv.pr_res = 0
        # Mark synthetic satellites as usable with carrier lock and time sync.
        sv.flags = NavSATSV.QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3 | NavSATSV.FLAGS_SV_USED
        return sv


def main():
    rclpy.init()
    converter = GpsConverter('gps_converter')
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
