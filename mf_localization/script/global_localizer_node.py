#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright (c) 2025  IBM Corporation
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

import signal
import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from mf_localization_msgs.msg import MFGlobalPosition2

from mf_localization import geoutil

from mf_localization.global_localizer import init_localizer
from mf_localization.global_localizer import GlobalPose
from mf_localization.global_localizer import LocalizerCallback
from mf_localization.global_localizer import LocalizerInterface
from mf_localization.global_localizer import GLOBAL_LOCALIZER_CONFIG_KEY


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)


class GlobalLocalizerNode:
    def __init__(self, node: Node, map_config, localizer: LocalizerInterface):
        self.node = node
        self.logger = node.get_logger()

        self.localizer: LocalizerInterface = localizer

        # create subscribers
        sensor_qos: QoSProfile = qos_profile_sensor_data
        self.points2_sub = node.create_subscription(PointCloud2, "points2", self.points2_callback, qos_profile=sensor_qos)
        self.image1_sub = node.create_subscription(CompressedImage, "compressed_image1", self.image1_callback, qos_profile=sensor_qos)  # front
        self.image2_sub = node.create_subscription(CompressedImage, "compressed_image2", self.image2_callback, qos_profile=sensor_qos)  # right"
        self.image3_sub = node.create_subscription(CompressedImage, "compressed_image3", self.image3_callback, qos_profile=sensor_qos)  # left
        self.imu_sub = node.create_subscription(Imu, "imu", self.imu_callback, qos_profile=sensor_qos)
        self.wifi_sub = node.create_subscription(String, "wifi", self.wifi_callback, qos_profile=sensor_qos)
        self.beacons_sub = node.create_subscription(String, "beacons", self.beacons_callback, qos_profile=sensor_qos)
        self.gnss_fix_sub = node.create_subscription(NavSatFix, "fix", self.gnss_fix_callback, qos_profile=sensor_qos)
        self.gnss_fix_velocity_sub = node.create_subscription(TwistWithCovarianceStamped, "fix_velocity", self.gnss_fix_velocity_callback, qos_profile=sensor_qos)
        self.odom_sub = node.create_subscription(Odometry, "odom", self.odom_callback, qos_profile=sensor_qos)

        # register callback to recieve global pose
        self.register_callback(self.global_pose_callback)

        # publisher
        self.pose_pub = node.create_publisher(PoseWithCovarianceStamped, "~/local_pose", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.mf_global_pose_pub = node.create_publisher(MFGlobalPosition2, "~/global_pose", 10, callback_group=MutuallyExclusiveCallbackGroup())

        # create service
        self.set_enabled_srv = node.create_service(SetBool, "~/set_enabled", self.set_enabled_callback)

        anchor_dict = map_config.get("anchor")
        global_anchor = geoutil.Anchor(lat=anchor_dict["latitude"],
                                       lng=anchor_dict["longitude"],
                                       rotate=anchor_dict["rotate"]
                                       )
        self.global_anchor = global_anchor

    def points2_callback(self, points2: PointCloud2):
        self.localizer.points2_callback(points2)

    def image1_callback(self, image: CompressedImage):
        self.localizer.image1_callback(image)

    def image2_callback(self, image: CompressedImage):
        self.localizer.image2_callback(image)

    def image3_callback(self, image: CompressedImage):
        self.localizer.image3_callback(image)

    def imu_callback(self, imu: Imu):
        self.localizer.imu_callback(imu)

    def wifi_callback(self, wifi: String):
        self.localizer.wifi_callback(wifi)

    def beacons_callback(self, beacons: String):
        self.localizer.beacons_callback(beacons)

    def gnss_fix_callback(self, fix: NavSatFix):
        self.localizer.gnss_fix_callback(fix)

    def gnss_fix_velocity_callback(self, fix_velocity: TwistWithCovarianceStamped):
        self.localizer.gnss_fix_velocity_callback(fix_velocity)

    def odom_callback(self, odom: Odometry):
        self.localizer.odom_callback(odom)

    def register_callback(self, callback: LocalizerCallback):
        self.localizer.register_callback(callback)

    def global_pose_callback(self, global_pose: GlobalPose):
        self.logger.info(F"global_pose_callback: global_pose = {global_pose}")

        now = self.node.get_clock().now().to_msg()
        frame_id = "map_global"
        # mf localization message
        mf_pos: MFGlobalPosition2 = global_pose.to_mf_global_position2(now, frame_id)
        self.mf_global_pose_pub.publish(mf_pos)

        # visualization message
        pose_with_cov: PoseWithCovarianceStamped = global_pose.to_pose_with_covariance_stamped(self.global_anchor, now, frame_id)
        self.pose_pub.publish(pose_with_cov)

    def set_enabled(self, active: bool):
        self.localizer.set_enabled(active)

    def set_enabled_callback(self, request: SetBool.Request, response: SetBool.Response):
        if bool(request.data):  # enable
            self.logger.info("set_enabled_callback: enable localization.")
            self.set_enabled(True)
            response.success = True
        else:  # disable
            self.logger.info("set_enabled_callback: disable localization.")
            self.set_enabled(False)
            response.success = True
        return response


def main():
    rclpy.init()
    node = Node("global_localizer")
    logger = node.get_logger()

    node.declare_parameter('map_config_file', "")
    map_config_file = node.get_parameter('map_config_file').value
    if map_config_file == "":
        logger.error("map_config_file is not specified.")
        return

    map_config = None
    with open(map_config_file) as stream:
        map_config = yaml.safe_load(stream)

    # log
    logger.info(F"map_config.keys={map_config.keys()}")

    global_localizer_config = map_config.get(GLOBAL_LOCALIZER_CONFIG_KEY)
    if global_localizer_config is None:
        logger.info("global_localizer config does not exist in map_config.")
        return

    localizer = init_localizer(node, map_config)

    logger.info(F"created {type(localizer).__name__}")

    _ = GlobalLocalizerNode(node, map_config, localizer)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
