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

from __future__ import annotations
from abc import abstractmethod
from dataclasses import dataclass
import importlib
import math
import types
from typing import Callable

import numpy as np

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
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

GLOBAL_LOCALIZER_CONFIG_KEY = "global_localizer"
LOCALIZER_CONFIG_KEY = "localizer"


@dataclass
class GlobalPose:
    latitude: float
    longitude: float
    rotation: float  # counter-clock-wise rotation from north [radians]
    floor: float
    height: float
    covariance: np.ndarray  # cov(x, y, z, x-axis rotation, y-axis rotation, z-axis rotation):  6 * 6 array
    confidence: float

    def to_mf_global_position2(self, stamp, frame_id):
        mf_pos = MFGlobalPosition2()
        mf_pos.header.stamp = stamp
        mf_pos.header.frame_id = frame_id
        mf_pos.latitude = self.latitude
        mf_pos.longitude = self.longitude
        mf_pos.floor = self.floor
        mf_pos.altitude = self.height
        mf_pos.heading = (-self.rotation / math.pi * 180.0) % 360.0  # rotation [rad] to heading (clock-wise from north) [degrees]
        mf_pos.position_covariance = self.covariance[:3, :3].flatten()
        mf_pos.confidence = self.confidence
        return mf_pos

    def to_pose(self, anchor: geoutil.Anchor):
        latlng = geoutil.Latlng(lat=self.latitude, lng=self.longitude)
        local_coord = geoutil.global2local(latlng, anchor)
        theta = self.rotation + (90 + anchor.rotate) / 180.0 * math.pi  # rotation[rad] to theta [rad]

        pose = Pose()
        pose.position.x = local_coord.x
        pose.position.y = local_coord.y
        pose.position.z = self.height
        pose.orientation.z = math.sin(theta/2.0)
        pose.orientation.w = math.cos(theta/2.0)
        return pose

    def to_pose_with_covariance(self, anchor: geoutil.Anchor):
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = self.to_pose(anchor)
        pose_with_cov.covariance = self.covariance.flatten()
        return pose_with_cov

    def to_pose_with_covariance_stamped(self, anchor: geoutil.Anchor, stamp, frame_id):
        pose_with_cov = PoseWithCovarianceStamped()
        pose_with_cov.header.stamp = stamp
        pose_with_cov.header.frame_id = frame_id
        pose_with_cov.pose = self.to_pose_with_covariance(anchor)
        return pose_with_cov


# the type of callback function to receive GlobalPose
LocalizerCallback = Callable[[GlobalPose], None]


def init_localizer(node: Node, map_config):
    # parse localizer_id "module/class"
    global_localizer_config = map_config.get(GLOBAL_LOCALIZER_CONFIG_KEY)
    localizer_id = global_localizer_config.get(LOCALIZER_CONFIG_KEY)
    localizer_module_name = localizer_id.split("/")[0]
    localizer_class_name = localizer_id.split("/")[1]

    module: types.ModuleType = importlib.import_module(localizer_module_name)
    cls: LocalizerInterface = getattr(module, localizer_class_name)

    localizer = cls.create(node, map_config)

    return localizer


class LocalizerInterface:
    @classmethod
    @abstractmethod
    def create(self, node: Node, config):
        pass

    @abstractmethod
    def register_callback(self, callback: LocalizerCallback):
        pass

    @abstractmethod
    def points2_callback(self, points2: PointCloud2):
        pass

    @abstractmethod
    def image1_callback(self, image: CompressedImage):
        pass

    @abstractmethod
    def image2_callback(self, image: CompressedImage):
        pass

    @abstractmethod
    def image3_callback(self, image: CompressedImage):
        pass

    @abstractmethod
    def imu_callback(self, imu: Imu):
        pass

    @abstractmethod
    def wifi_callback(self, wifi: String):
        pass

    @abstractmethod
    def beacons_callback(self, beacons: String):
        pass

    @abstractmethod
    def gnss_fix_callback(self, fix: NavSatFix):
        pass

    @abstractmethod
    def gnss_fix_velocity_callback(self, fix_velocity: TwistWithCovarianceStamped):
        pass

    @abstractmethod
    def odom_callback(self, odom: Odometry):
        pass

    @abstractmethod
    def set_enabled(self, active: bool):
        pass

    @abstractmethod
    def set_localizer_node(self, localizer_node: GlobalLocalizerNode):
        pass


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

        # set this instance to localizer to allow access
        localizer.set_localizer_node(self)

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
