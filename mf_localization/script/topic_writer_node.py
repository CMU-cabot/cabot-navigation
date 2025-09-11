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
import json
from pathlib import Path
import traceback
import queue

import cv2
import numpy as np
import open3d


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py import message_to_ordereddict

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import NavSatFix
from sensor_msgs_py import point_cloud2

from cabot_msgs.msg import PoseLog2
from mf_localization_msgs.msg import MFGlobalPosition


class TopicRecorder:
    def __init__(self, node: Node, output_dir: str):

        self._node: Node = node
        self._logger = node.get_logger()

        self._output_dir = None
        if output_dir != "":
            self._output_dir = output_dir

        # additional parameters
        self._rotate_image1 = node.declare_parameter("rotate_image1", True).value
        self._rotate_image2 = node.declare_parameter("rotate_image2", False).value
        self._rotate_image3 = node.declare_parameter("rotate_image3", True).value

        # callback group
        self._cb_group = ReentrantCallbackGroup()

        # data queue
        self._point2_queue = queue.Queue(maxsize=1)
        self._image1_queue = queue.Queue(maxsize=1)
        self._image2_queue = queue.Queue(maxsize=1)
        self._image3_queue = queue.Queue(maxsize=1)
        self._image1_throttled_queue = queue.Queue(maxsize=1)
        self._image2_throttled_queue = queue.Queue(maxsize=1)
        self._image3_throttled_queue = queue.Queue(maxsize=1)
        self._wifi_queue = queue.Queue(maxsize=1)
        self._beacons_queue = queue.Queue(maxsize=1)
        self._gnss_fix_queue = queue.Queue(maxsize=1)
        self._global_position_queue = queue.Queue(maxsize=1)
        self._pose_log2_queue = queue.Queue(maxsize=1)

        # create subscribers
        sensor_qos: QoSProfile = qos_profile_sensor_data
        self.points2_sub = node.create_subscription(PointCloud2, "points2", self.points2_callback, qos_profile=sensor_qos, callback_group=self._cb_group)
        self.image1_sub = node.create_subscription(CompressedImage, "compressed_image1", self.image1_callback, qos_profile=sensor_qos, callback_group=self._cb_group)  # front
        self.image2_sub = node.create_subscription(CompressedImage, "compressed_image2", self.image2_callback, qos_profile=sensor_qos, callback_group=self._cb_group)  # right"
        self.image3_sub = node.create_subscription(CompressedImage, "compressed_image3", self.image3_callback, qos_profile=sensor_qos, callback_group=self._cb_group)  # left
        self.image1_throttled_sub = node.create_subscription(CompressedImage, "compressed_image1_throttled", self.image1_throttled_callback, qos_profile=sensor_qos, callback_group=self._cb_group)  # front
        self.image2_throttled_sub = node.create_subscription(CompressedImage, "compressed_image2_throttled", self.image2_throttled_callback, qos_profile=sensor_qos, callback_group=self._cb_group)  # right"
        self.image3_throttled_sub = node.create_subscription(CompressedImage, "compressed_image3_throttled", self.image3_throttled_callback, qos_profile=sensor_qos, callback_group=self._cb_group)  # left
        self.wifi_sub = node.create_subscription(String, "wifi", self.wifi_callback, qos_profile=sensor_qos, callback_group=self._cb_group)
        self.beacons_sub = node.create_subscription(String, "beacons", self.beacons_callback, qos_profile=sensor_qos, callback_group=self._cb_group)
        self.gnss_fix_sub = node.create_subscription(NavSatFix, "fix", self.gnss_fix_callback, qos_profile=sensor_qos, callback_group=self._cb_group)

        self.global_position_sub = node.create_subscription(MFGlobalPosition, "/global_position", self.global_position_callback, qos_profile=sensor_qos, callback_group=self._cb_group)
        self.pose_log2_sub = node.create_subscription(PoseLog2, "/cabot/pose_log2", self.pose_log2_callback, qos_profile=sensor_qos, callback_group=self._cb_group)

        # timer
        self._node.create_timer(0.01, self.worker_tick, callback_group=self._cb_group)

    def points2_callback(self, points2: PointCloud2):
        # self._logger.info("points2_callback")
        if self._point2_queue.full():
            _ = self._point2_queue.get_nowait()
        self._point2_queue.put_nowait(points2)

    def image1_callback(self, image: CompressedImage):
        if self._image1_queue.full():
            _ = self._image1_queue.get_nowait()
        self._image1_queue.put_nowait(image)

    def image2_callback(self, image: CompressedImage):
        if self._image2_queue.full():
            _ = self._image2_queue.get_nowait()
        self._image2_queue.put_nowait(image)

    def image3_callback(self, image: CompressedImage):
        if self._image3_queue.full():
            _ = self._image3_queue.get_nowait()
        self._image3_queue.put_nowait(image)

    def image1_throttled_callback(self, image: CompressedImage):
        if self._image1_throttled_queue.full():
            _ = self._image1_throttled_queue.get_nowait()
        self._image1_throttled_queue.put_nowait(image)

    def image2_throttled_callback(self, image: CompressedImage):
        if self._image2_throttled_queue.full():
            _ = self._image2_throttled_queue.get_nowait()
        self._image2_throttled_queue.put_nowait(image)

    def image3_throttled_callback(self, image: CompressedImage):
        if self._image3_throttled_queue.full():
            _ = self._image3_throttled_queue.get_nowait()
        self._image3_throttled_queue.put_nowait(image)

    def wifi_callback(self, wifi: String):
        if self._wifi_queue.full():
            _ = self._wifi_queue.get_nowait()
        self._wifi_queue.put_nowait(wifi)

    def beacons_callback(self, beacons: String):
        if self._beacons_queue.full():
            _ = self._beacons_queue.get_nowait()
        self._beacons_queue.put_nowait(beacons)

    def gnss_fix_callback(self, fix: NavSatFix):
        if self._gnss_fix_queue.full():
            _ = self._gnss_fix_queue.get_nowait()
        self._gnss_fix_queue.put_nowait(fix)

    def global_position_callback(self, msg: MFGlobalPosition):
        if self._global_position_queue.full():
            _ = self._global_position_queue.get_nowait()
        self._global_position_queue.put_nowait(msg)

    def pose_log2_callback(self, msg: PoseLog2):
        if self._pose_log2_queue.full():
            _ = self._pose_log2_queue.get_nowait()
        self._pose_log2_queue.put_nowait(msg)

    def worker_tick(self):
        if self._global_position_queue.qsize() < 1:
            return
        global_position = self._global_position_queue.get_nowait()

        if self._point2_queue.qsize() > 0 \
           and self._image1_queue.qsize() > 0 \
           and self._wifi_queue.qsize() > 0\
           and self._gnss_fix_queue.qsize() > 0:
            pass
        else:
            return

        now = self._node.get_clock().now()
        timestamp = now.nanoseconds * 1e-9

        try:
            points2 = self._point2_queue.get_nowait()
            image1 = self._image1_queue.get_nowait()
            wifi = self._wifi_queue.get_nowait()
            fix = self._gnss_fix_queue.get_nowait()
            self._logger.info(F"points tick t = {timestamp}")

            # points2 conversion
            pts = []
            for p in point_cloud2.read_points(points2, skip_nans=True, field_names=("x", "y", "z")):
                pts.append(list(p))
            pts = np.array(pts)
            self._logger.info(F"pts = {pts}")

            # save pointcloud
            if self._output_dir is not None:
                output_path = Path(self._output_dir)
                # save pointcloud
                pcd = open3d.geometry.PointCloud()
                pcd.points = open3d.utility.Vector3dVector(pts)
                pcd_path = output_path.joinpath(F"{timestamp}_points2.pcd")
                open3d.io.write_point_cloud(pcd_path, pcd, compressed=True)

                # save image1
                img = cv2.imdecode(
                    np.frombuffer(image1.data, dtype='uint8'), cv2.IMREAD_UNCHANGED
                )
                if self._rotate_image1:
                    img = cv2.rotate(img, cv2.ROTATE_180)
                img1_path = output_path.joinpath(F"{timestamp}_image1.jpg")
                cv2.imwrite(str(img1_path), img)

                if self._image2_queue.qsize() > 0:
                    image2 = self._image2_queue.get_nowait()
                    img = cv2.imdecode(
                        np.frombuffer(image2.data, dtype='uint8'), cv2.IMREAD_UNCHANGED
                    )
                    if self._rotate_image2:
                        img = cv2.rotate(img, cv2.ROTATE_180)
                    img2_path = output_path.joinpath(F"{timestamp}_image2.jpg")
                    cv2.imwrite(str(img2_path), img)

                if self._image3_queue.qsize() > 0:
                    image3 = self._image3_queue.get_nowait()
                    img = cv2.imdecode(
                        np.frombuffer(image3.data, dtype='uint8'), cv2.IMREAD_UNCHANGED
                    )
                    if self._rotate_image3:
                        img = cv2.rotate(img, cv2.ROTATE_180)
                    img3_path = output_path.joinpath(F"{timestamp}_image3.jpg")
                    cv2.imwrite(str(img3_path), img)

                # save wifi
                wifi_json = json.loads(wifi.data)
                wifi_path = output_path.joinpath(F"{timestamp}_wifi.json")
                with open(wifi_path, "w") as f:
                    json.dump(wifi_json, f, indent=2)

                # beacons
                if self._beacons_queue.qsize() > 0:
                    beacons = self._beacons_queue.get_nowait()
                    beacons_json = json.loads(beacons.data)
                    beacons_path = output_path.joinpath(F"{timestamp}_beacons.json")
                    with open(beacons_path, "w") as f:
                        json.dump(beacons_json, f, indent=2)

                # fix
                fix_dict = message_to_ordereddict(fix)
                fix_path = output_path.joinpath(F"{timestamp}_fix.json")
                with open(fix_path, "w") as f:
                    json.dump(fix_dict, f, indent=2)

                # global_position
                gp_dict = message_to_ordereddict(global_position)
                gp_path = output_path.joinpath(F"{timestamp}_globalposition.json")
                with open(gp_path, "w") as f:
                    json.dump(gp_dict, f, indent=2)

                # pose log2
                if self._pose_log2_queue.qsize() > 0:
                    pose_log2 = self._pose_log2_queue.get_nowait()
                    pl2_dict = message_to_ordereddict(pose_log2)
                    pl2_path = output_path.joinpath(F"{timestamp}_poselog2.json")
                    with open(pl2_path, "w") as f:
                        json.dump(pl2_dict, f, indent=2)

        except queue.Empty:
            self._logger.info(traceback.format_exc())
            return


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)


def main():
    rclpy.init()
    node = Node("topic_writer")
    logger = node.get_logger()

    # parameter
    output_dir = node.declare_parameter("output_dir", "").value

    if output_dir == "":
        logger.error("output_dir is not specified.")
    else:
        logger.info(F"output_dir = {output_dir}")

    _: TopicRecorder = TopicRecorder(node, output_dir)
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
