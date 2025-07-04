#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2021  IBM Corporation
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


import os
import os.path
import json
import orjson
import math
from enum import Enum
import numpy as np
from packaging.specifiers import InvalidSpecifier
from packaging.specifiers import Specifier
from packaging.version import InvalidVersion
from packaging.version import Version
import sys
import signal
import threading
from typing import Optional
# import traceback
import yaml
from dataclasses import dataclass

import rclpy
import rclpy.client
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
import rclpy.time
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy

from launch import LaunchService
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
import osrf_pycommon.process_utils

import tf2_ros
import tf_transformations
import message_filters
from std_msgs.msg import String, Int64, Float64
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Point, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped  # gnss_fix_velocity
from sensor_msgs.msg import NavSatFix, NavSatStatus, FluidPressure
# necessary to use tfBuffer.transform(pose_stamped_msg, frame_id)
from tf2_geometry_msgs import PoseStamped
from tf2_geometry_msgs import PointStamped, Vector3Stamped

from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import GetTrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory
from cartographer_ros_msgs.srv import StartTrajectory
from cartographer_ros_msgs.srv import ReadMetrics
from cartographer_ros_msgs.srv import TrajectoryQuery

import mf_localization.geoutil as geoutil
import mf_localization.resource_utils as resource_utils

# from mf_localization.wireless_utils import extract_samples
from wireless_rss_localizer import create_wireless_rss_localizer

from mf_localization_msgs.msg import MFGlobalPosition
from mf_localization_msgs.msg import MFLocalizeStatus
from mf_localization_msgs.msg import MFNavSAT
from mf_localization_msgs.srv import ConvertLocalToGlobal
from mf_localization_msgs.srv import MFSetInt
from mf_localization_msgs.srv import MFTrigger
from mf_localization_msgs.srv import RestartLocalization
from mf_localization_msgs.srv import StartLocalization
from mf_localization_msgs.srv import StopLocalization

from mf_localization.altitude_manager import AltitudeManager
from mf_localization.altitude_manager import AltitudeFloorEstimator
from mf_localization.altitude_manager import AltitudeFloorEstimatorParameters
from mf_localization.altitude_manager import BalancedSampler
from mf_localization.altitude_manager import FloorHeightMapper

from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

import std_msgs.msg
from cabot_msgs.srv import LookupTransform
from std_srvs.srv import Trigger

from ublox_msgs.msg import NavSAT
from ublox_msgs.srv import SendCfgRST
from ublox_converter import UbloxConverterNode


def json2anchor(jobj):
    return geoutil.Anchor(lat=jobj["lat"],
                          lng=jobj["lng"],
                          rotate=jobj["rotate"],
                          )


def toTransmat(x, y, yaw):
    Tmat = np.array([[np.cos(yaw), -np.sin(yaw), x],
                    [np.sin(yaw), np.cos(yaw), y],
                    [0.0, 0.0, 1.0]])
    return Tmat


class LocalizationMode(Enum):
    INIT = "init"
    TRACK = "track"

    def __str__(self):
        return self.value


class IndoorOutdoorMode(Enum):
    UNKNOWN = "unknown"
    INDOOR = "indoor"
    OUTDOOR = "outdoor"

    def __str__(self):
        return self.value


class RSSType(Enum):
    iBeacon = 0
    WiFi = 1


def extract_samples_ble_wifi_other(samples):
    import copy
    samples_ble = []
    samples_wifi = []
    samples_other = []

    for s in samples:
        s2_ble = [b for b in s["data"]["beacons"] if b["type"] == "iBeacon"]
        s2_wifi = [b for b in s["data"]["beacons"] if b["type"] == "WiFi"]

        if 0 < len(s2_ble):
            s2 = copy.copy(s)
            s2["data"]["beacons"] = s2_ble
            samples_ble.append(s2)
        if 0 < len(s2_wifi):
            s2 = copy.copy(s)
            s2["data"]["beacons"] = s2_wifi
            samples_wifi.append(s2)
        if len(s2_ble) == 0 and len(s2_wifi) == 0:
            s2 = copy.copy(s)
            samples_other.append(s2)

    return samples_ble, samples_wifi, samples_other


def convert_samples_coordinate_slow(samples, from_anchor, to_anchor, floor):
    samples2 = []
    for s in samples:
        s2 = s.copy()
        info = s["information"]
        xy = geoutil.Point(x=info["x"], y=info["y"])
        latlng = geoutil.local2global(xy, from_anchor)
        local_coord = geoutil.global2local(latlng, to_anchor)
        s2["information"]["x"] = local_coord.x
        s2["information"]["y"] = local_coord.y
        s2["information"]["floor"] = floor
        samples2.append(s2)

    return samples2


def convert_samples_coordinate(samples, from_anchor, to_anchor, floor):
    # check empty list
    if len(samples) == 0:
        return []

    # convert from_anchor point to to_anchor coordinate
    xy = geoutil.Point(x=0.0, y=0.0)
    latlng = geoutil.local2global(xy, from_anchor)
    local_coord = geoutil.global2local(latlng, to_anchor)
    X0 = local_coord.x
    Y0 = local_coord.y

    # calculate from_anchor to to_anchor rotation
    rad_from = - np.deg2rad(from_anchor.rotate)
    rad_to = - np.deg2rad(to_anchor.rotate)
    theta = rad_to - rad_from

    # convert samples coordinate X to to_anchor coordinate
    X = np.array([[s["information"]["x"], s["information"]["y"]] for s in samples])  # create [[sample.x, sample.y]] array
    R = np.array([[np.cos(theta), np.sin(theta)],
                  [-np.sin(theta), np.cos(theta)]])
    X2 = X @ R.T + np.array([X0, Y0])

    # create converted samples
    samples2 = []
    for i, s in enumerate(samples):
        s2 = s.copy()
        s2["information"]["x"] = X2[i, 0]
        s2["information"]["y"] = X2[i, 1]
        s2["information"]["floor"] = floor
        samples2.append(s2)

    return samples2


def compute_relative_pose(pose1: Pose, pose2: Pose) -> Pose:
    relative_position = [
        pose2.position.x - pose1.position.x,
        pose2.position.y - pose1.position.y,
        pose2.position.z - pose1.position.z,
        1  # for homogeneous transformation
    ]
    q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
    q2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
    q1_inv = tf_transformations.quaternion_inverse(q1)
    R = tf_transformations.quaternion_matrix(q1_inv)  # homogeneous rotation matrix
    rotated_relative_position = R.dot(relative_position)
    relative_orientation = tf_transformations.quaternion_multiply(q1_inv, q2)

    relative_pose = Pose()
    relative_pose.position.x = rotated_relative_position[0]
    relative_pose.position.y = rotated_relative_position[1]
    relative_pose.position.z = rotated_relative_position[2]
    relative_pose.orientation.x = relative_orientation[0]
    relative_pose.orientation.y = relative_orientation[1]
    relative_pose.orientation.z = relative_orientation[2]
    relative_pose.orientation.w = relative_orientation[3]

    return relative_pose


class FloorManager:
    def __init__(self):
        self.node_id = None
        self.frame_id = None
        self.localizer = None
        self.wifi_localizer = None
        self.map_filename = ""
        self.min_hist_count = 1

        # publisher
        self.initialpose_pub = None
        self.imu_pub = None
        self.points_pub = None
        self.odom_pub = None
        self.fix_pub = None

        # services
        self.get_trajectory_states: rclpy.client.Client = None
        self.finish_trajectory: rclpy.client.Client = None
        self.start_trajectory: rclpy.client.Client = None
        self.read_metrics: rclpy.client.Client = None
        self.trajectory_query: rclpy.client.Client = None

        # variables
        self.previous_fix_local_published = None
        self.trajectory_initial_pose = None  # variable to keep the initial pose from /trajectory_query

        # variables - optimization
        self.constraints_count = 0

    def reset_states(self):
        self.previous_fix_local_published = None
        self.constraints_count = 0


class TFAdjuster:
    def __init__(self, frame_id: str, map_frame_adjust: str, adjust_tf: bool):
        # for gnss adjust
        # constant
        self.frame_id = frame_id
        self.map_frame_adjust = map_frame_adjust
        self.adjust_tf = adjust_tf

        # variable
        self.gnss_adjust_x = 0.0
        self.gnss_adjust_y = 0.0
        self.gnss_adjust_yaw = 0.0
        self.gnss_fix_list = []
        self.local_odom_list = []
        self.gnss_total_count = 0
        self.zero_adjust_uncertainty = 0.0

    def reset(self):
        self.gnss_adjust_x = 0.0
        self.gnss_adjust_y = 0.0
        self.gnss_adjust_yaw = 0.0
        self.gnss_fix_list = []
        self.local_odom_list = []
        self.gnss_total_count = 0
        self.zero_adjust_uncertainty = 0.0


@dataclass
class GNSSParameters:
    gnss_position_covariance_threshold: float = 0.2 * 0.2  # [meter^2]
    gnss_position_covariance_initial_threshold: float = 0.2 * 0.2  # [meters^2]
    # fix filter
    gnss_status_threshold: int = NavSatStatus.STATUS_GBAS_FIX
    gnss_fix_motion_filter_distance: float = 0.1  # [meter]
    gnss_fix_filter_min_cno: int = 30
    gnss_fix_filter_min_elev: int = 45
    gnss_fix_filter_num_sv: int = 15
    # track error
    gnss_track_error_threshold: float = 5.0  # [meter]
    gnss_track_yaw_threshold: float = np.radians(30)
    gnss_track_error_adjust: float = 0.1
    gnss_n_max_correspondences: int = 20
    gnss_n_min_correspondences_stable: int = 10
    gnss_odom_jump_threshold: float = 2.0
    gnss_odom_small_threshold: float = 0.5
    gnss_localization_interval: float = 10  # [s]
    # navsat timeout
    gnss_navsat_timeout: float = 5.0  # [s]
    gnss_position_covariance_indoor_threshold: float = 3.0 * 3.0
    # parameter for floor estimation
    gnss_use_floor_estimation: bool = False
    gnss_floor_search_radius: float = None  # [m]
    gnss_floor_estimation_probability_scale: float = 0.0  # p = exp(scale*(floor-max_floor))/Z  default: 0.0
    gnss_status_floor_selection: bool = True
    gnss_floor_selection_status_threhold: int = NavSatStatus.STATUS_GBAS_FIX

    # gnss reset service timeout
    gnss_reset_timeout: float = 1.0  # [s]


@dataclass
class RSSLocalizationParameters:
    initial_localization: bool = True
    continuous_localization: bool = True
    failure_detection: bool = True


class TagUtil:
    @staticmethod
    def parse_version_tags_input(tags_input: list | str):
        tags = []
        if type(tags_input) is list:
            for tag in tags_input:
                tags_temp = [t.strip() for t in tag.split(",")]
                tags.extend(tags_temp)
        else:  # comma-separated str
            tags = [t.strip() for t in tags_input.split(",")]
        tags2 = []
        for tag in tags:
            try:
                tags2.append(Version(tag))
            except InvalidVersion:
                tags2.append(tag)
        tags = tags2
        return tags

    @staticmethod
    def parse_specifier_tags_input(tags_input: list | str):
        tags = []
        if type(tags_input) is list:
            for tag in tags_input:
                tags_temp = [t.strip() for t in tag.split(",")]
                tags.extend(tags_temp)
        else:  # comma-separated str
            tags = [t.strip() for t in tags_input.split(",")]
        tags2 = []
        for tag in tags:
            try:
                tags2.append(Specifier(tag))
            except InvalidSpecifier:
                tags2.append(tag)
        tags = tags2
        return tags

    @staticmethod
    def versions_in_specifiers(version_tags, specifier_tags):
        match = False
        for ver_tag in version_tags:
            if ver_tag == "all":  # special string
                match = True
            for spec_tag in specifier_tags:
                if type(spec_tag) is Specifier:
                    if ver_tag in spec_tag:
                        match = True
                else:
                    if spec_tag == "all":  # special string
                        match = True
                    if ver_tag == spec_tag:
                        match = True
        return match


@dataclass
class MultiFloorManagerParameters:
    # area classification
    area_floor_const: float = 10000
    area_check_interval: float = 1.0  # [s]
    area_distance_threshold: float = 10  # [m]


class MultiFloorManager:
    def __init__(self, node,
                 multi_floor_manager_parameters: MultiFloorManagerParameters = MultiFloorManagerParameters()):
        self.node = node
        self.clock = node.get_clock()
        self.logger = node.get_logger()

        # state variables
        self.is_active = True
        self.floor = None  # state
        self.area = None  # state
        self.current_frame = None  # state
        self.mode = None  # state
        self.valid_beacon = False  # state for input validation
        self.valid_wifi = False  # state for input validation
        # for optimization detection
        self.map2odom = None  # state
        self.optimization_detected = False  # state
        self.optimization_queue_length = None
        # for initial pose estimation timeout
        self.initial_localization_time = None
        self.initial_localization_timeout_detected = False
        self.initial_localization_timeout = 300  # seconds
        self.initial_localization_timeout_auto_relocalization = True
        # for gnss initial pose estimation timeout
        self.gnss_init_time = None
        self.gnss_init_timeout = 60  # seconds
        self.gnss_init_timeout_detected = False
        # for loginfo
        self.spin_count = 0
        self.prev_spin_count = None

        self.ble_localizer_dict = {}
        self.ble_floor_localizer = None
        self.wifi_floor_localizer = None
        self.pressure_available = True
        self.altitude_manager = None
        self.altitude_floor_estimator = None
        self.floor_height_mapper: FloorHeightMapper = None

        # ble wifi localization
        self.use_ble = True
        self.use_wifi = False
        self.ble_localization_parameters: RSSLocalizationParameters = None
        self.wifi_localization_parameters: RSSLocalizationParameters = None

        # area identification
        self.area_localizer = None
        self.X_area = None
        self.Y_area = None
        self.previous_area_check_time = None
        # area identification parameters
        self.area_floor_const = multi_floor_manager_parameters.area_floor_const
        self.area_check_interval = multi_floor_manager_parameters.area_check_interval
        self.area_distance_threshold = multi_floor_manager_parameters.area_distance_threshold

        self.transforms = []

        # average floor values
        self.floor_queue_size = 10
        self.floor_queue = []  # state
        self.floor_list = []  # store known floor values

        # failure detection
        self.rmse_threshold = 5.0
        self.loc_queue_min_size = 5
        self.loc_queue_max_size = 10
        self.loc_queue = []
        self.loc_beacon_queue = []

        # auto-relocalization
        self.auto_relocalization = False

        # frames
        self.global_map_frame = "map"
        self.local_map_frame = "map"
        self.odom_frame = "odom"
        self.published_frame = "base_control_shift"
        self.base_link_frame = "base_link"
        self.global_position_frame = "base_link"  # frame_id to compute global position
        # unknown frame to temporarily cut a TF tree
        self.unknown_frame = "unknown"

        # publisher
        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.current_floor_pub = self.node.create_publisher(Int64, "current_floor", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.current_floor_raw_pub = self.node.create_publisher(Float64, "current_floor_raw", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.current_floor_smoothed_pub = self.node.create_publisher(Float64, "current_floor_smoothed", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.current_frame_pub = self.node.create_publisher(String, "current_frame", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.current_map_filename_pub = self.node.create_publisher(String, "current_map_filename", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.current_area_pub = self.node.create_publisher(Int64, "current_area", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.current_mode_pub = self.node.create_publisher(Int64, "current_mode", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.resetpose_pub = self.node.create_publisher(PoseWithCovarianceStamped, "resetpose", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.global_position_pub = self.node.create_publisher(MFGlobalPosition, "global_position", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.localize_status_pub = self.node.create_publisher(MFLocalizeStatus, "localize_status", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.localize_status = MFLocalizeStatus.UNKNOWN

        # verbosity
        self.verbose = False

        # for send_local_map_tf
        self.local_map_tf = None

        # for gnss localization
        # parameters
        self.gnss_params = GNSSParameters()
        # variables
        self.gnss_adjuster_dict = {}
        self.prev_navsat_msg = None
        self.prev_navsat_status = False
        self.prev_mf_navsat_msg = None
        self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
        self.gnss_is_active = False
        self.prev_publish_map_frame_adjust_timestamp = None
        self.gnss_fix_local_pub = None
        self.gnss_localization_time = None
        self.gnss_navsat_time = None
        self.gnss_balanced_floor_sampler = BalancedSampler()

        self.updater = Updater(self.node)

        def localize_status(stat):
            if self.valid_beacon:
                stat.add("Beacon input", "valid")
            else:
                stat.add("Beacon input", "invalid")
            if self.valid_wifi:
                stat.add("WiFi input", "valid")
            else:
                stat.add("WiFi input", "invalid")
            if self.floor:
                stat.add("Floor", F"{self.floor}")
            else:
                stat.add("Floor", "invalid")

            if self.localize_status == MFLocalizeStatus.UNKNOWN:
                stat.summary(DiagnosticStatus.WARN, "Unknown")
            if self.localize_status == MFLocalizeStatus.LOCATING:
                stat.summary(DiagnosticStatus.WARN, "Locating")
            if self.localize_status == MFLocalizeStatus.TRACKING:
                stat.summary(DiagnosticStatus.OK, "Tracking")
            if self.localize_status == MFLocalizeStatus.UNRELIABLE:
                stat.summary(DiagnosticStatus.WARN, "Unreliable")
            return stat
        self.updater.add(FunctionDiagnosticTask("Localize Status", localize_status))

    # state getter/setter
    @property
    def floor(self):
        return self.__floor

    @floor.setter
    def floor(self, value):
        self.__floor = value
        if value is not None and self.current_floor_pub:
            current_floor_msg = Int64()
            current_floor_msg.data = int(self.floor)
            self.current_floor_pub.publish(current_floor_msg)

    @property
    def area(self):
        return self.__area

    @area.setter
    def area(self, value):
        self.__area = value
        if value is not None and self.current_area_pub:
            current_area_msg = Int64()
            current_area_msg.data = int(self.area)
            self.current_area_pub.publish(current_area_msg)

    @property
    def mode(self):
        return self.__mode

    @mode.setter
    def mode(self, value):
        self.__mode = value
        if value is not None and self.current_mode_pub:
            current_mode_msg = Int64()
            current_mode_msg.data = 0 if self.mode == LocalizationMode.INIT else 1
            self.current_mode_pub.publish(current_mode_msg)

    @property
    def current_frame(self):
        return self.__current_frame

    @current_frame.setter
    def current_frame(self, value):
        self.__current_frame = value
        if value is not None and self.current_frame_pub:
            current_frame_msg = String()
            current_frame_msg.data = self.__current_frame
            self.current_frame_pub.publish(current_frame_msg)
            self.send_local_map_tf()

    # broadcast tf from global_map_frame to each (local) map_frame
    def send_static_transforms(self):
        for t in self.transforms:
            t.header.stamp = self.clock.now().to_msg()  # update timestamp
        static_broadcaster.sendTransform(self.transforms)

    # broadcast tf between the current_frame to local_map_frame
    def send_local_map_tf(self):
        if self.local_map_tf is None:
            # initialization
            t = TransformStamped()
            t.child_frame_id = self.local_map_frame  # static
            t.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)  # static
            q = tf_transformations.quaternion_from_euler(0, 0, 0, 'sxyz')
            rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            t.transform.rotation = rotation  # static
            # tentative values
            t.header.stamp = self.clock.now().to_msg()
            t.header.frame_id = ""
            self.local_map_tf = t

        if self.current_frame is not None:
            t = self.local_map_tf
            # send transform only when current_frame changes
            if self.current_frame != t.header.frame_id:
                t.header.stamp = self.clock.now().to_msg()
                t.header.frame_id = self.current_frame
                transform_list = self.transforms + [t]  # to keep self.transforms in static transform
                static_broadcaster.sendTransform(transform_list)

    # dummy tf to prevent timeout error before localization starts
    # global_map_frame -> local_map_frame(map) -> published_frame
    def send_dummy_local_map_tf(self):
        if self.current_frame is None:
            # dummy transform to prevent navigation2 errors
            stamp = self.clock.now().to_msg()
            dummy_transforms = []
            if self.local_map_frame != self.global_map_frame:
                t1 = TransformStamped()
                t1.child_frame_id = self.local_map_frame
                t1.header.frame_id = self.global_map_frame
                t1.header.stamp = stamp
                dummy_transforms.append(t1)
            t2 = TransformStamped()
            t2.child_frame_id = self.published_frame
            t2.header.frame_id = self.local_map_frame
            t2.header.stamp = stamp
            dummy_transforms.append(t2)
            transform_list = self.transforms + dummy_transforms  # to keep self.transforms in static transform
            self.logger.info(f"{transform_list=}")
            static_broadcaster.sendTransform(transform_list)

    @property
    def localize_status(self):
        return self.__localize_status

    @localize_status.setter
    def localize_status(self, value):
        self.__localize_status = value
        if value and self.localize_status_pub:  # unknown=0 is not published
            msg = MFLocalizeStatus()
            msg.status = value
            self.localize_status_pub.publish(msg)

    def initialpose_callback(self, pose_with_covariance_stamped_msg: PoseWithCovarianceStamped):
        if self.verbose:
            self.logger.info(f"multi_floor_manager.initialpose_callback: initialpose = {pose_with_covariance_stamped_msg}")

        # substitute ROS time to prevent error when gazebo is running and the pose message is published by rviz
        pose_with_covariance_stamped_msg.header.stamp = self.clock.now().to_msg()

        # convert initialpose frame if needed and possible
        frame_id = pose_with_covariance_stamped_msg.header.frame_id
        if frame_id != self.global_map_frame:
            if self.verbose:
                self.logger.info(f"transform initialpose on {frame_id} frame to the global frame ({self.global_map_frame})")
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header = pose_with_covariance_stamped_msg.header
            pose_stamped_msg.pose = pose_with_covariance_stamped_msg.pose.pose
            try:
                converted_pose_stamped = tfBuffer.transform(pose_stamped_msg, self.global_map_frame, timeout=Duration(seconds=1.0))  # timeout 1.0 s
                pose_with_covariance_stamped_msg.header.frame_id = self.global_map_frame
                pose_with_covariance_stamped_msg.pose.pose = converted_pose_stamped.pose
            except RuntimeError:
                # do not update pose_with_covariance_stamped_msg
                self.logger.info(F"LookupTransform Error {pose_stamped_msg.header.frame_id} -> {self.global_map_frame} in initialpose_callback. Assuming initialpose is published on the global frame({self.global_map_frame}).")

        status_code = self.initialize_with_global_pose(pose_with_covariance_stamped_msg, mode=LocalizationMode.TRACK)

        # set is_active to True if succeeded to start trajectory
        if status_code == 0:
            self.is_active = True

    def initialize_with_global_pose(self, pose_with_covariance_stamped_msg: PoseWithCovarianceStamped, mode=None):

        # set target mode
        if mode is not None:  # update the target mode
            target_mode = mode
        else:
            if self.mode is None:
                target_mode = LocalizationMode.INIT
            else:
                target_mode = self.mode  # keep the current mode

        if self.floor is None:
            self.logger.info("floor is unknown. Set floor by calling /set_current_floor service before publishing the 2D pose estimate.")

        if self.floor is not None:
            # transform pose in the message from map frame to a local frame
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header = pose_with_covariance_stamped_msg.header
            pose_stamped_msg.pose = pose_with_covariance_stamped_msg.pose.pose

            # detect area
            x_area = [[pose_stamped_msg.pose.position.x, pose_stamped_msg.pose.position.y, float(self.floor)*self.area_floor_const]]  # [x,y,floor]
            target_area = self.area_localizer.predict(x_area)[0]  # [area] area may change.

            if self.verbose:
                self.logger.info(f"multi_floor_manager.initialize_with_global_pose: mode={target_mode}, floor={self.floor}, area={target_area}")

            # get information from floor_manager
            floor_manager = self.ble_localizer_dict[self.floor][target_area][target_mode]
            frame_id = floor_manager.frame_id
            map_filename = floor_manager.map_filename
            node_id = floor_manager.node_id

            # transform initialpose on the global map frame to the local map frame (frame_id).
            try:
                # this assumes frame_id of pose_stamped_msg is correctly set.
                local_pose_stamped = tfBuffer.transform(pose_stamped_msg, frame_id, timeout=Duration(seconds=1.0))  # timeout 1.0 s
            except RuntimeError:
                # when the frame_id of pose_stamped_msg is not correctly set (e.g. frame_id = map), assume the initial pose is published on the target frame.
                # this workaround behaves intuitively in typical cases.
                self.logger.info(F"LookupTransform Error {pose_stamped_msg.header.frame_id} -> {frame_id} in initialize_with_global_pose. Assuming initial pose is published on the target frame ({frame_id}).")
                local_pose_stamped = PoseStamped()
                local_pose_stamped.header = pose_stamped_msg.header
                local_pose_stamped.header.frame_id = frame_id
                local_pose_stamped.pose = pose_stamped_msg.pose

            local_pose = local_pose_stamped.pose
            local_pose.position.z = 0.0  # set z = 0 to ensure 2D position on the local map

            # restart trajectory with local_pose
            if self.area is not None:
                self.finish_trajectory()  # finish trajectory before updating area value

            self.mode = target_mode
            self.area = target_area

            status_code = self.start_trajectory_with_pose(local_pose)
            self.logger.info(F"called /{node_id}/{self.mode}/start_trajectory")

            # reset altitude_floor_estimator in floor initialization process
            self.altitude_floor_estimator.reset(floor_est=self.floor)

            # set current_frame and publish it in the setter
            self.current_frame = frame_id

            # publish current map_filename
            self.current_map_filename_pub.publish(String(data=map_filename))

            if self.mode == LocalizationMode.INIT:
                self.localize_status = MFLocalizeStatus.LOCATING
                self.initial_localization_time = self.clock.now()
            elif self.mode == LocalizationMode.TRACK:
                self.localize_status = MFLocalizeStatus.TRACKING
                self.initial_localization_time = None

            return status_code  # result of start_trajectory_with_pose

    def restart_floor(self, local_pose: Pose):
        # set z = 0 to ensure 2D position on the local map
        local_pose.position.z = 0.0

        floor_manager = self.ble_localizer_dict[self.floor][self.area][self.mode]
        frame_id = floor_manager.frame_id
        map_filename = floor_manager.map_filename

        # local_pose to pose_cov_stamped
        pose_cov_stamped = PoseWithCovarianceStamped()
        pose_cov_stamped.header.stamp = self.clock.now().to_msg()
        pose_cov_stamped.header.frame_id = frame_id
        pose_cov_stamped.pose.pose = local_pose
        covariance = np.diag(self.initial_pose_variance)
        pose_cov_stamped.pose.covariance = list(covariance.flatten())

        # start trajectory with local_pose
        self.resetpose_pub.publish(pose_cov_stamped)  # publish local_pose for visualization
        status_code_start_trajectory = self.start_trajectory_with_pose(local_pose)
        self.logger.info(F"called /{floor_manager.node_id}/{self.mode}/start_trajectory, code={status_code_start_trajectory}")

        # set current_frame and publish it in the setter
        self.current_frame = frame_id

        # publish current map_filename
        self.current_map_filename_pub.publish(String(data=map_filename))

        if self.mode == LocalizationMode.INIT:
            self.localize_status = MFLocalizeStatus.LOCATING
            self.initial_localization_time = self.clock.now()
        if self.mode == LocalizationMode.TRACK:
            self.localize_status = MFLocalizeStatus.TRACKING
            self.initial_localization_time = None

    # simple failure detection based on the root mean square error between tracked and estimated locations
    def check_localization_failure(self, loc_track, loc_est):
        if self.verbose:
            self.logger.info(F"loc_track={loc_track}, loc_est={loc_est}")

        if self.loc_queue_max_size <= len(self.loc_queue):
            self.loc_queue.pop(0)
            self.loc_beacon_queue.pop(0)

        self.loc_queue.append(loc_track)
        self.loc_beacon_queue.append(loc_est)

        failure_detected = False
        if self.loc_queue_min_size <= len(self.loc_queue):
            X1 = np.array(self.loc_queue)
            X2 = np.array(self.loc_beacon_queue)
            rmse = np.sqrt(np.mean(np.sum((X1-X2)**2, axis=1)))
            if self.rmse_threshold <= rmse:
                failure_detected = True
                # clear location lists
                self.loc_queue = []
                self.loc_beacon_queue = []
            if self.verbose:
                self.logger.info(F"rmse={rmse}, failure_detected={failure_detected}")

        return failure_detected

    def pressure_callback(self, message):
        self.altitude_manager.put_pressure(message)

        if not self.pressure_available:
            return

        if not self.altitude_floor_estimator.enabled():
            return

        # get robot pose for floor height mapper
        if self.floor is not None and self.mode is not None:
            # get robot pose
            try:
                robot_pose = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.clock.clock_type))
            except RuntimeError as e:
                self.logger.warn(F"{e}")
                return
        else:
            return

        # update floor list and height list in altitude_floor_manager
        x = [robot_pose.transform.translation.x, robot_pose.transform.translation.y]
        floor_list, height_list = self.floor_height_mapper.get_floor_height_list(x)
        self.altitude_floor_estimator.set_floor_height_list(floor_list, height_list)

        # detect floor change event
        result = self.altitude_floor_estimator.put_pressure(message)
        target_floor = result.floor_est
        floor_change_event = result.floor_change_event

        # log floor change event
        if floor_change_event is not None:
            self.logger.info(F"pressure_callback: floor_change_event (floor_est={target_floor}) detected. (current_floor={self.floor}, floor_list={floor_list}, height_list={height_list})")

        if self.floor is not None:
            if self.floor != target_floor \
                    and (floor_change_event is not None):

                # detect area in target_floor
                x_area = [[robot_pose.transform.translation.x, robot_pose.transform.translation.y, float(target_floor) * self.area_floor_const]]  # [x,y,floor]

                # find area candidates
                neigh_dists, neigh_indices = self.area_localizer.kneighbors(x_area, n_neighbors=1)
                area_candidates = self.Y_area[neigh_indices]
                neigh_dist = neigh_dists[0][0]
                area = area_candidates[0][0]

                # reject if the candidate area may be unreachable.
                if self.area_distance_threshold < neigh_dist:
                    if self.verbose:
                        self.logger.info(F"pressure_callback: rejected unreachable floor change ({self.floor}, {self.area}) -> ({target_floor}, {area})")
                    return

                # set temporal variables
                target_area = area
                target_mode = self.mode

                # check the availablity of local_pose on the target frame
                floor_manager = self.ble_localizer_dict[target_floor][target_area][target_mode]
                frame_id = floor_manager.frame_id  # target frame_id
                local_transform = None
                try:
                    # tf from the origin of the target floor to the robot pose
                    local_transform = tfBuffer.lookup_transform(frame_id, self.base_link_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.clock.clock_type))
                except RuntimeError as e:
                    self.logger.error(F'LookupTransform Error from {frame_id} to {self.base_link_frame}. error=RuntimeError({e})')

                # update the trajectory only when local_transform is available
                if local_transform is not None:
                    # create local_pose instance
                    v3 = local_transform.transform.translation  # Vector3
                    position = Point(x=v3.x, y=v3.y, z=v3.z)
                    orientation = local_transform.transform.rotation  # Quaternion
                    local_pose = Pose(position=position, orientation=orientation)

                    # try to finish the current trajectory before updating state variables
                    self.finish_trajectory()

                    # update state variables to switch floor
                    self.floor = target_floor
                    self.area = target_area
                    self.mode = target_mode

                    # restart trajectory with the updated state variables
                    self.restart_floor(local_pose)

    def beacons_callback(self, message):
        self.valid_beacon = True
        if self.verbose:
            self.logger.info("multi_floor_manager.beacons_callback")

        data = json.loads(message.data)
        beacons = data["data"]

        if self.use_ble:
            self.rss_callback(beacons, rss_type=RSSType.iBeacon, rss_loc_params=self.ble_localization_parameters)

    def wifi_callback(self, message):
        self.valid_wifi = True
        if self.verbose:
            self.logger.info("multi_floor_manager.wifi_callback")

        data = json.loads(message.data)
        beacons = data["data"]

        if self.use_wifi:
            self.rss_callback(beacons, rss_type=RSSType.WiFi, rss_loc_params=self.wifi_localization_parameters)

    def rss_callback(self, beacons, rss_type=RSSType.iBeacon,
                     rss_loc_params: RSSLocalizationParameters = RSSLocalizationParameters()
                     ):
        if not self.is_active:
            # do nothing
            return

        # detect floor
        floor_localizer = None
        if rss_type == RSSType.iBeacon:
            floor_localizer = self.ble_floor_localizer
        elif rss_type == RSSType.WiFi:
            floor_localizer = self.wifi_floor_localizer

        loc = floor_localizer.predict(beacons)  # [[x,y,z,floor]]

        if loc is None:
            return

        floor_raw = np.mean(loc[:, 3])
        if self.verbose:
            self.logger.info(F"loc = {loc}")
            self.logger.info(F"floor_raw = {floor_raw}, {loc[:, 3]}")

        # extract latest floor_raw values from floor_queue to calculate the moving average
        now = self.clock.now()
        self.floor_queue = [elem for elem in self.floor_queue if now - elem[0] < Duration(seconds=self.floor_queue_size)]
        self.floor_queue.append([now, floor_raw])
        floor_values = [elem[1] for elem in self.floor_queue]
        if self.verbose:
            self.logger.info(F"floor_queue = {floor_values}")

        # calculate mean (smoothed) floor
        mean_floor = np.mean(floor_values)
        self.current_floor_raw_pub.publish(Float64(data=floor_raw))
        self.current_floor_smoothed_pub.publish(Float64(data=mean_floor))

        # use one of the known floor values closest to the mean value of floor_queue
        idx_floor = np.abs(np.array(self.floor_list) - mean_floor).argmin()
        floor = self.floor_list[idx_floor]

        # detect area
        x_area = [[loc[0, 0], loc[0, 1], floor * self.area_floor_const]]  # [x,y,floor]
        area = self.area_localizer.predict(x_area)[0]  # [area]

        if self.verbose:
            self.logger.info(F"floor = {floor}, area={area}")

        # do not start a trajectory when gnss is active and the robot is not in indoor environments.
        if self.floor is None:
            if self.gnss_is_active \
                    and self.indoor_outdoor_mode != IndoorOutdoorMode.INDOOR:
                self.logger.info("skip start trajectory (gnss_is_active and indoor_outdoor_mode!=INDOOR)")
                return
        else:
            # allow switch trajectories
            pass

        # switch cartgrapher node
        if self.floor is None:
            # skip if not activated
            if not rss_loc_params.initial_localization:
                self.logger.info(F"rss_callback({rss_type}): skipping initial localization", throttle_duration_sec=5.0)
                return

            # coarse initial localization on local frame (frame_id)
            localizer = None
            if rss_type == RSSType.iBeacon:
                localizer = self.ble_localizer_dict[floor][area][LocalizationMode.INIT].localizer
            elif rss_type == RSSType.WiFi:
                localizer = self.ble_localizer_dict[floor][area][LocalizationMode.INIT].wifi_localizer

            # local_loc is on the local coordinate on frame_id
            local_loc = localizer.predict(beacons)

            # project loc to sample locations
            local_loc = localizer.find_closest(local_loc)

            if local_loc is None:
                return

            # set variables if local loc is available
            self.floor = floor
            self.area = area
            self.mode = LocalizationMode.INIT
            self.logger.info(F"initialize floor = {self.floor}")

            # create a local pose instance
            position = Point(x=local_loc[0, 0], y=local_loc[0, 1], z=local_loc[0, 2])  # use the estimated position
            orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # orientation is unknown.
            local_pose = Pose(position=position, orientation=orientation)

            self.restart_floor(local_pose)

            # reset altitude_floor_estimator in floor initialization process
            self.altitude_floor_estimator.reset(floor_est=floor)

        # floor change or init->track
        elif ((self.altitude_manager.is_height_changed() or not self.pressure_available) and self.floor != floor) \
                or (self.mode == LocalizationMode.INIT and self.optimization_detected):
            # skip if not activated
            if not rss_loc_params.continuous_localization:
                self.logger.info(F"rss_callback({rss_type}): skipping continuous localization", throttle_duration_sec=5.0)
                return

            if self.floor != floor:
                self.logger.info(F"floor change detected ({self.floor} -> {floor}).")

                # check if the candidate pose is reachable
                # get robot pose
                try:
                    robot_pose = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.clock.clock_type))
                except RuntimeError as e:
                    self.logger.warn(F"{e}")
                    return
                # detect area in target_floor
                x_area = [[robot_pose.transform.translation.x, robot_pose.transform.translation.y, float(floor) * self.area_floor_const]]  # [x,y,floor]
                # find area candidates
                neigh_dists, neigh_indices = self.area_localizer.kneighbors(x_area, n_neighbors=1)
                area_candidates = self.Y_area[neigh_indices]
                neigh_dist = neigh_dists[0][0]
                area = area_candidates[0][0]
                # reject if the candidate area may be unreachable.
                if self.area_distance_threshold < neigh_dist:
                    if self.verbose:
                        self.logger.info(F"rss_callback: rejected unreachable floor change ({self.floor}, {self.area}) -> ({floor}, {area})")
                    return
            else:
                self.logger.info("optimization_detected. change localization mode init->track")

            # set temporal variables
            target_floor = floor
            target_area = area
            target_mode = LocalizationMode.TRACK

            # check the availablity of local_pose on the target frame
            floor_manager = self.ble_localizer_dict[target_floor][target_area][target_mode]
            frame_id = floor_manager.frame_id  # target frame_id
            local_transform = None
            try:
                # tf from the origin of the target floor to the robot pose
                local_transform = tfBuffer.lookup_transform(frame_id, self.base_link_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.clock.clock_type), no_cache=True)
            except RuntimeError:
                self.logger.error(F'LookupTransform Error from {frame_id} to {self.base_link_frame}')

            # update the trajectory only when local_transform is available
            if local_transform is not None:
                if self.floor != floor:  # floor change
                    pass  # nothing to do
                else:
                    self.optimization_detected = False

                # create local_pose instance
                v3 = local_transform.transform.translation  # Vector3
                position = Point(x=v3.x, y=v3.y, z=v3.z)
                orientation = local_transform.transform.rotation  # Quaternion
                local_pose = Pose(position=position, orientation=orientation)

                # try to finish the current trajectory before updating state variables
                self.finish_trajectory()

                # update state variables to switch floor
                self.floor = target_floor
                self.area = target_area
                self.mode = target_mode

                # restart trajectory with the updated state variables
                self.restart_floor(local_pose)

        else:
            # skip if not activated
            if not rss_loc_params.failure_detection:
                self.logger.info(F"rss_callback({rss_type}): skipping failure detection", throttle_duration_sec=5.0)
                return

            # check localization failure
            try:
                t = tfBuffer.lookup_transform(self.global_map_frame, self.base_link_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.clock.clock_type))
                loc2D_track = np.array([t.transform.translation.x, t.transform.translation.y])
                loc2D_beacon = np.array([loc[0, 0], loc[0, 1]])
                failure_detected = self.check_localization_failure(loc2D_track, loc2D_beacon)
                if failure_detected and self.auto_relocalization:
                    self.restart_localization()
                    self.logger.error("Auto-relocalization. (localization failure detected)")
            except RuntimeError:
                self.logger.info(F"LookupTransform Error from {self.global_map_frame} to {self.base_link_frame}")

    # periodically check and update internal state variables (area and mode)
    def check_and_update_states(self):
        # check interval
        now = self.clock.now()
        if self.previous_area_check_time is not None:
            if Duration(seconds=self.area_check_interval) <= now - self.previous_area_check_time:
                self.previous_area_check_time = self.clock.now()
            else:
                return
        else:
            self.previous_area_check_time = self.clock.now()

        if self.verbose:
            self.logger.info(F"multi_floor_manager.check_and_update_states. (floor={self.floor}, mode={self.mode}")

        # timeout detection
        if self.initial_localization_time is not None:
            if now - self.initial_localization_time > Duration(seconds=self.initial_localization_timeout):
                self.initial_localization_timeout_detected = True
        if self.initial_localization_timeout_detected:
            if self.initial_localization_timeout_auto_relocalization:
                self.restart_localization()
                self.logger.warn("Auto-relocalization. (initial localization timeout detected)")
                return

        if self.floor is not None and self.mode is not None:
            # get robot pose
            try:
                robot_pose = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.clock.clock_type))
            except RuntimeError as e:
                self.logger.warn(F"{e}")
                return

            # detect area switching
            x_area = [[robot_pose.transform.translation.x, robot_pose.transform.translation.y, float(self.floor) * self.area_floor_const]]  # [x,y,floor]

            # find area candidates
            neigh_dist, neigh_ind = self.area_localizer.kneighbors(x_area, n_neighbors=10)
            area_candidates = self.Y_area[neigh_ind]

            # switch area when the detected area is stable
            unique_areas = np.unique(area_candidates)
            if len(unique_areas) == 1:
                area = unique_areas[0]
            else:
                area = self.area

            # if area change detected, switch trajectory
            if self.area != area \
                    or (self.mode == LocalizationMode.INIT and self.optimization_detected) \
                    or (self.mode == LocalizationMode.INIT and self.initial_localization_timeout_detected) \
                    or (self.mode == LocalizationMode.INIT and self.gnss_init_timeout_detected):

                if self.area != area:
                    self.logger.info(F"area change detected ({self.area} -> {area}).")
                elif (self.mode == LocalizationMode.INIT and self.optimization_detected):
                    self.logger.info("optimization_detected. change localization mode init->track")
                elif (self.mode == LocalizationMode.INIT and self.initial_localization_timeout_detected):
                    self.logger.info("initial localization timeout detected. change localization mode init->track")
                elif (self.mode == LocalizationMode.INIT and self.gnss_init_timeout_detected):
                    self.logger.info("gnss initial localization timeout detected. change localization mode init->track")

                # set temporal variables
                target_area = area
                target_mode = LocalizationMode.TRACK
                # check the availablity of local_pose on the target frame
                floor_manager = self.ble_localizer_dict[self.floor][target_area][target_mode]
                frame_id = floor_manager.frame_id  # target frame_id
                local_transform = None
                try:
                    # tf from the origin of the target floor to the robot pose
                    local_transform = tfBuffer.lookup_transform(frame_id, self.base_link_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.clock.clock_type))
                except RuntimeError:
                    self.logger.error('LookupTransform Error from ' + frame_id + " to " + self.base_link_frame)

                # update the trajectory only when local_transform is available
                if local_transform is not None:

                    # update condition variables
                    self.optimization_detected = False
                    self.initial_localization_timeout_detected = False
                    self.gnss_init_timeout_detected = False

                    # create local_pose instance
                    v3 = local_transform.transform.translation  # Vector3
                    position = Point(x=v3.x, y=v3.y, z=v3.z)
                    orientation = local_transform.transform.rotation  # Quaternion
                    local_pose = Pose(position=position, orientation=orientation)
                    # try to finish the current trajectory before updating state variables
                    self.finish_trajectory()
                    # update state variables to switch area
                    self.area = target_area
                    self.mode = target_mode
                    # restart trajectory with the updated state variables
                    self.restart_floor(local_pose)
        return

    # publish global position
    def global_position_callback(self):
        if not self.floor:
            return
        averaging_interval = self.global_position_averaging_interval
        try:
            # convert global position on global_map_frame to lat lng
            end_time = tfBuffer.get_latest_common_time(self.global_map_frame, self.global_position_frame)  # latest available time
            start_time = end_time - Duration(seconds=averaging_interval)
            trans_pos = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, end_time)
            xy = geoutil.Point(x=trans_pos.transform.translation.x, y=trans_pos.transform.translation.y)
            latlng = geoutil.local2global(xy, self.global_anchor)
            floor = self.floor
            # convert robot rotation to heading
            anchor_rotation = self.global_anchor.rotate  # degrees (0 -> north, clock-wise)
            euler_angles = tf_transformations.euler_from_quaternion([trans_pos.transform.rotation.x, trans_pos.transform.rotation.y, trans_pos.transform.rotation.z, trans_pos.transform.rotation.w], 'sxyz')
            yaw_angle = euler_angles[2]  # [roll, pitch, yaw] radien (0 -> x-axis, counter-clock-wise)
            heading = anchor_rotation + 90.0 - 180.0 * yaw_angle / math.pi  # added 90 degrees to convert y-axis to x-axis
            heading = heading % 360  # clip to the space of heading [0, 2pi]

            # velocity on odom_frame to prevent it from jumping]
            trans_vel_end = tfBuffer.lookup_transform(self.odom_frame, self.global_position_frame, end_time)
            trans_vel_start = tfBuffer.lookup_transform(self.odom_frame, self.global_position_frame, start_time)
            delta_x = trans_vel_end.transform.translation.x - trans_vel_start.transform.translation.x
            delta_y = trans_vel_end.transform.translation.y - trans_vel_start.transform.translation.y
            v_x = delta_x / averaging_interval
            v_y = delta_y / averaging_interval
            v_xy = math.sqrt(v_x**2 + v_y**2)

            # create and publishg a MFGlobalPosition message
            global_position = MFGlobalPosition()
            global_position.header.stamp = end_time.to_msg()
            global_position.header.frame_id = self.global_position_frame
            global_position.latitude = latlng.lat
            global_position.longitude = latlng.lng
            global_position.floor = int(floor)
            global_position.heading = heading
            global_position.speed = v_xy
            self.global_position_pub.publish(global_position)
        except RuntimeError as e:
            self.logger.info(F"LookupTransform Error {self.global_map_frame}-> {self.global_position_frame}. error=RuntimeError({e})")
        except tf2_ros.TransformException as e:
            self.logger.info(F"{e}")

    def stop_localization_callback(self, request, response):
        if not self.is_active:
            response.status.code = 1
            response.status.message = "Stop localization failed. (localization is aleady stopped.)"
            return response
        try:
            self.is_active = False
            self.finish_trajectory()
            self.reset_states()
            response.status.code = 0
            response.status.message = "Stopped localization."
        except:  # noqa: E722
            response.status.code = 1
            response.status.message = "Stop localization failed."
        return response

    def start_localization_callback(self, request, response):
        if self.is_active:
            response.status.code = 1
            response.status.message = "Start localization failed. (localization is aleady started.)"
            return response
        if request.floor == StartLocalization.Request.FLOOR_UNKNOWN:
            response.status.message = "Starting localization."
        else:
            self.floor = int(request.floor)
            response.status.message = f"Starting localization with floor={request.floor}."
        self.is_active = True
        response.status.code = 0

        return response

    def finish_trajectory(self):
        # try to finish the current trajectory
        floor_manager: FloorManager = self.ble_localizer_dict[self.floor][self.area][self.mode]

        # wait for services
        get_trajectory_states = floor_manager.get_trajectory_states
        self.logger.info(F"wait for {get_trajectory_states.srv_name} service")
        get_trajectory_states.wait_for_service()
        finish_trajectory = floor_manager.finish_trajectory
        self.logger.info(F"wait for {finish_trajectory.srv_name} service")
        finish_trajectory.wait_for_service()

        req = GetTrajectoryStates.Request()
        res0 = get_trajectory_states.call(req)
        self.logger.info(F"{res0}")
        last_trajectory_id = res0.trajectory_states.trajectory_id[-1]
        last_trajectory_state = res0.trajectory_states.trajectory_state[-1]  # uint8 -> int

        # finish trajectory only if the trajectory is active.
        if last_trajectory_state in [TrajectoryStates.ACTIVE]:
            trajectory_id_to_finish = last_trajectory_id
            req = FinishTrajectory.Request(trajectory_id=trajectory_id_to_finish)
            res1 = finish_trajectory.call(req)
            self.logger.info(F"{res1}")

        # reset floor_manager
        floor_manager.reset_states()

        # reset gnss adjuster and publish
        self.gnss_adjuster_dict[self.floor][self.area].reset()
        self.publish_map_frame_adjust_tf()

    def start_trajectory_with_pose(self, initial_pose: Pose):

        floor_manager: FloorManager = self.ble_localizer_dict[self.floor][self.area][self.mode]
        start_trajectory = floor_manager.start_trajectory
        self.logger.info(F"wait for {start_trajectory.srv_name} service")
        start_trajectory.wait_for_service()

        # start trajectory
        configuration_directory = floor_manager.configuration_directory
        configuration_basename = floor_manager.configuration_basename
        use_initial_pose = True
        relative_to_trajectory_id = 0

        # compute relative pose
        if floor_manager.trajectory_initial_pose is None:
            # trajectory query (for the first time)
            trajectory_query = floor_manager.trajectory_query
            self.logger.info(F"wait for {trajectory_query.srv_name} service")
            trajectory_query.wait_for_service()
            req = TrajectoryQuery.Request(
                trajectory_id=relative_to_trajectory_id
            )
            res: TrajectoryQuery.Response = trajectory_query.call(req)
            trajectory = res.trajectory
            trajectory_initial_pose: PoseStamped = trajectory[0]

            self.logger.info(F"trajectory {relative_to_trajectory_id} initial pose = {trajectory_initial_pose}")
            floor_manager.trajectory_initial_pose = trajectory_initial_pose

        # compute relative pose to trajectory initial pose
        relative_pose = compute_relative_pose(floor_manager.trajectory_initial_pose.pose, initial_pose)
        self.logger.info(F"converted initial_pose ({initial_pose}) to relative_pose ({relative_pose}) on trajectory {relative_to_trajectory_id}")

        self.logger.info("prepare request")
        req = StartTrajectory.Request(
            configuration_directory=configuration_directory,
            configuration_basename=configuration_basename,
            use_initial_pose=use_initial_pose,
            initial_pose=relative_pose,
            relative_to_trajectory_id=relative_to_trajectory_id)
        res2 = start_trajectory.call(req)
        self.logger.info(F"start_trajectory response = {res2}")
        status_code = res2.status.code

        tfBuffer.clear()  # clear buffered tf to avoid the effect of the finished trajectory

        return status_code

    def reset_states(self):
        self.floor = None
        self.area = None
        self.current_frame = None
        self.mode = None
        self.map2odom = None
        self.optimization_detected = False

        # timeout
        self.initial_localization_time = None
        self.initial_localization_timeout_detected = False

        # gnss
        self.gnss_init_time = None
        self.gnss_init_timeout_detected = False
        self.gnss_localization_time = None
        self.gnss_navsat_time = None
        if self.gnss_is_active:
            self.indoor_outdoor_mode = IndoorOutdoorMode.UNKNOWN
        else:
            self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR

        self.floor_queue = []
        self.altitude_floor_estimator.reset()

        self.spin_count = 0
        self.prev_spin_count = None

        tfBuffer.clear()  # clear buffered tf added by finished trajectories
        self.localize_status = MFLocalizeStatus.UNKNOWN

    def restart_localization(self, floor_value=None):
        self.is_active = False
        self.finish_trajectory()
        self.reset_states()
        if floor_value is not None:
            self.floor = floor_value
        self.is_active = True

    def restart_localization_callback(self, request, response):
        try:
            response.status.code = 0
            if request.floor == RestartLocalization.Request.FLOOR_UNKNOWN:
                self.restart_localization()
                response.status.message = "Restarting localization..."
            else:
                self.restart_localization(floor_value=int(request.floor))
                response.status.message = f"Restarting localization with floor={request.floor}..."
            response.status.code = 0
        except Exception as e:  # noqa: E722
            response.status.code = 1
            response.status.message = f"Restart localization failed with exception = {e}."
        return response

    def enable_relocalization_callback(self, request, response):
        self.auto_relocalization = True
        response.status.code = 0
        response.status.message = "Enabled auto relocalization."
        return response

    def disable_relocalization_callback(self, request, response):
        self.auto_relocalization = False
        response.status.code = 0
        response.status.message = "Disabled auto relocalization."
        return response

    def reset_gnss_callback(self, request, response):
        self.logger.info("reset_gnss_callback")

        send_cfg_rst_req = SendCfgRST.Request()
        self.logger.info(f"request={send_cfg_rst_req}")

        # service call with timeout
        event = threading.Event()

        def unblock(future) -> None:
            nonlocal event
            event.set()

        future = seng_cfg_rst_client.call_async(send_cfg_rst_req)
        future.add_done_callback(unblock)

        if not future.done():
            if not event.wait(self.gnss_params.gnss_reset_timeout):
                seng_cfg_rst_client.remove_pending_request(future)
                response.status.code = 1
                response.status.message = "Failed to reset GNSS (Timeout)."
                self.logger.error(f"response={response}")
                return response

        exception = future.exception()
        if exception is not None:
            response.status.code = 1
            response.status.message = f"Failed to reset GNSS (Exception={exception})."
            self.logger.error(f"response={response}")
            return response

        success = future.result().success
        if success:
            response.status.code = 0
            response.status.message = "Succeeded to reset GNSS."
        else:
            response.status.code = 1
            response.status.message = "Failed to reset GNSS."
        self.logger.info(f"response={response}")
        return response

    def set_current_floor_callback(self, request, response):
        floor = int(request.data)
        if self.floor is None:
            self.floor = floor
            response.status.code = 0
            response.status.message = F"Set floor to {floor}."
        else:
            response.status.code = 1
            response.status.message = F"Failed to set floor to {floor}. Floor is already set to {self.floor}."
        return response

    # return
    #      MFGlobalPosition global_position
    # input:
    #      MFLocalPosition local_position
    def convert_local_to_global_callback(self, request, response):
        averaging_interval = self.global_position_averaging_interval  # noqa: F841
        try:
            pos = PointStamped()
            vel = Vector3Stamped()
            pos.header = request.local_position.header
            vel.header = request.local_position.header
            pos.point = request.local_position.position
            vel.vector = request.local_position.velocity

            transformed_position_stamped = tfBuffer.transform(pos, self.global_map_frame, timeout=Duration(seconds=1.0))  # timeout 1.0 s
            transformed_velocity_stamped = tfBuffer.transform(vel, self.global_map_frame, timeout=Duration(seconds=1.0))  # timeout 1.0 s

            # point to latlng
            xy = geoutil.Point(x=transformed_position_stamped.point.x, y=transformed_position_stamped.point.y)
            latlng = geoutil.local2global(xy, self.global_anchor)

            floor = self.floor

            # velocity vector to heading and speed
            speed = np.sqrt(transformed_velocity_stamped.vector.x ** 2 + transformed_velocity_stamped.vector.y ** 2)
            yaw_angle = np.arctan2(transformed_velocity_stamped.vector.y, transformed_velocity_stamped.vector.x)  # heading angle
            anchor_rotation = self.global_anchor.rotate  # degrees (0 -> north, clock-wise)
            heading = anchor_rotation + 90.0 - 180.0*yaw_angle / math.pi  # added 90 degrees to convert y-axis to x-axis
            heading = heading % 360  # clip to the space of heading [0, 2pi]

            # create a
            response.global_position.header.stamp = transformed_position_stamped.header.stamp
            response.global_position.header.frame_id = self.global_position_frame
            response.global_position.latitude = latlng.lat
            response.global_position.longitude = latlng.lng
            response.global_position.floor = floor
            response.global_position.heading = heading
            response.global_position.speed = speed
            return response
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.logger.info(F"LookupTransform Error {self.global_map_frame} -> {self.global_position_frame}")
            return None
        except tf2_ros.TransformException:
            return None

    def navsat_callback(self, msg: NavSAT):
        self.prev_navsat_msg = msg

        filtered_num_sv = 0
        for s in msg.sv:
            if self.gnss_params.gnss_fix_filter_min_cno <= s.cno \
                    and self.gnss_params.gnss_fix_filter_min_elev <= s.elev:
                filtered_num_sv += 1

        if self.gnss_params.gnss_fix_filter_num_sv <= filtered_num_sv:
            self.prev_navsat_status = True
        else:
            self.prev_navsat_status = False

        self.logger.info(F"multi_floor_manager.navsat_callback: filtered_num_sv={filtered_num_sv}")

    def mf_navsat_callback(self, msg: MFNavSAT):
        self.prev_mf_navsat_msg = msg

    def estimateRt(self, X, Y):
        """
        estimate rotation (R) and translation (t) from 2D point corespondances (y = [R|t]x)
        input: X(n_samples, 2), Y(n_samples, 2)
        """
        Xmean = np.mean(X, axis=0)
        Ymean = np.mean(Y, axis=0)

        Xdev = X - Xmean
        Ydev = Y - Ymean

        H = np.dot(np.transpose(Xdev), Ydev)
        U, S, Vt = np.linalg.svd(H)

        # rotation
        R = np.dot(Vt.T, U.T)
        # M = np.eye(len(R))
        if np.linalg.det(R) < 0.0:  # check reflection
            # M[len(R)-1, len(R)-1] = -1.0
            Vt[-1, :] = -Vt[-1, :]
            S[-1] = -S[-1]

        R = np.dot(Vt.T, U.T)

        # translation
        t = - np.dot(R, Xmean.T) + Ymean.T

        return R, t

    # input:
    #      NavSatFix gnss_fix
    #      TwistWithCovarianceStamped gnss_fix_velocity
    def gnss_fix_callback(self, fix: NavSatFix, fix_velocity: TwistWithCovarianceStamped):
        # start converting gnss_fix message to global_map_frame and publish it for visualization
        # read message
        now = self.clock.now()
        stamp = fix.header.stamp
        gnss_frame = fix.header.frame_id
        latitude = fix.latitude
        longitude = fix.longitude
        position_covariance = fix.position_covariance

        # calculate moving direction from fix_velocity
        vel_e = fix_velocity.twist.twist.linear.x
        vel_n = fix_velocity.twist.twist.linear.y
        # speed = np.sqrt(vel_e**2 + vel_n**2)  # comment out by daisukes, speed is not used
        heading = np.arctan2(vel_n, vel_e)  # yaw angle
        heading_degree = 90.0 - 180.0*heading/math.pi  # added 90 degrees to convert y-axis to x-axis
        heading_degree = heading_degree % 360  # clip to the space of heading [0, 2pi]

        frame_id = self.global_map_frame
        anchor = self.global_anchor

        # lat,lng -> x,y
        latlng = geoutil.Latlng(lat=latitude, lng=longitude)
        gnss_xy = geoutil.global2local(latlng, anchor)

        # heading -> yaw
        anchor_rotation = anchor.rotate
        yaw_degrees = anchor_rotation + 90.0 - heading_degree
        yaw_degrees = yaw_degrees % 360
        gnss_yaw = np.radians(yaw_degrees)

        # pose covariance
        covariance_matrix = np.zeros((6, 6))
        covariance_matrix[0:3, 0:3] = np.reshape(position_covariance, (3, 3))
        # TODO: orientation covariance

        if self.floor is None:
            # estimate floor if enabled
            if self.gnss_params.gnss_use_floor_estimation:
                near_floor_list = self.floor_height_mapper.get_floor_list([gnss_xy.x, gnss_xy.y],
                                                                          radius=self.gnss_params.gnss_floor_search_radius)
                if len(near_floor_list) == 0:
                    floor_raw = 0  # assume ground floor
                    idx_floor = np.abs(np.array(self.floor_list) - floor_raw).argmin()
                    floor = self.floor_list[idx_floor]  # select from floor_list to prevent using an unregistered value.
                    self.logger.info(f"gnss_fix_callback: floor_est={floor}, near_floor_list={near_floor_list}")
                else:
                    max_floor = np.max(near_floor_list)
                    near_floor_array = np.array(near_floor_list)
                    probs_floor = np.exp(self.gnss_params.gnss_floor_estimation_probability_scale * (near_floor_array - max_floor))
                    probs_floor /= np.sum(probs_floor)
                    self.gnss_balanced_floor_sampler.update(near_floor_array, p=probs_floor)
                    _floor_selected = False
                    if self.gnss_params.gnss_status_floor_selection:
                        if self.gnss_params.gnss_floor_selection_status_threhold <= fix.status.status and \
                                self.gnss_balanced_floor_sampler.selectable(max_floor):
                            floor = self.gnss_balanced_floor_sampler.select(max_floor)
                            _floor_selected = True
                        else:
                            floor = self.gnss_balanced_floor_sampler.sample()
                    else:
                        floor = self.gnss_balanced_floor_sampler.sample()
                    self.logger.info(f"gnss_fix_callback: floor_est={floor} (selected={_floor_selected}), near_floor_list={near_floor_list}, probs_floor={probs_floor}")
            else:
                floor_raw = 0  # assume ground floor
                idx_floor = np.abs(np.array(self.floor_list) - floor_raw).argmin()
                floor = self.floor_list[idx_floor]  # select from floor_list to prevent using an unregistered value.
                self.logger.info(f"gnss_fix_callback: floor_est={floor}, floor_raw={floor_raw}, floor_list={self.floor_list}")
        else:
            floor = self.floor

        # x,y,yaw -> PoseWithCovarianceStamped message
        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.header.stamp = now.to_msg()
        pose_with_covariance_stamped.header.frame_id = frame_id
        pose_with_covariance_stamped.pose.pose.position.x = gnss_xy.x
        pose_with_covariance_stamped.pose.pose.position.y = gnss_xy.y
        pose_with_covariance_stamped.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, gnss_yaw, 'sxyz')
        pose_with_covariance_stamped.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        pose_with_covariance_stamped.pose.covariance = covariance_matrix.flatten()

        fix_local = [now.nanoseconds/1e9, gnss_xy.x, gnss_xy.y, gnss_yaw]  # timestamp, x, y, yaw

        # publish gnss fix in local frame
        self.gnss_fix_local_pub.publish(pose_with_covariance_stamped)

        # do not update internal states when the multi_floor_manager is not active
        if not self.is_active:
            return

        # set gnss_navsat_time for timeout detection
        if self.gnss_navsat_time is None:
            self.gnss_navsat_time = now
        # update indoor / outdoor status by using navsat status
        if self.prev_mf_navsat_msg is not None:
            sv_status = self.prev_mf_navsat_msg.sv_status
            if self.indoor_outdoor_mode == IndoorOutdoorMode.UNKNOWN:  # at start up
                if sv_status == MFNavSAT.STATUS_INACTIVE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
                    self.logger.info("gnss_fix_callback: indoor_outdoor_mode = UNKNOWN -> INDOOR (STATUS_INACTIVE)")
                elif sv_status == MFNavSAT.STATUS_INTERMEDIATE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
                    self.logger.info("gnss_fix_callback: indoor_outdoor_mode = UNKNOWN -> INDOOR (STATUS_INTERMEDIATE)")
                else:  # sv_status == MFNavSAT.STATUS_ACTIVE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.OUTDOOR
                    self.logger.info("gnss_fix_callback: indoor_outdoor_mode = UNKNOWN -> OUTDOOR (STATUS_ACTIVE)")
            elif self.indoor_outdoor_mode == IndoorOutdoorMode.INDOOR:
                if sv_status == MFNavSAT.STATUS_ACTIVE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.OUTDOOR
                    self.logger.info("gnss_fix_callback: indoor_outdoor_mode = INDOOR -> OUTDOOR (STATUS_ACTIVE)")
            else:  # self.indoor_outdoor_mode == IndoorOutdoorMode.OUTDOOR:
                if sv_status == MFNavSAT.STATUS_INACTIVE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
                    self.logger.info("gnss_fix_callback: indoor_outdoor_mode = OUTDOOR -> INDOOR (STATUS_INACTIVE)")
            self.prev_mf_navsat_msg = None
            self.gnss_navsat_time = now

        # disable gnss adjust in indoor invironments
        if self.indoor_outdoor_mode == IndoorOutdoorMode.INDOOR:
            # reset all gnss adjust
            for _floor in self.gnss_adjuster_dict.keys():
                for _area in self.gnss_adjuster_dict[_floor].keys():
                    self.gnss_adjuster_dict[_floor][_area].reset()

        # use different covariance threshold for initial localization and tracking
        if (self.gnss_localization_time is None) and (self.mode is None):
            # initial localization
            fix_rejection_position_covariance = self.gnss_params.gnss_position_covariance_initial_threshold
        else:
            fix_rejection_position_covariance = self.gnss_params.gnss_position_covariance_threshold
        # do not use unreliable gnss fix
        if fix.status.status == NavSatStatus.STATUS_NO_FIX \
                or fix_rejection_position_covariance < fix.position_covariance[0]:
            # log before gnss-based initial localization
            if self.gnss_localization_time is None:
                self.logger.info(F"gnss_fix_callback: waiting for position_covariance convergence (cov[0]={fix.position_covariance[0]}, covariance_threshold={fix_rejection_position_covariance})",
                                 throttle_duration_sec=1.0)
            # if navsat topic is timeout, use position covariance for indoor/outdoor mode instead
            if now - self.gnss_navsat_time > Duration(seconds=self.gnss_params.gnss_navsat_timeout):
                if self.gnss_params.gnss_position_covariance_indoor_threshold < fix.position_covariance[0]:
                    if self.indoor_outdoor_mode != IndoorOutdoorMode.INDOOR:
                        self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
                        self.logger.info(F"gnss_fix_callback: indoor_outdoor_mode = INDOOR (thredhold={self.gnss_params.gnss_position_covariance_indoor_threshold} < cov[0]={fix.position_covariance[0]})")
            # return if gnss fix is unreliable
            return
        else:
            # set outdoor mode if gnss fix is reliable
            if self.indoor_outdoor_mode != IndoorOutdoorMode.OUTDOOR:
                self.indoor_outdoor_mode = IndoorOutdoorMode.OUTDOOR
                self.logger.info(F"gnss_fix_callback: indoor_outdoor_mode = OUTDOOR (cov[0]={fix.position_covariance[0]} <= threshold={fix_rejection_position_covariance})")

        # do not start trajectories when gnss is not active
        if not self.gnss_is_active:
            return

        # publish gnss fix to localizer node
        if self.floor is not None and self.area is not None and self.mode is not None:
            floor_manager = self.ble_localizer_dict[self.floor][self.area][self.mode]
            if fix.status.status >= self.gnss_params.gnss_status_threshold \
                    and self.prev_navsat_status:
                fix_pub = floor_manager.fix_pub
                fix.header.stamp = now.to_msg()  # replace gnss timestamp with ros timestamp for rough synchronization

                # publish fix topic only when the distance travelled exceeds a certain level to avoid adding too many constraints
                if floor_manager.previous_fix_local_published is None:
                    fix_pub.publish(fix)
                    floor_manager.previous_fix_local_published = fix_local
                else:
                    prev_fix_local = floor_manager.previous_fix_local_published
                    distance_fix_local = np.sqrt((fix_local[1] - prev_fix_local[1])**2 + (fix_local[2] - prev_fix_local[2])**2)
                    if self.gnss_params.gnss_fix_motion_filter_distance <= distance_fix_local:
                        fix_pub.publish(fix)
                        floor_manager.previous_fix_local_published = fix_local

                # check initial pose optimization timeout in reliable gnss fix loop
                if self.gnss_init_time is not None:
                    if now - self.gnss_init_time > Duration(seconds=self.gnss_init_timeout):
                        self.gnss_init_time = None
                        self.gnss_init_timeout_detected = True

        # Forcibly prevent the tracked trajectory from going far away from the gnss position history
        track_error_detected = False
        # Prevent too large adjust
        large_adjust_detected = False

        update_gnss_adjust = False
        gnss_adjust_x = None
        gnss_adjust_y = None
        gnss_adjust_yaw = None
        may_stable_Rt = False

        tf_available = False

        # lookup tf
        try:
            # position on global_map_frame
            end_time = tfBuffer.get_latest_common_time(self.global_map_frame, gnss_frame)  # latest available time
            # robot pose
            trans_pos = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, end_time)
            # gnss pose
            transform_gnss = tfBuffer.lookup_transform(self.global_map_frame, gnss_frame, end_time)

            # get tf required to compute gnss adjust
            # gnss adjuster
            gnss_adjuster = self.gnss_adjuster_dict[self.floor][self.area]
            tf_global2local = tfBuffer.lookup_transform(self.global_map_frame, gnss_adjuster.frame_id, end_time)
            tf_adjust2odom = tfBuffer.lookup_transform(gnss_adjuster.map_frame_adjust, self.odom_frame, end_time)
            tf_odom2gnss = tfBuffer.lookup_transform(self.odom_frame, gnss_frame, end_time)

            tf_available = True
        except RuntimeError as e:
            self.logger.info(F"LookupTransform Error {self.global_map_frame} -> {gnss_frame}. error=RuntimeError({e})")
        except tf2_ros.TransformException as error:
            self.logger.info(F"{error=}")
        except KeyError as error:
            self.logger.info(F"{error=}")

        if tf_available:
            # robot pose
            euler_angles_pos = tf_transformations.euler_from_quaternion([trans_pos.transform.rotation.x, trans_pos.transform.rotation.y, trans_pos.transform.rotation.z, trans_pos.transform.rotation.w], 'sxyz')
            yaw_pos = euler_angles_pos[2]  # [roll, pitch, yaw] radien (0 -> x-axis, counter-clock-wise

            # gnss pose
            euler_angles_gnss = tf_transformations.euler_from_quaternion([transform_gnss.transform.rotation.x, transform_gnss.transform.rotation.y, transform_gnss.transform.rotation.z, transform_gnss.transform.rotation.w], 'sxyz')
            yaw_angle = euler_angles_gnss[2]  # [roll, pitch, yaw] radien (0 -> x-axis, counter-clock-wise)

            # calculate error
            xy_error = np.linalg.norm([transform_gnss.transform.translation.x - gnss_xy.x, transform_gnss.transform.translation.y - gnss_xy.y])
            cosine = np.cos(gnss_yaw)*np.cos(yaw_angle) + np.sin(gnss_yaw)*np.sin(yaw_angle)
            cosine = np.clip(cosine, -1.0, 1.0)
            # yaw_error = np.arccos(cosine)
            # self.logger.info("xy_error="+str(xy_error)+", yaw_error="+str(yaw_error))

            # gnss adjuster
            gnss_adjuster = self.gnss_adjuster_dict[self.floor][self.area]

            # estimate map_local -> map_adjust
            # global_map_frame -> local_map_frame (gnss_adjuster.frame_id) -> map_frame_adjust -> odom_frame -> base_link -> ... -> gnss

            euler_global2local = tf_transformations.euler_from_quaternion([tf_global2local.transform.rotation.x, tf_global2local.transform.rotation.y, tf_global2local.transform.rotation.z, tf_global2local.transform.rotation.w], 'sxyz')
            euler_adjust2odom = tf_transformations.euler_from_quaternion([tf_adjust2odom.transform.rotation.x, tf_adjust2odom.transform.rotation.y, tf_adjust2odom.transform.rotation.z, tf_adjust2odom.transform.rotation.w], 'sxyz')
            euler_odom2gnss = tf_transformations.euler_from_quaternion([tf_odom2gnss.transform.rotation.x, tf_odom2gnss.transform.rotation.y, tf_odom2gnss.transform.rotation.z, tf_odom2gnss.transform.rotation.w], 'sxyz')

            global2local = [stamp.nanosec/1e9, tf_global2local.transform.translation.x, tf_global2local.transform.translation.y, euler_global2local[2]]
            adjust2odom = [stamp.nanosec/1e9, tf_adjust2odom.transform.translation.x, tf_adjust2odom.transform.translation.y, euler_adjust2odom[2]]
            odom2gnss = [stamp.nanosec/1e9, tf_odom2gnss.transform.translation.x, tf_odom2gnss.transform.translation.y, euler_odom2gnss[2]]

            Tglobal2local = toTransmat(global2local[1], global2local[2], global2local[3])
            Tadjust2odom = toTransmat(adjust2odom[1], adjust2odom[2], adjust2odom[3])

            # jump detection and small motion filtering
            if len(gnss_adjuster.gnss_fix_list) > 0:
                diff = np.linalg.norm([gnss_adjuster.local_odom_list[-1][1] - odom2gnss[1],
                                       gnss_adjuster.local_odom_list[-1][2] - odom2gnss[2]])
                # self.logger.info(F"diff={diff}")
                if diff > self.gnss_params.gnss_odom_jump_threshold:  # if the local slam is jumped
                    self.logger.info(F"detected local slam jump. diff={diff}.")
                    gnss_adjuster.gnss_fix_list = []
                    gnss_adjuster.local_odom_list = []
                    gnss_adjuster.gnss_fix_list.append(fix_local)
                    gnss_adjuster.local_odom_list.append(odom2gnss)
                    gnss_adjuster.gnss_total_count += 1
                    update_gnss_adjust = True
                elif diff <= self.gnss_params.gnss_odom_small_threshold:  # if the movement is too small.
                    pass
                else:
                    gnss_adjuster.gnss_fix_list.append(fix_local)
                    gnss_adjuster.local_odom_list.append(odom2gnss)
                    gnss_adjuster.gnss_total_count += 1
                    update_gnss_adjust = True
            else:
                gnss_adjuster.gnss_fix_list.append(fix_local)
                gnss_adjuster.local_odom_list.append(odom2gnss)
                gnss_adjuster.gnss_total_count += 1
                update_gnss_adjust = True

            def apply_deadzone(x, deadzone):
                x_new = 0.0
                if deadzone <= x:
                    x_new = x - deadzone
                elif x <= -deadzone:
                    x_new = x + deadzone
                return x_new

            # estimate gnss_adjust
            if update_gnss_adjust:
                if 2 <= len(gnss_adjuster.gnss_fix_list):  # use at least two points to calculate R, t

                    W = []  # odom_frame -> gnss_frame
                    Z = []  # global_map_frame -> gnss_frame

                    for idx in range(np.min([self.gnss_params.gnss_n_max_correspondences, len(gnss_adjuster.gnss_fix_list)])):
                        W.append([gnss_adjuster.local_odom_list[-idx-1][1], gnss_adjuster.local_odom_list[-idx-1][2], 1])
                        Z.append([gnss_adjuster.gnss_fix_list[-idx-1][1], gnss_adjuster.gnss_fix_list[-idx-1][2], 1])

                    W = np.array(W)
                    Z = np.array(Z)

                    X = W @ Tadjust2odom.transpose()
                    Y = (np.linalg.inv(Tglobal2local) @ Z.transpose()).transpose()

                    R, t = self.estimateRt(X[:, :2], Y[:, :2])

                    gnss_adjust_x = t[0]
                    gnss_adjust_y = t[1]
                    gnss_adjust_yaw = np.arctan2(R[1, 0], R[0, 0])  # [-pi, pi]

                    # apply dead zone
                    gnss_adjust_x = apply_deadzone(gnss_adjust_x, self.gnss_params.gnss_track_error_adjust)
                    gnss_adjust_y = apply_deadzone(gnss_adjust_y, self.gnss_params.gnss_track_error_adjust)

                    # apply zero adjust weight
                    if gnss_adjuster.zero_adjust_uncertainty < 1.0:
                        alpha = gnss_adjuster.zero_adjust_uncertainty
                        gnss_adjust_x = (1.0-alpha)*0.0 + alpha*gnss_adjust_x
                        gnss_adjust_y = (1.0-alpha)*0.0 + alpha*gnss_adjust_y
                        gnss_adjust_yaw = (1.0-alpha)*0.0 + alpha*gnss_adjust_yaw
                        # update zero_adjust_uncertainty
                        gnss_adjuster.zero_adjust_uncertainty += 1.0/self.gnss_params.gnss_n_max_correspondences
                        gnss_adjuster.zero_adjust_uncertainty = np.min([gnss_adjuster.zero_adjust_uncertainty, 1.0])  # clipping

                    # update gnss adjust values when the uncertainty of those values is high (initial stage) or those values are very stable (tracking stage).
                    may_stable_Rt = self.gnss_params.gnss_n_min_correspondences_stable <= len(gnss_adjuster.gnss_fix_list)
                    if gnss_adjuster.gnss_total_count < self.gnss_params.gnss_n_min_correspondences_stable \
                            or may_stable_Rt:
                        if gnss_adjuster.adjust_tf:
                            gnss_adjuster.gnss_adjust_x = gnss_adjust_x
                            gnss_adjuster.gnss_adjust_y = gnss_adjust_y
                            gnss_adjuster.gnss_adjust_yaw = gnss_adjust_yaw
                            self.logger.info(F"gnss_adjust updated: gnss_adjust_x={gnss_adjust_x}, gnss_adjust_y={gnss_adjust_y}, gnss_adjust_yaw={gnss_adjust_yaw}")
                        else:
                            self.logger.info(F"gnss_adjust NOT updated: gnss_adjust_x={gnss_adjust_x}, gnss_adjust_y={gnss_adjust_y}, gnss_adjust_yaw={gnss_adjust_yaw}")

            if np.sqrt(position_covariance[0]) + self.gnss_params.gnss_track_error_threshold <= xy_error:
                self.logger.info(F"gnss tracking error detected. sqrt(position_covariance[0])={np.sqrt(position_covariance[0])} + gnss_track_error_threshold={self.gnss_params.gnss_track_error_threshold} < xy_error={xy_error}")
                track_error_detected = True

            if may_stable_Rt:
                if np.sqrt(position_covariance[0]) + self.gnss_params.gnss_track_error_threshold <= np.sqrt(gnss_adjuster.gnss_adjust_x**2 + gnss_adjuster.gnss_adjust_y**2) \
                        or self.gnss_params.gnss_track_yaw_threshold < np.abs(gnss_adjuster.gnss_adjust_yaw):
                    self.logger.info("gnss map adjustment becomes too large.")
                    large_adjust_detected = True

            # update pose for reset
            if may_stable_Rt:
                q = tf_transformations.quaternion_from_euler(0, 0, yaw_pos, 'sxyz')
                pose_with_covariance_stamped.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # publish (possibly) updated map adjust
        reset_trajectory = False
        reset_zero_adjust_uncertainty = False  # set zero adjust uncertainty to 1 (unknown) when gnss adjust is completely unknown (e.g. initialization, large error with estimated gnss adjust)

        if self.floor is None:  # run one time
            # set floor before initialize_with_global_pose
            self.floor = floor
            reset_trajectory = True
            reset_zero_adjust_uncertainty = True
        elif self.floor is not None and \
                self.area is None:
            # self.floor is specified but area and mode are unknown
            reset_trajectory = True
            reset_zero_adjust_uncertainty = True
        elif track_error_detected:
            reset_trajectory = True
            reset_zero_adjust_uncertainty = True
        elif large_adjust_detected:
            reset_trajectory = True
        else:
            reset_trajectory = False

        if not reset_trajectory:
            return

        if self.gnss_localization_time is not None:
            if now - self.gnss_localization_time < Duration(seconds=self.gnss_params.gnss_localization_interval):
                return

        # start localization
        target_mode = None
        if self.mode is None:
            target_mode = LocalizationMode.INIT
            self.mode = target_mode
            self.gnss_init_time = now
        elif self.mode == LocalizationMode.INIT \
                and may_stable_Rt:  # change mode INIT -> TRACK if mode has not been updated by optimization
            target_mode = LocalizationMode.TRACK

        # if floor, area, and mode are active, try to finish the trajectory.
        if self.area is not None:
            # preprocessing to prevent TF tree from jumping in a short moment after publishing map -> map_adjust and before resetting a trajectory.

            # finish trajectory to stop publishing map_adjust -> ... -> published_frame TF
            self.finish_trajectory()

            gnss_adjuster = self.gnss_adjuster_dict[self.floor][self.area]

            # temporarily cut TF tree between map_adjust -> odom_frame
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = multi_floor_manager.unknown_frame
            t.child_frame_id = multi_floor_manager.odom_frame
            t.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
            t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            broadcaster.sendTransform([t])

        if target_mode is not None:
            self.mode = target_mode  # set mode after finishing old trajectory

        # reset all gnss adjust
        for floor in self.gnss_adjuster_dict.keys():
            for area in self.gnss_adjuster_dict[floor].keys():
                gnss_adjuster = self.gnss_adjuster_dict[floor][area]
                gnss_adjuster.reset()
                if reset_zero_adjust_uncertainty:
                    gnss_adjuster.zero_adjust_uncertainty = 1.0

        # self.publish_map_frame_adjust_tf()
        # here, map_adjust -> ... -> published_frame TF must be disabled.
        self.initialize_with_global_pose(pose_with_covariance_stamped)  # reset pose on the global frame
        self.gnss_localization_time = now

    def publish_map_frame_adjust_tf(self):
        # prevent publishing redundant tf
        stamp = self.clock.now().to_msg()
        if self.prev_publish_map_frame_adjust_timestamp is not None:
            if self.prev_publish_map_frame_adjust_timestamp == stamp:
                return

        transform_list = []
        for floor in self.ble_localizer_dict.keys():
            for area in self.ble_localizer_dict[floor].keys():
                gnss_adjuster = self.gnss_adjuster_dict[floor][area]

                parent_frame_id = gnss_adjuster.frame_id
                child_frame_id = gnss_adjuster.map_frame_adjust

                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = parent_frame_id
                t.child_frame_id = child_frame_id
                t.transform.translation = Vector3(x=gnss_adjuster.gnss_adjust_x, y=gnss_adjuster.gnss_adjust_y, z=0.0)
                q = tf_transformations.quaternion_from_euler(0.0, 0.0, gnss_adjuster.gnss_adjust_yaw, 'sxyz')
                t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                transform_list.append(t)

        broadcaster.sendTransform(transform_list)
        self.prev_publish_map_frame_adjust_timestamp = stamp

    def map_frame_adjust_callback(self):
        self.publish_map_frame_adjust_tf()

    # input:
    #      MFLocalPosition global_position
    def global_localizer_global_pose_callback(self, msg):
        self.logger.info(F"received global_pose from global_localizer: global_pose={msg}")

        stamp = msg.header.stamp
        latitude = msg.latitude
        longitude = msg.longitude
        floor_val = msg.floor  # noqa: F841
        heading = msg.heading

        frame_id = self.global_map_frame
        anchor = self.global_anchor

        # lat,lng -> x,y
        latlng = geoutil.Latlng(lat=latitude, lng=longitude)
        xy = geoutil.global2local(latlng, anchor)

        # heading -> yaw
        anchor_rotation = anchor.rotate
        yaw_degrees = anchor_rotation + 90.0 - heading
        yaw_degrees = yaw_degrees % 360
        yaw = np.radians(yaw_degrees)

        # x,y,yaw -> PoseWithCovarianceStamped message
        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.header.stamp = stamp
        pose_with_covariance_stamped.header.frame_id = frame_id
        pose_with_covariance_stamped.pose.pose.position.x = xy.x
        pose_with_covariance_stamped.pose.pose.position.y = xy.y
        pose_with_covariance_stamped.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, yaw, 'sxyz')
        pose_with_covariance_stamped.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # set floor before initialize_with_global_pose
        self.floor = floor

        # start localization
        self.initialize_with_global_pose(pose_with_covariance_stamped)
        self.is_active = True

    # check if pose graph is optimized
    def is_optimized(self):
        if self.mode != LocalizationMode.INIT:
            self.logger.info(f"localization mode is not init. mode={self.mode}")
            return False

        floor_manager: FloorManager = self.ble_localizer_dict[self.floor][self.area][self.mode]
        read_metrics = floor_manager.read_metrics
        self.logger.info(F"wait for {read_metrics.srv_name} service")
        read_metrics.wait_for_service()

        req = ReadMetrics.Request()
        self.logger.info("request read_metrics")
        res = read_metrics.call(req)

        if res.status.code != 0:  # OK
            self.logger.info(f"read_metrics fails {res.status}")
            return False

        # comment out the following two lines to debug
        # from rosidl_runtime_py import message_to_yaml
        # self.logger.info(message_to_yaml(res))

        # monitor mapping_2d_pose_graph_constraints -> inter_submap -> different trajectory to detect inter trajectory pose graph optimization
        # because this value is updated after running optimization
        optimized = False
        for metric_family in res.metric_families:
            if metric_family.name == "mapping_2d_pose_graph_constraints":
                for metric in metric_family.metrics:
                    # inter_submap -> different trajectory
                    if metric.labels[0].key == "tag" and metric.labels[0].value == "inter_submap" \
                            and metric.labels[1].key == "trajectory" and metric.labels[1].value == "different":
                        self.logger.info(f"inter_submap different trajectory constraints. count={metric.value}")
                        # check if the number of constraints changed
                        if floor_manager.constraints_count != metric.value:
                            floor_manager.constraints_count = metric.value
                            if metric.value >= floor_manager.min_hist_count:
                                optimized = True
                                self.logger.info("pose graph optimization detected.")
        return optimized


'''
class CurrentPublisher:
    def __init__(self, node, verbose=False):
        self.node = node
        self.verbose = False
        self.publish_current_rate = self.node.declare_parameter("publish_current_rate", 0).value  # 0 for latch
        self.current_floor = None
        self.current_frame = None
        self.current_map_filename = None
        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Int64, "current_floor", self.current_floor_cb, latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.node.create_subscription(String, "current_frame", self.current_frame_cb, latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.node.create_subscription(String, "current_map_filename", self.current_map_filename_cb, latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub_floor = self.node.create_publisher(Int64, "current_floor", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub_frame = self.node.create_publisher(String, "current_frame", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub_map = self.node.create_publisher(String, "current_map_filename", latched_qos, callback_group=MutuallyExclusiveCallbackGroup())

        if self.publish_current_rate == 0:
            if self.verbose:
                self.logger.info("node will not publish current regularly (publish_current_rate = 0)")
            return
        self.current_timer = self.node.create_timer(1.0 / self.publish_current_rate, self.publish_current)

    def current_floor_cb(self, msg):
        self.current_floor = msg

    def current_frame_cb(self, msg):
        self.current_frame = msg

    def current_map_filename_cb(self, msg):
        self.current_map_filename = msg

    def publish_current(self):
        if self.verbose:
            self.logger.info("node will publish current regularly (publish_current_rate = {})".format(self.publish_current_rate))

        if rclpy.ok():
            # publish
            if self.current_floor is not None:
                if self.verbose:
                    self.logger.info(F"current_floor = {self.current_floor.data}")
                self.pub_floor.publish(self.current_floor)

            if self.current_frame is not None:
                if self.verbose:
                    self.logger.info(F"current_frame = {self.current_frame.data}")
                self.pub_frame.publish(self.current_frame)

            if self.current_map_filename is not None:
                if self.verbose:
                    self.logger.info(F"current_map_filename = {self.current_map_filename.data}")
                self.pub_map.publish(self.current_map_filename)

            if self.verbose:
                self.logger.info("try to publish")
'''


class CartographerParameterConverter:
    # yaml parameter structure
    #
    # cartographer:
    #   key: value
    #   init:
    #     key: value
    #   track:
    #     key: value
    # map_list:
    #   - node_id: node_id
    #     ...
    #     cartographer:
    #       key: value
    #       init:
    #         key: value
    #       track:
    #         key: value

    def __init__(self, all_params):
        modes = [LocalizationMode.INIT, LocalizationMode.TRACK]
        modes_str = [str(mode) for mode in modes]

        map_list = all_params.get("map_list")

        cartographer_dict_global = all_params.get("cartographer", None)

        parameter_dict = {}

        for map_dict in map_list:
            node_id = map_dict["node_id"]
            parameter_dict[node_id] = {}
            cartographer_dict_map = map_dict.get("cartographer", None)

            for mode in modes_str:
                cartographer_parameters = {}
                if cartographer_dict_global is not None:
                    for key in cartographer_dict_global.keys():
                        if key not in modes_str:
                            cartographer_parameters[key] = cartographer_dict_global[key]

                    if mode in cartographer_dict_global.keys():
                        cartographer_dict_global_mode = cartographer_dict_global[mode]
                        for key in cartographer_dict_global_mode.keys():
                            if key not in modes_str:
                                cartographer_parameters[key] = cartographer_dict_global_mode[key]

                if cartographer_dict_map is not None:
                    for key in cartographer_dict_map.keys():
                        if key not in modes_str:
                            cartographer_parameters[key] = cartographer_dict_map[key]

                    if mode in cartographer_dict_map.keys():
                        cartographer_dict_map_mode = cartographer_dict_map[mode]
                        for key in cartographer_dict_map_mode.keys():
                            if key not in modes_str:
                                cartographer_parameters[key] = cartographer_dict_map_mode[key]

                parameter_dict[node_id][mode] = cartographer_parameters

        self.parameter_dict = parameter_dict

    def get_parameters(self, node_id, mode):
        return self.parameter_dict.get(node_id).get(str(mode))


def extend_node_parameter_dictionary(all_params: dict) -> dict:
    """If specific parameters are undefined, calculate and add them to the parameter dictionary."""
    all_params_new = all_params.copy()

    map_list = all_params_new.get("map_list")
    floor_count = {}  # count floor for assigning area
    for map_dict in map_list:
        # read floor
        floor = float(map_dict["floor"])
        floor_str = str(int(map_dict["floor"]))
        floor_count.setdefault(floor, 0)

        # automatically assign area if undefined
        area = int(map_dict["area"]) if "area" in map_dict else None
        if area is None:
            area = floor_count[floor]
            map_dict["area"] = area
        floor_count[floor] += 1
        area_str = str(area)

        # automatically assign node_id if undefined
        node_id = map_dict["node_id"] if "node_id" in map_dict else None
        if node_id is None:
            node_id = "carto_" + floor_str + "_" + area_str
            map_dict["node_id"] = node_id

        # automatically assign floor_id if undefined
        frame_id = map_dict["frame_id"] if "frame_id" in map_dict else None
        if frame_id is None:
            frame_id = "map_" + node_id
            map_dict["frame_id"] = frame_id

    all_params_new["map_list"] = map_list
    return all_params_new


# import yappi
# yappi.start()
def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    print("shutting down launch service")
    # yappi.stop()
    # yappi.get_func_stats().save('mf-callgrind.out', type='callgrind')
    loop.create_task(launch_service.shutdown())
    thread.join()
    # debug
    # for t in threading.enumerate():
    #     print(t)
    print(F"exit 0 {threading.get_ident()}")
    sys.exit(0)


class BufferProxy():
    def __init__(self, node):
        self._clock = node.get_clock()
        self._logger = node.get_logger()
        self.lookup_transform_service = node.create_client(LookupTransform, 'lookup_transform', callback_group=MutuallyExclusiveCallbackGroup())
        self.clear_transform_buffer_service = node.create_client(Trigger, 'clear_transform_buffer', callback_group=MutuallyExclusiveCallbackGroup())
        self.countPub = node.create_publisher(std_msgs.msg.Int32, "transform_count", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.transformMap = {}
        self.min_interval = rclpy.duration.Duration(seconds=0.2)
        self.lookup_transform_service_call_timeout_sec = 1.0
        self.lookup_transform_service_wait_timeout_sec = 5.0

    def call(self, service, request, timeout_sec: Optional[float] = None):
        # service call with timeout
        event = threading.Event()

        def unblock(future) -> None:
            nonlocal event
            event.set()

        future = service.call_async(request)
        future.add_done_callback(unblock)

        if not future.done():
            if not event.wait(timeout_sec):
                service.remove_pending_request(future)
                raise TimeoutError("service call timeout")

        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    def lookup_transform_service_call(self, request, timeout_sec: Optional[float] = None):
        return self.call(self.lookup_transform_service, request, timeout_sec)

    def clear(self):
        # clear local transform cache
        self.transformMap = {}

        # clear TF buffer in lookup transform node
        service_available = self.clear_transform_buffer_service.wait_for_service(timeout_sec=self.lookup_transform_service_wait_timeout_sec)
        if not service_available:
            self._logger.error("clear_transform_buffer_service timeout error")
            return
        req = Trigger.Request()
        self.clear_transform_buffer_service.call(req)

    def debug(self):
        if not hasattr(self, "count"):
            self.count = 0
        self.count += 1
        msg = std_msgs.msg.Int32()
        msg.data = self.count
        self.countPub.publish(msg)

    # buffer interface
    def lookup_transform(self, target, source, time=None, no_cache=False):
        # find the latest saved transform first
        key = f"{target}-{source}"
        now = self._clock.now()
        if not no_cache:
            if key in self.transformMap:
                (transform, last_time) = self.transformMap[key]
                if now - last_time < self.min_interval:
                    self._logger.info(f"found old lookup_transform({target}, {source}, {(now - last_time).nanoseconds/1000000000:.2f}sec)")
                    return transform

        if __debug__:
            self.debug()
        self._logger.info(f"lookup_transform({target}, {source})")
        req = LookupTransform.Request()
        req.target_frame = target
        req.source_frame = source
        lookup_transform_service_available = self.lookup_transform_service.wait_for_service(timeout_sec=self.lookup_transform_service_wait_timeout_sec)
        if not lookup_transform_service_available:
            raise RuntimeError("lookup_transform service unavailable (wait_for_service timeout).")
        try:
            result = self.lookup_transform_service_call(req, timeout_sec=self.lookup_transform_service_call_timeout_sec)
        except TimeoutError as e:
            raise RuntimeError(f"lookup_transform service call timeout. error=TimeoutError({e}).")
        if result.error.error > 0:
            raise RuntimeError(result.error.error_string)

        # cache valid transforms
        self.transformMap[key] = (result.transform, now)

        return result.transform

    def get_latest_common_time(self, target, source):
        transform = self.lookup_transform(target, source)
        self._logger.info(f"get_latest_common_time: transform={transform}")

        return rclpy.time.Time(seconds=transform.header.stamp.sec,
                               nanoseconds=transform.header.stamp.nanosec,
                               clock_type=self._clock.clock_type)

    def transform(self, pose_stamped, target, timeout=None):
        do_transform = tf2_ros.TransformRegistration().get(type(pose_stamped))
        transform = self.lookup_transform(target, pose_stamped.header.frame_id)
        return do_transform(pose_stamped, transform)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('multi_floor_manager')
    logger = node.get_logger()
    clock = node.get_clock()
    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()
    loop = osrf_pycommon.process_utils.get_loop()
    launch_service = LaunchService()

    # daisukes: all_params is replaced with map_config. ROS2 cannot load arbitrary dictionary as parameters
    map_list = []
    map_config_file = node.declare_parameter("map_config_file", "").value
    with open(map_config_file) as map_config:
        map_config = yaml.safe_load(map_config)

    # site map tags
    launch_tags = node.declare_parameter("tags", "").value
    default_tags = map_config.get("tags", "all")
    if launch_tags == "":
        launch_tags = default_tags
    else:
        map_config["tags"] = launch_tags
    # parse version tags
    tags = TagUtil.parse_version_tags_input(launch_tags)
    logger.info(f"launch tags = {tags}")

    sub_topics = node.declare_parameter("topic_list", ['beacons', 'wireless/beacons', 'wireless/wifi']).value

    static_broadcaster = tf2_ros.StaticTransformBroadcaster(node)
    broadcaster = tf2_ros.TransformBroadcaster(node)
    tfBuffer = BufferProxy(node)

    # multi floor manager
    multi_floor_manager_config = map_config.get("multi_floor_manager", None)
    if multi_floor_manager_config is None:
        multi_floor_manager = MultiFloorManager(node)
    else:
        multi_floor_manager_parameters = MultiFloorManagerParameters(**multi_floor_manager_config)
        logger.info(f"multi_floor_manager_config={multi_floor_manager_config}")
        multi_floor_manager = MultiFloorManager(node, multi_floor_manager_parameters)

    # load node parameters
    if not node.has_parameter("use_sim_time"):
        node.declare_parameter("use_sim_time", False)
    use_sim_time = node.get_parameter("use_sim_time").get_parameter_value().bool_value

    configuration_directory_raw = node.declare_parameter("configuration_directory", '').value
    configuration_file_prefix = node.declare_parameter("configuration_file_prefix", '').value
    temporary_directory_name = node.declare_parameter("temporary_directory_name", "tmp").value

    multi_floor_manager.local_map_frame = node.declare_parameter("local_map_frame", "map").value
    multi_floor_manager.global_map_frame = node.declare_parameter("global_map_frame", "map").value
    multi_floor_manager.odom_frame = node.declare_parameter("odom_frame", "odom").value
    multi_floor_manager.published_frame = node.declare_parameter("published_frame", "base_link").value
    multi_floor_manager.global_position_frame = node.declare_parameter("global_position_frame", "base_link").value
    meters_per_floor = node.declare_parameter("meters_per_floor", 5).value
    multi_floor_manager.floor_queue_size = node.declare_parameter("floor_queue_size", 3).value  # [seconds]

    multi_floor_manager.initial_pose_variance = node.declare_parameter("initial_pose_variance", [3.0, 3.0, 0.1, 0.0, 0.0, 100.0]).value
    n_neighbors_floor = node.declare_parameter("n_neighbors_floor", 3).value
    n_neighbors_local = node.declare_parameter("n_neighbors_local", 3).value
    n_strongest_floor = node.declare_parameter("n_strongest_floor", 10).value
    n_strongest_local = node.declare_parameter("n_strongest_local", 10).value
    min_beacons_floor = node.declare_parameter("min_beacons_floor", 3).value
    min_beacons_local = node.declare_parameter("min_beacons_local", 3).value
    floor_localizer_type = node.declare_parameter("floor_localizer", "SimpleRSSLocalizer").value
    local_localizer_type = node.declare_parameter("local_localizer", "SimpleRSSLocalizer").value

    # timeout
    multi_floor_manager.initial_localization_timeout = node.declare_parameter("initial_localization_timeout", 300).value  # [seconds]
    multi_floor_manager.initial_localization_timeout_auto_relocalization = node.declare_parameter("initial_localization_timeout_auto_relocalization", True).value
    # gnss timeout
    multi_floor_manager.gnss_init_timeout = node.declare_parameter("gnss_initial_pose_optimization_timeout", 60).value  # [seconds]
    # update by map_config
    multi_floor_manager.initial_localization_timeout = map_config.get("initial_localization_timeout",
                                                                      multi_floor_manager.initial_localization_timeout)
    multi_floor_manager.initial_localization_timeout_auto_relocalization = map_config.get("initial_localization_timeout_auto_relocalization",
                                                                                          multi_floor_manager.initial_localization_timeout_auto_relocalization)
    multi_floor_manager.gnss_init_timeout = map_config.get("gnss_initial_pose_optimization_timeout",
                                                           multi_floor_manager.gnss_init_timeout)

    # update localizer parameters by map_config
    floor_localizer_type = map_config.get("floor_localizer", floor_localizer_type)
    local_localizer_type = map_config.get("local_localizer", local_localizer_type)

    # load use_ble and use_wifi
    multi_floor_manager.use_ble = node.declare_parameter("use_ble", True).value
    multi_floor_manager.use_wifi = node.declare_parameter("use_wifi", True).value

    # update use_ble and use_wifi by map_config
    multi_floor_manager.use_ble = map_config.get("use_ble", multi_floor_manager.use_ble)
    multi_floor_manager.use_wifi = map_config.get("use_wifi", multi_floor_manager.use_wifi)
    # update ble and wifi config by map_config
    ble_config = map_config.get("ble", {})
    wifi_config = map_config.get("wifi", {})
    multi_floor_manager.ble_localization_parameters = RSSLocalizationParameters(**ble_config)
    multi_floor_manager.wifi_localization_parameters = RSSLocalizationParameters(**wifi_config)
    logger.info(f"ble config = {multi_floor_manager.ble_localization_parameters}")
    logger.info(f"wifi config = {multi_floor_manager.wifi_localization_parameters}")

    # external localizer parameters
    use_gnss = node.declare_parameter("use_gnss", False).value
    use_global_localizer = node.declare_parameter("use_global_localizer", False).value

    # auto-relocalization parameters
    multi_floor_manager.auto_relocalization = node.declare_parameter("auto_relocalization", False).value
    multi_floor_manager.rmse_threshold = node.declare_parameter("rmse_threshold", 5.0).value
    multi_floor_manager.loc_queue_min_size = node.declare_parameter("location_queue_min_size", 5).value
    multi_floor_manager.loc_queue_max_size = node.declare_parameter("location_queue_max_size", 10).value

    # pressure topic parameters
    multi_floor_manager.pressure_available = node.declare_parameter("pressure_available", True).value
    altitude_floor_estimator_config = map_config["altitude_floor_estimator"] if "altitude_floor_estimator" in map_config else None
    floor_height_mapper_config = map_config.get("floor_height_mapper", None)

    verbose = node.declare_parameter("verbose", True).value
    multi_floor_manager.verbose = verbose

    # global position parameters
    global_position_interval = node.declare_parameter("global_position_interval", 1.0).value  # default 1 [s] -> 1 [Hz]
    multi_floor_manager.global_position_averaging_interval = node.declare_parameter("averaging_interval", 1.0).value  # default 1 [s]

    # gnss parameters
    gnss_config = map_config["gnss"] if "gnss" in map_config else None

    # cartographer QoS overrides parameters
    cartographer_qos_overrides_default = {"imu": {"subscription": {"depth": 100}}}  # imu QoS depth is increased to reduce the effect of dropping imu messages
    cartographer_qos_overrides = map_config["cartographer_qos_overrides"] if "cartographer_qos_overrides" in map_config else cartographer_qos_overrides_default

    # current_publisher = CurrentPublisher(node, verbose=verbose)

    # ublox converter
    ublox_converter_config = {
                                "min_cno": node.declare_parameter("ublox_converter.min_cno", 30).value,
                                "min_elev": node.declare_parameter("ublox_converter.min_elev", 15).value,
                                "num_sv_threshold_low": node.declare_parameter("ublox_converter.num_sv_threshold_low", 5).value,
                                "num_sv_threshold_high": node.declare_parameter("ublox_converter.num_sv_threshold_high", 10).value,
                            }
    map_ublox_converter_config = map_config.get("ublox_converter", {})
    ublox_converter_config.update(map_ublox_converter_config)  # update if defined in map_config
    ublox_converter = UbloxConverterNode.from_dict(node, ublox_converter_config)
    logger.info(F"ublox_converter = {ublox_converter}")

    # configuration file check
    configuration_directory = configuration_directory_raw  # resource_utils.get_filename(configuration_directory_raw)
    temporary_directory = os.path.join(configuration_directory, temporary_directory_name)
    if not os.path.exists(temporary_directory):
        logger.error(F"temporary_directory [{temporary_directory}] does not exist.")
        raise RuntimeError(F"temporary_directory [{temporary_directory}] does not exist.")

    # resolve topic remapping
    imu_topic_name = node.resolve_topic_name("imu")
    scan_topic_name = node.resolve_topic_name("scan")
    points2_topic_name = node.resolve_topic_name("points2")
    beacons_topic_name = node.resolve_topic_name("beacons")
    initialpose_topic_name = node.resolve_topic_name("initialpose")
    odom_topic_name = node.resolve_topic_name("odom")
    fix_topic_name = node.resolve_topic_name("gnss_fix")
    fix_velocity_topic_name = node.resolve_topic_name("gnss_fix_velocity")

    # update QoS overrides parameters with resolved topic names
    cartographer_qos_overrides_resolved = {}
    for topic_name in cartographer_qos_overrides.keys():
        if topic_name not in ["imu", "scan", "points2", "odom"]:
            raise RuntimeError(F"topic name ({topic_name}) in cartographer_qos_overrides parameter is not correctly specified.")
        resolved_topic_name = node.resolve_topic_name(topic_name)
        cartographer_qos_overrides_resolved[resolved_topic_name] = cartographer_qos_overrides[topic_name]
    logger.info(F"cartographer_qos_overrides_resolved={cartographer_qos_overrides_resolved}")

    # rss offset parameter
    rssi_offset = 0.0
    robot = node.declare_parameter("robot", "").value

    # set from the dictionary inf map_config
    if "rssi_offset_list" in map_config:
        rssi_offset_list = map_config["rssi_offset_list"]
        if robot in rssi_offset_list.keys():
            rssi_offset = rssi_offset_list[robot]

    # overwrite rssi_offset if exists
    if node.has_parameter("rssi_offset"):
        rssi_offset = node.declare_parameter("rssi_offset").value
    logger.info("rssi_offset="+str(rssi_offset))

    # extend parameter dictionary
    map_config = extend_node_parameter_dictionary(map_config)

    # load the main anchor point
    anchor_dict = map_config["anchor"]
    map_list = map_config["map_list"]

    modes = [LocalizationMode.INIT, LocalizationMode.TRACK]

    global_anchor = geoutil.Anchor(lat=anchor_dict["latitude"],
                                   lng=anchor_dict["longitude"],
                                   rotate=anchor_dict["rotate"]
                                   )
    multi_floor_manager.global_anchor = global_anchor

    samples_global_all = []
    samples_ble_global_all = []
    samples_wifi_global_all = []
    floor_set = set()

    # load cartographer parameters
    cartographer_parameter_converter = CartographerParameterConverter(map_config)

    # override parameters from map_config
    for key in map_config:
        if isinstance(map_config[key], dict):
            continue
        else:
            if node.has_parameter(key):
                try:
                    node.set_parameters([Parameter(key, value=map_config[key])])
                    logger.info(F"set parameter {key}:{map_config[key]} from map config file")
                except:  # noqa E722
                    logger.error(F"cannot set parameter {key}:{map_config[key]} from map config file")
            else:
                logger.error(F"No such parameter named {key}")

    for map_dict in map_list:
        floor = float(map_dict["floor"])
        floor_str = str(int(map_dict["floor"]))
        area = int(map_dict["area"]) if "area" in map_dict else 0
        area_str = str(area)
        height = map_dict.get("height", np.nan)
        effective_radius = map_dict.get("effective_radius", np.nan)
        node_id = map_dict["node_id"]
        frame_id = map_dict["frame_id"]

        # parse specifier tags and compare with version tags
        map_tags_input = map_dict.get("tags", "all")
        map_tags = TagUtil.parse_specifier_tags_input(map_tags_input)
        skip = not TagUtil.versions_in_specifiers(tags, map_tags)
        map_dict["skip"] = skip

        if skip:
            logger.info(f"Skip loading map (node_id={node_id}) because tags ({tags}) and map_tags ({map_tags}) do not match.")
            continue

        anchor = geoutil.Anchor(lat=map_dict["latitude"],
                                lng=map_dict["longitude"],
                                rotate=map_dict["rotate"]
                                )
        load_state_filename = resource_utils.get_filename(map_dict["load_state_filename"])
        samples_filename = resource_utils.get_filename(map_dict["samples_filename"])
        # rssi gain config
        rssi_gain = map_dict.get("rssi_gain", 0.0)
        rssi_gain_ble = map_dict.get("ble", {}).get("rssi_gain", rssi_gain)
        rssi_gain_wifi = map_dict.get("wifi", {}).get("rssi_gain", rssi_gain)
        # keep the original string without resource resolving. if not found in map_dict, use "".
        map_filename = map_dict["map_filename"] if "map_filename" in map_dict else ""
        environment = map_dict["environment"] if "environment" in map_dict else "indoor"
        use_gnss_adjust = map_dict["use_gnss_adjust"] if "use_gnss_adjust" in map_dict else False
        min_hist_count = map_dict["min_hist_count"] if "min_hist_count" in map_dict else 1

        # check value
        if environment not in ["indoor", "outdoor"]:
            raise RuntimeError(
                "unknown environment ("+environment+") is in the site configuration file")

        floor_set.add(floor)

        with open(samples_filename, "rb") as f:
            samples = orjson.loads(f.read())

        # append additional information to the samples
        for s in samples:
            s["information"]["area"] = area
            s["information"]["height"] = height
            s["information"]["effective_radius"] = effective_radius

        # extract iBeacon, WiFi, and other samples
        samples_ble, samples_wifi, samples_other = extract_samples_ble_wifi_other(samples)
        # assign rssi gain
        for s in samples_ble:
            s["information"]["rssi_gain"] = rssi_gain_ble
        for s in samples_wifi:
            s["information"]["rssi_gain"] = rssi_gain_wifi

        # BLE beacon localizer
        ble_localizer_floor = None
        if multi_floor_manager.use_ble:
            # fit localizer for the floor
            ble_localizer_floor = create_wireless_rss_localizer(local_localizer_type, logger, n_neighbors=n_neighbors_local, min_beacons=min_beacons_local, rssi_offset=rssi_offset, n_strongest=n_strongest_local)
            ble_localizer_floor.fit(samples_ble)
        else:
            samples_ble = []

        # WiFi localizer
        wifi_localizer_floor = None
        if multi_floor_manager.use_wifi:
            # fit wifi localizer for the floor
            wifi_localizer_floor = create_wireless_rss_localizer(local_localizer_type, logger, n_neighbors=n_neighbors_local, min_beacons=min_beacons_local, n_strongest=n_strongest_local)
            wifi_localizer_floor.fit(samples_wifi)
        else:
            samples_wifi = []

        if floor not in multi_floor_manager.ble_localizer_dict:
            multi_floor_manager.ble_localizer_dict[floor] = {}

        multi_floor_manager.ble_localizer_dict[floor][area] = {}

        # gnss adjust
        map_frame_adjust = frame_id + "_adjust"
        if floor not in multi_floor_manager.gnss_adjuster_dict:
            multi_floor_manager.gnss_adjuster_dict[floor] = {}
        multi_floor_manager.gnss_adjuster_dict[floor][area] = TFAdjuster(frame_id, map_frame_adjust, use_gnss_adjust)

        # run ros nodes
        for mode in modes:
            namespace = node_id+"/"+str(mode)
            sub_mode = "tracking" if mode == LocalizationMode.TRACK else "rss_localization"
            if environment != "indoor":
                sub_mode = sub_mode + "_" + environment

            included_configuration_basename = configuration_file_prefix + "_" + sub_mode + ".lua"
            tmp_configuration_basename = temporary_directory_name + "/" + configuration_file_prefix + "_" + sub_mode + "_" + floor_str + "_" + area_str + ".lua"

            # update config variables if needed
            options_map_frame = frame_id
            if use_gnss:
                options_map_frame = map_frame_adjust

            # load cartographer_parameters
            cartographer_parameters = cartographer_parameter_converter.get_parameters(node_id, mode)

            # create  temporary config files
            with open(os.path.join(configuration_directory, tmp_configuration_basename), "w") as f:
                f.write("include \""+included_configuration_basename+"\""+"\n")
                f.write("options.map_frame = \""+options_map_frame+"\""+"\n")
                f.write("options.odom_frame = \""+multi_floor_manager.odom_frame+"\""+"\n")
                f.write("options.published_frame = \""+multi_floor_manager.published_frame+"\""+"\n")

                # overwrite cartographer parameters if exist
                for cartographer_param_key in cartographer_parameters.keys():
                    if type(cartographer_parameters[cartographer_param_key]) is bool:
                        f.write(cartographer_param_key + " = " + str(cartographer_parameters[cartographer_param_key]).lower() + "\n")
                    else:
                        f.write(cartographer_param_key + " = " + str(cartographer_parameters[cartographer_param_key]) + "\n")

                # end of the config file
                f.write("return options")

            package1 = "cartographer_ros"
            executable1 = "cartographer_node"

            # run cartographer node
            launch_service.include_launch_description(LaunchDescription([
                LogInfo(msg=F"Launching cartographer node {namespace}"),
                Node(
                    package=package1,
                    executable=executable1,
                    name="cartographer_node",
                    namespace=namespace,
                    remappings=[("scan", scan_topic_name),
                                ("points2", points2_topic_name),
                                ("imu", imu_topic_name),
                                ("odom", odom_topic_name),
                                ("fix", "/"+namespace+fix_topic_name)],
                    output="both",
                    arguments=[
                        "-configuration_directory", configuration_directory,
                        "-configuration_basename", tmp_configuration_basename,
                        "-load_state_filename", load_state_filename,
                        "-start_trajectory_with_default_topics=false",
                        "--collect_metrics"
                    ],
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'qos_overrides': cartographer_qos_overrides_resolved
                    }]
                )
            ]))

            # create floor_manager
            floor_manager = FloorManager()
            floor_manager.configuration_directory = configuration_directory
            floor_manager.configuration_basename = tmp_configuration_basename
            multi_floor_manager.ble_localizer_dict[floor][area][mode] = floor_manager

        # set values to floor_manager
        for mode in modes:
            floor_manager = multi_floor_manager.ble_localizer_dict[floor][area][mode]

            floor_manager.localizer = ble_localizer_floor
            floor_manager.wifi_localizer = wifi_localizer_floor
            floor_manager.node_id = node_id
            floor_manager.frame_id = frame_id
            floor_manager.map_filename = map_filename
            floor_manager.min_hist_count = min_hist_count
            # publishers
            floor_manager.initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, node_id+"/"+str(mode)+initialpose_topic_name, 10, callback_group=MutuallyExclusiveCallbackGroup())
            floor_manager.fix_pub = node.create_publisher(NavSatFix, node_id+"/"+str(mode)+fix_topic_name, 10, callback_group=MutuallyExclusiveCallbackGroup())

            multi_floor_manager.ble_localizer_dict[floor][area][mode] = floor_manager

        # convert samples to the coordinate of global_anchor
        samples_ble_global = convert_samples_coordinate(samples_ble, anchor, global_anchor, floor)
        samples_wifi_global = convert_samples_coordinate(samples_wifi, anchor, global_anchor, floor)
        samples_other_global = convert_samples_coordinate(samples_other, anchor, global_anchor, floor)

        samples_ble_global_all.extend(samples_ble_global)
        samples_wifi_global_all.extend(samples_wifi_global)
        samples_global_all.extend(samples_ble_global)
        samples_global_all.extend(samples_wifi_global_all)
        samples_global_all.extend(samples_other_global)

        # calculate static transform
        xy = geoutil.global2local(anchor, global_anchor)
        yaw = - math.radians(anchor.rotate - global_anchor.rotate)

        t = TransformStamped()
        t.header.stamp = clock.now().to_msg()
        t.header.frame_id = multi_floor_manager.global_map_frame
        t.child_frame_id = frame_id

        z = floor * meters_per_floor  # for visualization
        trans = Vector3(x=xy.x, y=xy.y, z=z)
        t.transform.translation = trans

        q = tf_transformations.quaternion_from_euler(0, 0, yaw, 'sxyz')
        rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        t.transform.rotation = rotation

        multi_floor_manager.transforms.append(t)

    # execute launch task
    launch_task = loop.create_task(launch_service.run_async())

    # wait for services after launching all nodes to reduce waiting time
    for map_dict in map_list:
        floor = float(map_dict["floor"])
        area = int(map_dict["area"]) if "area" in map_dict else 0
        node_id = map_dict["node_id"]

        if map_dict["skip"]:
            continue

        for mode in modes:
            floor_manager = multi_floor_manager.ble_localizer_dict[floor][area][mode]
            # rospy service
            # define callback group accessing the same ros node
            node_service_callback_group = MutuallyExclusiveCallbackGroup()
            floor_manager.get_trajectory_states = node.create_client(GetTrajectoryStates, node_id+"/"+str(mode)+'/get_trajectory_states', callback_group=node_service_callback_group)
            floor_manager.finish_trajectory = node.create_client(FinishTrajectory, node_id+"/"+str(mode)+'/finish_trajectory', callback_group=node_service_callback_group)
            floor_manager.start_trajectory = node.create_client(StartTrajectory, node_id+"/"+str(mode)+'/start_trajectory', callback_group=node_service_callback_group)
            floor_manager.read_metrics = node.create_client(ReadMetrics, node_id+"/"+str(mode)+'/read_metrics', callback_group=node_service_callback_group)
            floor_manager.trajectory_query = node.create_client(TrajectoryQuery, node_id+"/"+str(mode)+'/trajectory_query', callback_group=node_service_callback_group)

    multi_floor_manager.floor_list = list(floor_set)

    # a localizer to estimate floor
    # ble floor localizer
    if multi_floor_manager.use_ble:
        multi_floor_manager.ble_floor_localizer = create_wireless_rss_localizer(floor_localizer_type, logger, n_neighbors=n_neighbors_floor, min_beacons=min_beacons_floor, rssi_offset=rssi_offset, n_strongest=n_strongest_floor)
        multi_floor_manager.ble_floor_localizer.fit(samples_ble_global_all)
    # wifi floor localizer
    if multi_floor_manager.use_wifi:
        multi_floor_manager.wifi_floor_localizer = create_wireless_rss_localizer(floor_localizer_type, logger, n_neighbors=n_neighbors_floor, min_beacons=min_beacons_floor, n_strongest=n_strongest_floor)
        multi_floor_manager.wifi_floor_localizer.fit(samples_wifi_global_all)

    # altitude manager and altitude floor estimator
    multi_floor_manager.altitude_manager = AltitudeManager(node)
    if altitude_floor_estimator_config is None:
        altitude_floor_estimator_parameters = AltitudeFloorEstimatorParameters(enable=False)
    else:
        altitude_floor_estimator_parameters = AltitudeFloorEstimatorParameters(**altitude_floor_estimator_config)
    multi_floor_manager.altitude_floor_estimator = AltitudeFloorEstimator(altitude_floor_estimator_parameters)

    # floor height mapper for floor estimation
    X_height_mapper = []
    for s in samples_global_all:
        x_a = float(s["information"]["x"])
        y_a = float(s["information"]["y"])
        f_a = float(s["information"]["floor"])
        area = int(s["information"]["area"])
        height = s["information"]["height"]
        effective_radius = s["information"]["effective_radius"]
        X_height_mapper.append([x_a, y_a, f_a, area, height, effective_radius])
    if floor_height_mapper_config is None:
        multi_floor_manager.floor_height_mapper = FloorHeightMapper(X_height_mapper, logger=logger,)
    else:
        logger.info(f"floor_height_mapper_config={floor_height_mapper_config}")
        multi_floor_manager.floor_height_mapper = FloorHeightMapper(X_height_mapper, logger=logger, **floor_height_mapper_config)

    # area localizer
    X_area = []
    Y_area = []
    for s in samples_global_all:
        x_a = float(s["information"]["x"])
        y_a = float(s["information"]["y"])
        f_a = float(s["information"]["floor"])
        f_a = f_a * multi_floor_manager.area_floor_const
        X_area.append([x_a, y_a, f_a])
        area = int(s["information"]["area"])
        Y_area.append(area)
    from sklearn.neighbors import KNeighborsClassifier
    area_classifier = KNeighborsClassifier(n_neighbors=1)
    area_classifier.fit(X_area, Y_area)
    multi_floor_manager.X_area = np.array(X_area)
    multi_floor_manager.Y_area = np.array(Y_area)
    multi_floor_manager.area_localizer = area_classifier

    # define callback group accessing the state variables
    state_update_callback_group = MutuallyExclusiveCallbackGroup()

    # global subscribers
    sensor_qos = qos_profile_sensor_data
    beacons_sub = node.create_subscription(String, "beacons", multi_floor_manager.beacons_callback, 1, callback_group=state_update_callback_group)
    wifi_sub = node.create_subscription(String, "wifi", multi_floor_manager.wifi_callback, 1, callback_group=state_update_callback_group)
    initialpose_sub = node.create_subscription(PoseWithCovarianceStamped, "initialpose", multi_floor_manager.initialpose_callback, 10, callback_group=state_update_callback_group)
    pressure_sub = node.create_subscription(FluidPressure, "pressure", multi_floor_manager.pressure_callback, 10, callback_group=state_update_callback_group)

    # services
    stop_localization_service = node.create_service(StopLocalization, "stop_localization", multi_floor_manager.stop_localization_callback, callback_group=state_update_callback_group)
    start_localization_service = node.create_service(StartLocalization, "start_localization", multi_floor_manager.start_localization_callback, callback_group=state_update_callback_group)
    restart_localization_service = node.create_service(RestartLocalization, "restart_localization", multi_floor_manager.restart_localization_callback, callback_group=state_update_callback_group)
    enable_relocalization_service = node.create_service(MFTrigger, "enable_auto_relocalization", multi_floor_manager.enable_relocalization_callback, callback_group=MutuallyExclusiveCallbackGroup())
    disable_relocalization_service = node.create_service(MFTrigger, "disable_auto_relocalization", multi_floor_manager.disable_relocalization_callback, callback_group=MutuallyExclusiveCallbackGroup())
    set_current_floor_service = node.create_service(MFSetInt, "set_current_floor", multi_floor_manager.set_current_floor_callback, callback_group=state_update_callback_group)
    convert_local_to_global_service = node.create_service(ConvertLocalToGlobal, "convert_local_to_global", multi_floor_manager.convert_local_to_global_callback, callback_group=state_update_callback_group)

    seng_cfg_rst_client = node.create_client(SendCfgRST, "/ublox/send_cfg_rst", callback_group=MutuallyExclusiveCallbackGroup())
    reset_gnss_service = node.create_service(MFTrigger, "reset_gnss", multi_floor_manager.reset_gnss_callback, callback_group=MutuallyExclusiveCallbackGroup())

    # external localizer
    if gnss_config is not None:
        multi_floor_manager.gnss_params = GNSSParameters(**gnss_config)
        logger.info(F"gnss_config={gnss_config}, multi_floor_manager.gnss_params={multi_floor_manager.gnss_params}")
    if use_gnss:
        multi_floor_manager.gnss_is_active = True
        multi_floor_manager.indoor_outdoor_mode = IndoorOutdoorMode.UNKNOWN
        gnss_fix_sub = message_filters.Subscriber(node, NavSatFix, "gnss_fix", callback_group=state_update_callback_group)
        gnss_fix_velocity_sub = message_filters.Subscriber(node, TwistWithCovarianceStamped, "gnss_fix_velocity", callback_group=state_update_callback_group)
        time_synchronizer = message_filters.TimeSynchronizer([gnss_fix_sub, gnss_fix_velocity_sub], 10)
        time_synchronizer.registerCallback(multi_floor_manager.gnss_fix_callback)
        navsat_sub = node.create_subscription(NavSAT, "navsat", multi_floor_manager.navsat_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        mf_navsat_sub = node.create_subscription(MFNavSAT, "mf_navsat", multi_floor_manager.mf_navsat_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        multi_floor_manager.gnss_fix_local_pub = node.create_publisher(PoseWithCovarianceStamped, "gnss_fix_local", 10, callback_group=MutuallyExclusiveCallbackGroup())

        # map_frame_adjust_time = node.create_timer(0.1, multi_floor_manager.map_frame_adjust_callback) # 10 Hz

    if use_global_localizer:
        multi_floor_manager.is_active = False  # deactivate multi_floor_manager
        global_localizer_global_pose_sub = node.create_subscription(MFGlobalPosition, "/global_localizer/global_pose", multi_floor_manager.global_localizer_global_pose_callback, 10, callback_group=state_update_callback_group)
        # call external global localization service
        logger.info("wait for service /global_localizer/request_localization")
        global_localizer_request_localization = node.create_client(MFSetInt, '/global_localizer/request_localization', callback_group=MutuallyExclusiveCallbackGroup())
        global_localizer_request_localization.wait_for_service()
        n_compute_global_pose = 1  # request computing global_pose once
        resp = global_localizer_request_localization(n_compute_global_pose)
        logger.info("called /global_localizer/request_localization")

    # publish global_map->local_map by /tf_static
    multi_floor_manager.send_static_transforms()
    # publish dummy tf to prevent map_global -> map -> published_frame connection timeout
    multi_floor_manager.send_dummy_local_map_tf()

    # global position
    global_position_timer = node.create_timer(global_position_interval, multi_floor_manager.global_position_callback, callback_group=state_update_callback_group)

    # detect optimization
    multi_floor_manager.map2odom = None

    # ros spin
    spin_rate = 10  # 10 Hz

    # for loginfo
    log_interval = spin_rate  # loginfo at about 1 Hz

    multi_floor_manager.localize_status = MFLocalizeStatus.UNKNOWN

    def main_loop():
        # detect optimization
        if multi_floor_manager.is_optimized():
            tfBuffer.clear()  # clear outdated tf before optimization
            multi_floor_manager.optimization_detected = True
            multi_floor_manager.initial_localization_time = None  # clear timeout count

        # detect area and mode switching
        multi_floor_manager.check_and_update_states()  # 1 Hz

        # publish adjustment tf for outdoor mode
        if use_gnss:
            multi_floor_manager.publish_map_frame_adjust_tf()

    main_timer = node.create_timer(1.0 / spin_rate, main_loop, callback_group=state_update_callback_group)

    def run():
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor)
        print(F"exit 0 (run thread) {threading.get_ident()}")
        sys.exit(0)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    loop.run_until_complete(launch_task)
