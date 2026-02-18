#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
# Copyright (c) 2024  ALPS ALPINE CO., LTD.
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
"""
Cabot UI Manager

This class manages the state of the robot.
It serves low level event and maps to high level event which may change state of the robot.

Ideally, this class has plugin architecture to add new UI component but it is not the current goal.
So, all controls which needs to see the current state of the robot are managed by this code.

Low-level (cabot_common.event) should be mapped into ui-level (cabot_ui.event)

Author: Daisuke Sato<daisuke@cmu.edu>
"""

import os
import signal
import sys
import threading
import time
import traceback
import yaml


import tf_transformations
import numpy as np


import geometry_msgs
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
import rclpy.client
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
import sensor_msgs
import std_msgs.msg
import std_srvs.srv
from nav_msgs.msg import Odometry, OccupancyGrid
from cabot_msgs.msg import PoseLog


import cabot_common.button
from cabot_common.event import BaseEvent, ButtonEvent, ClickEvent, HoldDownEvent
from cabot_ui.event import MenuEvent, NavigationEvent, ExplorationEvent
from cabot_ui.menu import Menu
from cabot_ui.status import State, StatusManager
from cabot_ui.interface import UserInterface
from cabot_ui.exploration import Exploration
from cabot_ui.navigation import Navigation, NavigationInterface
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.description import Description
from cabot_ui.explore.test_speak import speak_text


from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus
from cabot_common import vibration
from enum import Enum
import math

class ExplorationMode(Enum):
        MANUAL = 0
        SHARED = 1
        AUTONOMOUS = 2
        TOTAL_FREE = 3

class CabotUIManager(NavigationInterface, object): 
    def __init__(self, node, nav_node, tf_node, srv_node, act_node, soc_node, desc_node): #TODO : Resume from here, try to verify logs and if the code is called properly
        self._node = node
        self._logger = self._node.get_logger()
        CaBotRclpyUtil.initialize(self._node)

        CaBotRclpyUtil.info("CabotUIManager initializing")

        CabotUIManager.instance = self
        self.in_navigation = False
        self.destination = None

        self.reset()

        self.free_mode_detect_lidar_obstacles = self._node.declare_parameter('free_mode_detect_lidar_obstacles', False).value
        self.free_mode_detect_low_obstacles = self._node.declare_parameter('free_mode_detect_low_obstacles', False).value
        self.free_mode_detect_lidar_max_limit_speed = self._node.declare_parameter('free_mode_detect_lidar_max_limit_speed', 0.05).value
        self.free_mode_detect_costmap_obstacles = self._node.declare_parameter('free_mode_detect_costmap_obstacles', True).value
        self.free_mode_detect_costmap_threshold = self._node.declare_parameter('free_mode_detect_costmap_threshold', 30).value
        self.free_mode_detect_costmap_forward_distance = self._node.declare_parameter('free_mode_detect_costmap_forward_distance', 0.5).value
        self.free_mode_warn_cost_threshold = self._node.declare_parameter('free_mode_warn_cost_threshold', 50).value
        self.free_mode_warn_forward_distance = self._node.declare_parameter('free_mode_warn_forward_distance', 0.7).value
        self.free_mode_warn_left_side_distance = self._node.declare_parameter('free_mode_warn_left_side_distance', 0.3).value
        self.free_mode_warn_left_forward_distance = self._node.declare_parameter('free_mode_warn_left_forward_distance', 0.3).value
        self.free_mode_warn_right_side_distance = self._node.declare_parameter('free_mode_warn_right_side_distance', 0.3).value
        self.free_mode_warn_right_forward_distance = self._node.declare_parameter('free_mode_warn_right_forward_distance', 0.3).value
        self.free_mode_warn_back_distance = self._node.declare_parameter('free_mode_warn_back_distance', 0.3).value
        self.free_mode_stop_duration = self._node.declare_parameter('free_mode_stop_duration', 0.5).value
        self.free_mode_correction_back_duration = self._node.declare_parameter('free_mode_correction_back_duration', 0.5).value
        self.free_mode_correction_back_enabled = self._node.declare_parameter('free_mode_correction_back_enabled', True).value
        self.free_mode_correction_back_speed = self._node.declare_parameter('free_mode_correction_back_speed', 0.2).value
        self.free_mode_correction_side_duration = self._node.declare_parameter('free_mode_correction_side_duration', 0.5).value
        self.free_mode_correction_side_enabled = self._node.declare_parameter('free_mode_correction_side_enabled', True).value
        self.free_mode_correction_side_speed = self._node.declare_parameter('free_mode_correction_side_speed', 0.2).value
        self.free_mode_correction_side_turnspeed = self._node.declare_parameter('free_mode_correction_side_turnspeed', 0.5).value
        self.free_mode_correction_touch_required = self._node.declare_parameter('free_mode_correction_touch_required', True).value
        self.free_mode_switch_autonomous_mode = self._node.declare_parameter('free_mode_switch_autonomous_mode', False).value
        self.free_mode_switch_autonomous_wizard_mode = self._node.declare_parameter('free_mode_switch_autonomous_wizard_mode', False).value
        self.free_mode_switch_autonomous_mode_temp = self._node.declare_parameter('free_mode_switch_autonomous_mode_temp', False).value
        self.free_mode_switch_autonomous_mode_temp_duration = self._node.declare_parameter('free_mode_switch_autonomous_mode_temp_duration', 5.0).value
        self.free_mode_end_userfree_movement_time = self._node.declare_parameter('free_mode_end_userfree_movement_time', 0.5).value

        self.cabot_vlm_use_button = self._node.declare_parameter('cabot_vlm_use_button', False).value

        self.cabot_allowed_modes_bitmask = self._node.declare_parameter('cabot_allowed_modes_bitmask', 15).value

        self._logger.info(f"CabotUIManager free_mode_correction_back_duration: {self.free_mode_correction_back_duration}")
        self._logger.info(f"CabotUIManager free_mode_correction_side_duration: {self.free_mode_correction_side_duration}")

        self._logger.info(f"CabotUIManager cabot_allowed_modes_bitmask: {self.cabot_allowed_modes_bitmask}")

        self._logger.info(f"free_mode_switch_autonomous_mode_temp : {self.free_mode_switch_autonomous_mode_temp}, free_mode_switch_autonomous_mode_temp_duration : {self.free_mode_switch_autonomous_mode_temp_duration}")
        self._logger.info(f"free_mode_switch_autonomous_wizard_mode : {self.free_mode_switch_autonomous_wizard_mode}")


        # process allowed modes bitmask
        allowed_modes_bytes = self.cabot_allowed_modes_bitmask & 0x0F  # only lower 4 bits are used
        if allowed_modes_bytes == 0:
            self.allowed_modes = [
                ExplorationMode.AUTONOMOUS,
                ExplorationMode.SHARED,
                ExplorationMode.MANUAL,
                ExplorationMode.TOTAL_FREE
            ]
        else:
            self.allowed_modes = []
            if allowed_modes_bytes & 0x01:
                self.allowed_modes.append(ExplorationMode.AUTONOMOUS)
                self._logger.info("CabotUIManager AUTONOMOUS mode allowed")
            if allowed_modes_bytes & 0x02:
                self.allowed_modes.append(ExplorationMode.SHARED)
                self._logger.info("CabotUIManager SHARED mode allowed")
            if allowed_modes_bytes & 0x04:
                self.allowed_modes.append(ExplorationMode.MANUAL)
                self._logger.info("CabotUIManager MANUAL mode allowed")
            if allowed_modes_bytes & 0x08:
                self.allowed_modes.append(ExplorationMode.TOTAL_FREE)
                self._logger.info("CabotUIManager TOTAL_FREE mode allowed")

        # Force allowed mode
        self.default_mode = self.allowed_modes[0]

        self._handle_button_mapping = node.declare_parameter('handle_button_mapping', 2).value
        if self._handle_button_mapping == 2:
            self._event_mapper = EventMapper2()
        else:
            self._event_mapper = EventMapper1(self)
        self._event_mapper.delegate = self
        self._status_manager = StatusManager.get_instance()
        self._status_manager.delegate = self
        self._interface = UserInterface(self._node)
        self._interface.delegate = self
        self._navigation = Navigation(nav_node, tf_node, srv_node, act_node, soc_node)
        self._navigation.delegate = self
        self._description = Description(desc_node)
        self._exploration = Exploration(self._node)
        self._exploration.delegate = self

        self._retry_count = 0

        if self.free_mode_detect_lidar_obstacles:
            self._lidarLimitSub = self._node.create_subscription(std_msgs.msg.Float32, "/cabot/lidar_speed", self._lidar_limit_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())
        if self.free_mode_detect_low_obstacles:
            self._lowLidarLimitSub = self._node.create_subscription(std_msgs.msg.Float32, "/cabot/low_lidar_speed", self._lidar_limit_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())

        self._allowButtons = True
        self._enableHandleButtons = self._node.create_subscription(std_msgs.msg.Bool, "/cabot/allow_buttons", self._allow_buttons_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())

        self._touchHandle = False

        self._touchSub = self._node.create_subscription(std_msgs.msg.Float32, "/cabot/touch_speed_switched", self._touch_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())

        if self.cabot_vlm_use_button:
            self._vlmButtonPub = self._node.create_publisher(std_msgs.msg.Bool, "/cabot/vlm_button", 10, callback_group=MutuallyExclusiveCallbackGroup())
            self._vlmButtonPub.publish(std_msgs.msg.Bool(data=False))

        #self._lidarLimitSub = self._node.create_subscription(sensor_msgs.msg.LaserScan, "/scan", self._lidar_limit_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())
        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.registering_map = False
        self.registering_odom = False
        self.map_sub = self._node.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._map_callback, transient_local_qos)
        #self.odom_sub = self._node.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        #self.odom_sub = self._node.create_subscription(PoseLog, "/cabot/pose_log", self._odom_callback, 10)
        self._odom_timer = act_node.create_timer(0.01, self._odom_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.register_map_lock = threading.RLock()
        self.register_odom_lock = threading.RLock()

        self.dx = 0.0
        self.dy = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_orientation = 0.0

        self._node.create_subscription(std_msgs.msg.String, "/cabot/event", self._event_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self._eventPub = self._node.create_publisher(std_msgs.msg.String, "/cabot/event", 10, callback_group=MutuallyExclusiveCallbackGroup())

        self._speedOverwritePub = self._node.create_publisher(std_msgs.msg.Float32, "/cabot/speed_overwrite", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self._turnSpeedOverwritePub = self._node.create_publisher(std_msgs.msg.Float32, "/cabot/turn_speed_overwrite", 10, callback_group=MutuallyExclusiveCallbackGroup())
        


        self._personaPub = self._node.create_publisher(std_msgs.msg.String, "/cabot/persona", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.persona_list = ["navigation", "middle", "explore"]
        self.persona_index = 1
        self._logger.info(f"[CHILOG] [BUTTON] [PERSONA] [{self.persona_list[self.persona_index]}]")

        CaBotRclpyUtil.info("CabotUIManager initializing 2")

        CaBotRclpyUtil.info(f"CabotUIManager LIDAR Detect Obstacles: {self.free_mode_warn_cost_threshold}")

        # request language
        e = NavigationEvent("getlanguage", None)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

        self._event_mapper.late_initialize()

        def handleside_callback(msg):
            # request handleside
            self.handleside = msg.data
            self.send_handleside()
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.handleside_sub = node.create_subscription(std_msgs.msg.String, "/cabot/features/handleside", handleside_callback, qos_profile)

        def touchmode_callback(msg):
            # request touchmode
            self.touchmode = msg.data
            self.send_touchmode()
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.touchmode_sub = node.create_subscription(std_msgs.msg.String, "/cabot/features/touchmode", touchmode_callback, qos_profile)

        self._touchModeProxy = self._node.create_client(std_srvs.srv.SetBool, "/cabot/set_touch_speed_active_mode", callback_group=MutuallyExclusiveCallbackGroup())

        self._userSpeedEnabledProxy = self._node.create_client(std_srvs.srv.SetBool, "/cabot/user_speed_enabled", callback_group=MutuallyExclusiveCallbackGroup())

        self.updater = Updater(self._node)

        def manager_status(stat):
            if self._navigation.i_am_ready:
                stat.summary(DiagnosticStatus.OK, "Ready")
            else:
                stat.summary(DiagnosticStatus.ERROR, "Not ready")
            return stat
        self.updater.add(FunctionDiagnosticTask("UI Manager", manager_status))

        self.create_menu_timer = self._node.create_timer(1.0, self.create_menu, callback_group=MutuallyExclusiveCallbackGroup())

        self.send_speaker_audio_files()


    def _allow_buttons_callback(self, msg):
        self._allowButtons = msg.data

    def _touch_callback(self, msg):
        self._touchHandle = msg.data > 0.0

    def _map_callback(self, msg):
        self._logger.info("Map callback")

        if self.cabot_vlm_use_button: # Using this as a slow update
            self._vlmButtonPub.publish(std_msgs.msg.Bool(data=False))

        with self.register_map_lock:
            if self.registering_map == True:
                return
            self.registering_map = True

        self._logger.info("Processing map callback")

        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

        # self.global_map_pub.publish(msg)

        # calculate map orientation in radian
        map_quaternion = (msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(map_quaternion)
        self.map_orientation = yaw

        if(self.map_width == 0):
            self.logger.info("[CaBotMapNode] map callback received but map width is 0, waiting for valid map...")
            return

        # get occupancy grid data
        self.map_data = np.asarray(msg.data).reshape((msg.info.height, msg.info.width))

        with self.register_map_lock:
            self.registering_map = False

    def _odom_callback(self):
        self._logger.info("Odom callback")

        with self.register_odom_lock:
            if self.registering_odom == True:
                return
            self.registering_odom = True

        self._logger.info("Processing odom callback")

        try:
            current_pose = self._navigation.current_local_pose()
        except:
            self._logger.info("pose callback not ready")
            with self.register_odom_lock:
                self.registering_odom = False
            return
        
        #current_time = self._node.get_clock().now()

        # speed = 0.0
        # if not hasattr(self, 'odom_x'):
        #     speed = 0.0
        #     self.dx = 0.0
        #     self.dy = 0.0
        # else:
        #     self.dx = 0.90*self.dx + 0.1*(current_pose.x - self.odom_x)
        #     self.dy = 0.90*self.dy + 0.1*(current_pose.y - self.odom_y)
        #     speed = math.sqrt(self.dx ** 2 + self.dy ** 2) / ((current_time - self.odom_time).nanoseconds / 1e9 + 1e-6)
        #speed = math.sqrt((current_pose.x - self.odom_x) ** 2 + (current_pose.y - self.odom_y) ** 2) / ((current_time - self.odom_time).nanoseconds / 1e9 + 1e-6)

        self.odom_x = current_pose.x
        self.odom_y = current_pose.y
        self.odom_orientation = current_pose.r
        #self.odom_time = current_time

        # self.odom_x = msg.pose.pose.position.x
        # self.odom_y = msg.pose.pose.position.y
        # self.odom_x = msg.pose.position.x
        # self.odom_y = msg.pose.position.y

        #self._logger.info(f"Robot odom coordinates: ({self.odom_x}, {self.odom_y}) with speed {speed}")

        # Calculate costmap coordinates
        if not hasattr(self, 'map_x'):
            with self.register_odom_lock:
                self.registering_odom = False
                return
            
        # Transform odom coordinates to map coordinates
        posX = self.odom_x - self.map_x
        posY = self.odom_y - self.map_y

        self._logger.info(f"Robot odom position: ({self.odom_x}, {self.odom_y})")

        # Local offset, check forward direction
        forwardDistance = self.free_mode_detect_costmap_forward_distance  # meters
        checkX = posX + forwardDistance * math.cos(self.odom_orientation)
        checkY = posY + forwardDistance * math.sin(self.odom_orientation)

        # Convert to map grid indices
        self.robot_map_x = int(checkX / self.map_resolution)
        self.robot_map_y = int((checkY / self.map_resolution))
        #self._logger.info(f"Robot map grid indices: ({self.robot_map_x}, {self.robot_map_y})")

        # Get cost at robot's position
        if 0 <= self.robot_map_x < self.map_width and 0 <= self.robot_map_y < self.map_height:
            current_cost = self.map_data[self.robot_map_y, self.robot_map_x]

            # Log current cost
            #self._logger.info(f"Current cost at robot position: {current_cost}, current speed : {speed}")

            max_allowed_cost = self.free_mode_detect_costmap_threshold
            # if speed > 0.5:
            #     max_allowed_cost = 20
            # if speed > 1.0:
            #     max_allowed_cost = 0
            # if speed > 1.5:
            #     max_allowed_cost = -1

            #self._logger.info(f"Max allowed cost: {max_allowed_cost}")

            if self.free_mode_detect_costmap_obstacles:
                if current_cost > max_allowed_cost or current_cost == -1:
                    self._event_mapper.checkLidarLimit(self._logger, 0.0, self._speedOverwritePub, self._turnSpeedOverwritePub, self)

        with self.register_odom_lock:
            self.registering_odom = False

    def _lidar_limit_callback(self, msg):
        self._logger.debug("Lidar limit callback")
        self._event_mapper.checkLidarLimit(self._logger, msg.data, self._speedOverwritePub, self._turnSpeedOverwritePub, self)

    def send_handleside(self):
        e = NavigationEvent("gethandleside", self.handleside)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

    def send_touchmode(self):
        e = NavigationEvent("gettouchmode", self.touchmode)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

    def send_speaker_audio_files(self):
        if self._interface.audio_files:
            e = NavigationEvent("getspeakeraudiofiles", self._interface.audio_files)
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)
        else:
            return

    def create_menu(self):
        try:
            self.create_menu_timer.cancel()
            menu_file = node.declare_parameter('menu_file', '').value
            with open(menu_file, 'r') as stream:
                menu_obj = yaml.safe_load(stream)
                # overwrite init_speed if it is set
                if "max_velocity_menu" in menu_obj:
                    try:
                        desc = ParameterDescriptor()
                        desc.type = ParameterType.PARAMETER_NOT_SET
                        desc.dynamic_typing = True
                        self._node.declare_parameter('init_speed', None, descriptor=desc)
                        init_speed = self._node.get_parameter("init_speed").value
                        # check if init_speed is not empty and is float num
                        if init_speed is not None and isinstance(init_speed, float):
                            self._logger.error(f"init_speed is {init_speed}")
                            menu_obj["max_velocity_menu"]["value"] = init_speed
                    except:  # noqa: #722
                        self._logger.error(traceback.format_exc())
                        pass
                self.main_menu = Menu.create_menu(menu_obj, {"menu": "main_menu"})
            self.speed_menu = None
            if self.main_menu:
                self.main_menu.delegate = self
                self.speed_menu = self.main_menu.get_menu_by_identifier("max_velocity_menu")
            else:
                self._logger.error("menu is not initialized")
            if self.speed_menu:
                max_speed = min(2.75, self._node.declare_parameter("max_speed", 1.0).value)
                self.speed_menu._max = max_speed
                init_speed = self.speed_menu.value
                try:
                    desc = ParameterDescriptor()
                    desc.type = ParameterType.PARAMETER_DOUBLE
                    self._node.declare_parameter('init_speed', None, descriptor=desc)
                    temp = self._node.get_parameter("init_speed").value
                    if temp is not None:
                        init_speed = min(temp, max_speed)
                except:  # noqa: #722
                    self._logger.error(traceback.format_exc())
                    pass
                self._logger.info(f"Initial Speed = {init_speed}")
                self.speed_menu.set_value(init_speed)
            self.menu_stack = []
            self._logger.info("create_menu completed")
        except:  # noqa: #722
            self._logger.error(traceback.format_exc())

    # region NavigationInterface
    def activity_log(self, category="", text="", memo=""):
        self._interface.activity_log(category, text, memo)

    def i_am_ready(self):
        self._status_manager.set_state(State.idle)
        self._interface.i_am_ready()

    def start_navigation(self):
        self._logger.info("self._interface.start_navigation()")
        self._interface.start_navigation()

    def update_pose(self, **kwargs):
        self._interface.update_pose(**kwargs)

    def notify_turn(self, device=None, turn=None):
        self._interface.notify_turn(device=device, turn=turn)

    def notify_human(self, angle=0):
        self._interface.notify_human(angle=angle)

    def goal_canceled(self, goal):
        # unexpected cancel, may need to retry
        if self._status_manager.state == State.in_action:
            self._logger.info("NavigationState: canceled (system)")
            self._status_manager.set_state(State.in_pausing)
            self._retry_navigation()
            return
        self._logger.info("NavigationState: canceled (user)")

    def _retry_navigation(self):
        self._retry_count += 1
        self._logger.info("NavigationState: retrying (system)")
        self._status_manager.set_state(State.in_action)
        self._navigation.retry_navigation()
        self._logger.info("NavigationState: retried (system)")

    def have_arrived(self, goal):
        # do not read arrival message from robot
        # self._logger.info("delegate have_arrived called")
        # self._interface.have_arrived(goal)
        pass

    def have_completed(self):
        self._status_manager.set_state(State.idle)
        # send navigation_arrived event when all goals are completed
        self._logger.info("NavigationState: arrived")
        # notify external nodes about arrival
        e = NavigationEvent("arrived", None)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

    def approaching_to_poi(self, poi=None):
        self._interface.approaching_to_poi(poi=poi)

    def approached_to_poi(self, poi=None):
        self._interface.approached_to_poi(poi=poi)

    def passed_poi(self, poi=None):
        self._interface.passed_poi(poi=poi)

    def could_not_get_current_location(self):
        self._interface.could_not_get_current_location()

    def please_call_elevator(self, pos):
        self._interface.please_call_elevator(pos)

    def enter_goal(self, goal):
        self._interface.enter_goal(goal)

    def exit_goal(self, goal):
        self._interface.exit_goal(goal)

    def elevator_opening(self):
        self._interface.elevator_opening()

    def floor_changed(self, floor):
        self._interface.floor_changed(floor)

    def queue_start_arrived(self):
        self._interface.queue_start_arrived()

    def queue_proceed(self):
        self._interface.queue_proceed()

    def queue_target_arrived(self):
        self._interface.queue_target_arrived()

    def please_pass_door(self):
        self._interface.please_pass_door()

    def door_passed(self):
        self._interface.door_passed()

    def please_follow_behind(self):
        self._interface.please_follow_behind()

    def please_return_position(self):
        self._interface.please_return_position()

    def announce_social(self, message):
        self._interface.announce_social(message)

    def request_sound(self, sound):
        self._interface.request_sound(sound)

    def system_pause_navigation(self, callback):
        def done_callback():
            self._status_manager.set_state(State.in_pause)
            self._logger.info("NavigationState: paused (system)")
            callback()
        self._status_manager.set_state(State.in_pausing)
        self._navigation.pause_navigation(done_callback)

    # endregion NavigationInterface

    def _event_callback(self, msg):
        self._logger.info(f"event received: {msg.data}")
        if msg.data == "navigation_tmp_finishchat":
            self._logger.info("NavigationState: Finish chat")
            self._exploration.set_conversation_control(False)
            return

        if msg.data == "navigation_tmp_finishchat" or msg.data == "navigation_finishchat" or msg.data == "navigation_finish_button_control":
            self._logger.info("NavigationState: Finish chat")
            self._exploration.set_conversation_control(False)
            self._exploration.set_button_control(False)
            return
    
        event = BaseEvent.parse(msg.data)
        if event is None:
            self._logger.error("cabot event %s cannot be parsed", msg.data)
            return
        try:
            self.process_event(event)
        except:  # noqa: #722
            self._logger.error(traceback.format_exc())

    def reset(self):
        """reset menu"""
        # if self.main_menu:
        #     self.main_menu.reset()
        # self.menu_stack = [self.main_menu]

    # menu delegate method
    def menu_selected(self, menu):
        self._logger.debug(F"menu_selected, {menu.identifier}, {menu.type}")
        if menu.identifier == "destination_menu":
            event = NavigationEvent("destination", menu.value.value)
            self.process_event(event)

        # if menu.identifier == "main_menu" and menu.value is not None:
        #     self._logger.info(menu.value)
        #     self._logger.info(menu.value.identifier)
        #     if menu.value.identifier == "exploration_menu":
        #         event = ExplorationEvent("start")
        #         self.process_event(event)

    # event delegate method
    def process_event(self, event):
        '''
        all events go through this method
        '''
        self._logger.info(f"process_event {str(event)}")

        self._event_mapper.push(event, self._logger)
        self._process_menu_event(event)
        self._process_navigation_event(event)
        self._process_exploration_event(event)

    def _process_menu_event(self, event):
        '''
        process only menu event
        '''
        if event.type != MenuEvent.TYPE:
            return

        curr_menu = self.menu_stack[-1]
        if event.subtype == "next":
            curr_menu.next()
            self._interface.menu_changed(menu=curr_menu)

        elif event.subtype == "prev":
            curr_menu.prev()
            self._interface.menu_changed(menu=curr_menu)

        elif event.subtype == "select":
            selected = curr_menu.select()
            if selected is None:  # from main menu
                if curr_menu.value is None:
                    curr_menu.next()
                selected = curr_menu
            elif not selected.can_explore:
                self.reset()
            elif selected is not curr_menu:
                self.menu_stack.append(selected)
                if selected.value is None:
                    selected.next()

            self._interface.menu_changed(menu=selected, usage=True)

        elif event.subtype == "back":
            if len(self.menu_stack) > 1:
                self.menu_stack.pop()
                curr_menu = self.menu_stack[-1]

            self._interface.menu_changed(menu=curr_menu, backed=True)

            self.speed = 0
            # self.cancel_pub.publish(True)
            self._navigation.pause_navigation()

    def _process_navigation_event(self, event):
        try:
            self._logger.info(f"[CabotUIManager] event type: {event.type}; subtype: {event.subtype}")
        except Exception as e:
            self._logger.error(f"[CabotUIManager] event {event}")
        if event.type != NavigationEvent.TYPE:
            return
        self._logger.info(f"_process_navigation_event {event}")

        # operations indepent from the navigation state
        if event.subtype == "language":
            self._interface.change_language(event.param)
            return

        if event.subtype == "reqfeatures":
            self.send_handleside()
            self.send_touchmode()
            self.send_speaker_audio_files()
            return

        if event.subtype == "handleside":
            self._logger.info("calling set_handle_side")
            self._navigation.set_handle_side(event.param)
            return

        if event.subtype == "touchmode":
            self._logger.info("calling set_touch_mode")
            self._navigation.set_touch_mode(event.param)
            return

        # ignore get event
        if event.subtype == "getlanguage":
            return
        if event.subtype == "gethandleside":
            return
        if event.subtype == "gettouchmode":
            return
        if event.subtype == "getspeakeraudiofiles":
            return

        if event.subtype == "speedup":
            self.speed_menu.prev()
            self._interface.speed_changed(speed=self.speed_menu.description)
            e = NavigationEvent("sound", "SpeedUp")
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)

        if event.subtype == "speeddown":
            self.speed_menu.next()
            self._interface.speed_changed(speed=self.speed_menu.description)
            e = NavigationEvent("sound", "SpeedDown")
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)

        if event.subtype == "event":
            self._navigation.process_event(event)

        if event.subtype == "arrived":
            self.destination = None

        if event.subtype == "decision":
            if self.destination is None:
                self._logger.info("NavigationState: Subtour")
                e = NavigationEvent("subtour", None)
                msg = std_msgs.msg.String()
                msg.data = str(e)
                self._eventPub.publish(msg)

        # deactivate control
        if event.subtype == "idle":
            self._logger.info("NavigationState: Pause control = True")
            self._interface.set_pause_control(True)
            self._navigation.set_pause_control(True)

        def description_callback(result):
            try:
                if result:
                    self._logger.info(f"description - {result=}")
                    self._interface.describe_surround(result['translated'])
                else:
                    self._logger.info("description - Error")
                    self._interface.describe_error()
            except:   # noqa: #722
                self._logger.error(traceback.format_exc())

        if event.subtype == "description" and self._description.enabled:
            # TODO: needs to reset last_plan_distance when arrived/paused
            self._logger.info(F"Request Description duration={event.param}")
            if self._interface.last_pose:
                gp = self._interface.last_pose['global_position']
                cf = self._interface.last_pose['current_floor']
                length_index = min(2, int(event.param) - 1)   # 1 sec -> 0, 2 sec -> 1, < 3 sec -> 2
                if self._description.stop_reason_enabled and self._description.surround_enabled:
                    if length_index <= 1:
                        self._interface.requesting_describe_surround_stop_reason()
                    else:
                        self._interface.requesting_describe_surround()
                elif self._description.stop_reason_enabled and not self._description.surround_enabled:
                    self._interface.requesting_describe_surround_stop_reason()
                elif not self._description.stop_reason_enabled and self._description.surround_enabled:
                    self._interface.requesting_describe_surround()
                self._description.request_description_with_images1(gp, cf, self._interface.lang, length_index=length_index, callback=description_callback)

        # request description internal functions
        def request_stop_reason_description():
            self._logger.info("description - Request Stop Reason Description")
            try:
                if self._interface.last_pose:
                    gp = self._interface.last_pose['global_position']
                    cf = self._interface.last_pose['current_floor']
                    if self._description.request_description_with_images2(
                            gp,
                            cf,
                            "stop_reason",
                            self._interface.lang,
                            length_index=0,
                            callback=description_callback):
                        self._interface.requesting_describe_surround_stop_reason()
                    else:
                        self._interface.requesting_please_wait()
            except:  # noqa: #722
                self._logger.error(traceback.format_exc())

        def request_surround_description():
            self._logger.info(F"description - Request Surround Description (Duration: {event.param})")
            try:
                if self._interface.last_pose:
                    length_index = event.param
                    gp = self._interface.last_pose['global_position']
                    cf = self._interface.last_pose['current_floor']
                    if self._description.request_description_with_images2(
                            gp,
                            cf,
                            "surround",
                            self._interface.lang,
                            length_index=length_index,
                            callback=description_callback):
                        self._interface.requesting_describe_surround()
                    else:
                        self._interface.requesting_please_wait()
            except:  # noqa: #722
                self._logger.error(traceback.format_exc())

        if event.subtype == "description_stop_reason" and self._description.enabled:
            request_stop_reason_description()

        if event.subtype == "description_surround" and self._description.enabled:
            request_surround_description()

        if event.subtype == "speaker_enable":
            self._interface.enable_speaker(event.param)
            return

        if event.subtype == "speaker_alert":
            self._interface.speaker_alert()
            return

        if event.subtype == "speaker_audio_file":
            self._interface.set_audio_file(event.param)
            return

        if event.subtype == "speaker_volume":
            self._interface.set_speaker_volume(event.param)
            return

        if event.subtype == "toggle_speak_state":
            self._logger.info("Request Toggle TTS State")
            e = NavigationEvent("togglespeakstate", None)
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)

        if event.subtype == "toggle_conversation":
            self._logger.info("Request Start/Stop Conversation Interface")
            e = NavigationEvent("toggleconversation", None)
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)

        # operations depents on the current navigation state
        if self._status_manager.state == State.in_preparation:
            self.activity_log("cabot_ui/navigation", "in preparation")
            self._interface.in_preparation()
            return

        if event.subtype == "destination":
            self._logger.info(f"NavigationState: {self._status_manager.state}")
            if self._status_manager.state != State.idle:
                self.activity_log("cabot_ui/navigation", "destination", "need to cancel")

                def done_callback():
                    self._status_manager.set_state(State.idle)
                    self._process_navigation_event(event)
                self._navigation.cancel_navigation(done_callback)
                return

            self._logger.info(F"Destination: {event.param}")
            self._retry_count = 0
            self.destination = event.param
            # change handle mode
            request = std_srvs.srv.SetBool.Request()
            request.data = True
            if self._touchModeProxy.wait_for_service(timeout_sec=1):
                self._touchModeProxy.call_async(request)
                # response: std_srvs.srv.SetBool.Response = self._touchModeProxy.call(request)
                # if not response.success:
                #     self._logger.error("Could not set touch mode to True")
            else:
                self._logger.error("Could not find set touch mode service")

            if self._userSpeedEnabledProxy.wait_for_service(timeout_sec=1):
                self._userSpeedEnabledProxy.call_async(request)
                # response = self._userSpeedEnabledProxy.call(request)
                # if not response.success:
                #     self._logger.info("Could not set user speed enabled to True")
            else:
                self._logger.error("Could not find set user speed enabled service")

            # change state
            # change to waiting_action by using actionlib
            self._status_manager.set_state(State.in_action)
            self._navigation.set_destination(event.param)

        if event.subtype == "summons":
            if self._status_manager.state != State.idle:
                self.activity_log("cabot_ui/navigation", "summons", "need to cancel")

                def done_callback():
                    self._status_manager.set_state(State.idle)
                    self._process_navigation_event(event)
                self._navigation.cancel_navigation(done_callback)
                return

            self._logger.info(F"Summons Destination: {event.param}")
            self.destination = event.param
            # change handle mode
            request = std_srvs.srv.SetBool.Request()
            request.data = False
            if self._touchModeProxy.wait_for_service(timeout_sec=1):
                response: std_srvs.srv.SetBool.Response = self._touchModeProxy.call(request)
                if not response.success:
                    self._logger.info("Could not set touch mode to False")
            else:
                self._logger.error("Could not find set touch mode service")

            if self._userSpeedEnabledProxy.wait_for_service(timeout_sec=1):
                response = self._userSpeedEnabledProxy.call(request)
                if not response.success:
                    self._logger.info("Could not set user speed enabled to False")
            else:
                self._logger.error("Could not find set user speed enabled service")

            # change state
            # change to waiting_action by using actionlib
            self._status_manager.set_state(State.in_summons)
            self._navigation.set_destination(event.param)

        if event.subtype == "cancel":
            self._logger.info("NavigationState: User Cancel requested")
            if self._status_manager.state != State.idle:
                self._logger.info("NavigationState: canceling (user)")
                self._interface.cancel_navigation()

                def done_callback():
                    self.in_navigation = False
                    self.destination = None
                    self._status_manager.set_state(State.idle)
                    self._logger.info("NavigationState: canceled (user)")
                self._navigation.cancel_navigation(done_callback)
            else:
                self._logger.info("NavigationState: state is not in action state={}".format(self._status_manager.state))

        if event.subtype == "pause":
            self._logger.info("NavigationState: User Pause requested")
            if self._status_manager.state == State.in_action or \
               self._status_manager.state == State.in_summons:
                self._logger.info("NavigationState: pausing (user)")

                def done_callback():
                    self._status_manager.set_state(State.in_pause)
                    self._logger.info("NavigationState: paused (user)")
                self._status_manager.set_state(State.in_pausing)
                self._interface.pause_navigation()
                self._navigation.pause_navigation(done_callback)
            else:
                # force to pause state
                self._logger.info("NavigationState: state is not in action state={}".format(self._status_manager.state))

        if event.subtype in {"resume", "resume_or_stop_reason"}:
            if self.destination is not None:
                self._logger.info("NavigationState: User Resume requested")
                if self._status_manager.state == State.in_pause:
                    self._logger.info("NavigationState: resuming (user)")
                    self._interface.resume_navigation()
                    self._status_manager.set_state(State.in_action)
                    self._navigation.resume_navigation()
                    self._logger.info("NavigationState: resumed (user)")
                elif self._status_manager.state == State.in_pausing:
                    self._interface.pausing_navigation()
                else:
                    if event.subtype == "resume_or_stop_reason" and self._description.enabled:
                        request_stop_reason_description()
                    else:
                        self._logger.info("NavigationState: state is not in pause state")
            else:
                self._logger.info("NavigationState: Next")
                e = NavigationEvent("next", None)
                msg = std_msgs.msg.String()
                msg.data = str(e)
                self._eventPub.publish(msg)

            # activate control
            self._logger.info("NavigationState: Pause control = False")
            self._interface.set_pause_control(False)
            self._navigation.set_pause_control(False)

    def _process_exploration_event(self, event):
        self._logger.info(f"process_exploration_event {str(event)}")
        if event.type != ExplorationEvent.TYPE:
            return
        in_conversation = self._exploration.get_conversation_control()
        in_button_control = self._exploration.get_button_control()

        try:
            self._logger.info(f"[CabotUIManager] event type: {event.type}; subtype: {event.subtype}, State: in_conversation={in_conversation}, in_button_control={in_button_control}")
        except Exception as e:
            self._logger.error(f"[CabotUIManager] event {event}")

        if event.subtype == "front":
            if in_button_control:
                # self._interface.exploring_direction("front")
                # self._interface.vibrate(vibration.FRONT)
                self._exploration.send_query("direction","front")
            else:
                # speed up
                self.speed_menu.prev()
                e = NavigationEvent("sound", "SpeedUp")
                msg = std_msgs.msg.String()
                msg.data = str(e)
                self._eventPub.publish(msg)
                self._logger.info("[CHILOG] [SPEED] [UP]")

        elif event.subtype == "back":
            if in_button_control:
                # self._interface.exploring_direction("back")
                self._exploration.send_query("direction","back")
            else:
                # speed down
                self.speed_menu.next()
                e = NavigationEvent("sound", "SpeedDown")
                msg = std_msgs.msg.String()
                msg.data = str(e)
                self._eventPub.publish(msg)
                self._logger.info("[CHILOG] [SPEED] [DOWN]")

        elif event.subtype == "left":
            if in_button_control:
                # self._interface.exploring_direction("left")
                # self._interface.vibrate(vibration.LEFT_TURN)
                self._exploration.send_query("direction","left")
            else:
                self.update_persona(direction=-1)
                self._interface.update_persona(self.persona_list[self.persona_index])
        elif event.subtype == "right":
            if in_button_control:
                # self._interface.exploring_direction("right")
                # self._interface.vibrate(vibration.RIGHT_TURN)
                self._exploration.send_query("direction","right")
            else:   
                self.update_persona(direction=+1)
                self._interface.update_persona(self.persona_list[self.persona_index])

        elif event.subtype == "chat": 
            self._logger.info(f"received event: {event.subtype}")
            self._logger.info("Processing Chat event")
            if in_conversation:
                self._logger.info("[CHILOG] [BUTTON] [FINISH_CHAT]")
                self._logger.info("NavigationState: Finish chat")
                self._interface.finish_chat()
                self.publish_event("finishchat")
                self._exploration.set_conversation_control(False)
            else:
                self._logger.info("[CHILOG] [BUTTON] [START_CHAT]")
                self._logger.info("NavigationState: Start chat")
                self._interface.speak("")
                self._interface.start_chat()
                self.publish_event("startchat")  # use this to trigger smartphone conversation UI
                self._exploration.set_conversation_control(True)

        elif event.subtype == "button_control":
            self._logger.info("Processing Button Control event")
            if in_conversation:
                self._logger.info("[CHILOG] [BUTTON] [FINISH_CHAT]")
                self._logger.info("Switching from conversation to button control")
                self._interface.finish_chat()  
                self.publish_event("finishchat")
                self._exploration.set_conversation_control(False)
                return

            if in_button_control:
                self._logger.info("[CHILOG] [BUTTON] [FINISH_BUTTON_CONTROL]")
                self._logger.info("NavigationState: Finish button control")
                self._interface.speak("")
                self._interface.set_button_control(False)
                self.publish_event("finish_button_control")
                self._exploration.set_button_control(False)
            else:
                self._logger.info("[CHILOG] [BUTTON] [START_BUTTON_CONTROL]")
                self._logger.info("NavigationState: Start button control")
                self._interface.set_button_control(True)
                self.publish_event("button_control")
                self._exploration.set_button_control(True)

        if event.subtype == "wheel_switch":
            pause_control = self._exploration.get_pause_control()
            if pause_control:
                self._logger.info("[CHILOG] [BUTTON] [WHEEL_SWITCH_START]")
                self._logger.info("NavigationState: Pause control = False")
                self._eventPub.publish(std_msgs.msg.String(data="navigation;cancel"))
                self._interface.set_pause_control(False)
                self._navigation.set_pause_control(False)
                self._exploration.set_pause_control(False)
            else:
                self._logger.info("[CHILOG] [BUTTON] [WHEEL_SWITCH_STOP]")
                self._logger.info("NavigationState: Pause control = True")
                self._interface.set_pause_control(True)
                self._navigation.set_pause_control(True)
                self._exploration.set_pause_control(True)

        if event.subtype == "reset_navigation":
            self._logger.info("NavigationState: Reset Navigation")
            self._eventPub.publish(std_msgs.msg.String(data="navigation;cancel"))

    def publish_event(self, event):
        msg = std_msgs.msg.String()
        e = NavigationEvent(event, None)
        msg.data = str(e)
        self._eventPub.publish(msg)

    def update_persona(self, direction):
        self._logger.info(f"Updating persona: {self.persona_index}, to direction: {direction}")
        self.persona_index += direction
        if self.persona_index < 0:
            self.persona_index = 2
        elif self.persona_index > 2:
            self.persona_index = 0
        self._logger.info(f"New persona index: {self.persona_index}, persona: {self.persona_list[self.persona_index]}")
        self._logger.info(f"[CHILOG] [BUTTON] [PERSONA] [{self.persona_list[self.persona_index]}]")
        msg = std_msgs.msg.String()
        msg.data = self.persona_list[self.persona_index]
        self._personaPub.publish(msg)


class EventMapper1(object):

    def __init__(self, delegate):
        self.delegate = delegate
        self.clearWaiters = False
        self._manager = StatusManager.get_instance()
        self.description_duration = 0
        self.mode = "exploration"
        self.exploration_mode = self.delegate.default_mode
        self.lock = threading.RLock()
        self.cv = threading.Condition(self.lock)
        self.wheelsLocked = False
        self.lock2 = threading.RLock()
            
    def late_initialize(self):
        if self.exploration_mode == ExplorationMode.MANUAL:
            CabotUIManager.instance._interface.set_pause_control(True)
            CabotUIManager.instance._navigation.set_pause_control(True)
            CabotUIManager.instance._exploration.set_pause_control(True)
            speak_text("自由走行モードに切り替えました。", force=True)
        elif self.exploration_mode == ExplorationMode.SHARED:
            CabotUIManager.instance._interface.set_pause_control(False)
            CabotUIManager.instance._navigation.set_pause_control(False)
            CabotUIManager.instance._exploration.set_pause_control(False)
            e = ExplorationEvent(subtype="button_control")
            self.delegate.process_event(e)
            speak_text("共有走行モードに切り替えました。", force=True)
        elif self.exploration_mode == ExplorationMode.AUTONOMOUS:
            CabotUIManager.instance._interface.set_pause_control(False)
            CabotUIManager.instance._navigation.set_pause_control(False)
            CabotUIManager.instance._exploration.set_pause_control(False)
            speak_text("自律走行モードに切り替えました。", force=True)
        elif self.exploration_mode == ExplorationMode.TOTAL_FREE:
            # in TOTAL_FREE mode, wheels are always unlocked
            CabotUIManager.instance._interface.set_pause_control(True)
            CabotUIManager.instance._navigation.set_pause_control(True)
            CabotUIManager.instance._exploration.set_pause_control(True)
            speak_text("全自由走行モードに切り替えました。", force=True)

    def checkLidarLimit(self, logger, lidar_dist, speedOverwritePub, turnSpeedOverwritePub, ui_manager):

        # lidar_dist = 0.0

        # if hasattr(lidar_limit, "ranges"):
        #     ranges = [x for x in lidar_limit.ranges if not math.isnan(x) and not math.isinf(x) and x >= lidar_limit.range_min and x <= lidar_limit.range_max]
        #     if len(ranges) > 0:
        #         lidar_dist = min(ranges)
        #     else:
        #         lidar_dist = float('inf')


        logger.info(f"Checking Lidar Limit: {lidar_dist}")
        if lidar_dist <= ui_manager.free_mode_detect_lidar_max_limit_speed:            
            with self.lock:
                logger.info("Lidar limit reached in MANUAL mode")

                if self.exploration_mode != ExplorationMode.MANUAL:
                    return

                if self.clearWaiters:
                    return
                
                # we go backward a bit to avoid being stuck

                turn_speed = 0.0
                turn_speed_msg = std_msgs.msg.Float32()
                turn_speed_msg.data = turn_speed
                turnSpeedOverwritePub.publish(turn_speed_msg)

                backward_speed = 0.00001
                speed_msg = std_msgs.msg.Float32()
                speed_msg.data = -backward_speed
                speedOverwritePub.publish(speed_msg)


                # Obstacle detected in MANUAL mode
                self.wheelsLocked = True
                CabotUIManager.instance._interface.set_pause_control(False)
                CabotUIManager.instance._navigation.set_pause_control(False)
                CabotUIManager.instance._exploration.set_pause_control(False)

                posX = ui_manager.odom_x - ui_manager.map_x
                posY = ui_manager.odom_y - ui_manager.map_y

                # Local offset, check forward direction
                forwardDistance = ui_manager.free_mode_warn_forward_distance  # meters
                checkXForward = int((posX + forwardDistance * math.cos(ui_manager.odom_orientation)) / ui_manager.map_resolution)
                checkYForward = int((posY + forwardDistance * math.sin(ui_manager.odom_orientation)) / ui_manager.map_resolution)

                leftSideDistance = ui_manager.free_mode_warn_left_side_distance  # meters
                leftForwardDistance = ui_manager.free_mode_warn_left_forward_distance  # meters
                checkXLeft = int((posX + leftSideDistance * math.cos(ui_manager.odom_orientation + math.pi / 2) + leftForwardDistance * math.cos(ui_manager.odom_orientation)) / ui_manager.map_resolution)
                checkYLeft = int((posY + leftSideDistance * math.sin(ui_manager.odom_orientation + math.pi / 2) + leftForwardDistance * math.sin(ui_manager.odom_orientation)) / ui_manager.map_resolution)

                rightSideDistance = ui_manager.free_mode_warn_right_side_distance  # meters
                rightForwardDistance = ui_manager.free_mode_warn_right_forward_distance  # meters
                checkXRight = int((posX + rightSideDistance * math.cos(ui_manager.odom_orientation - math.pi / 2) + rightForwardDistance * math.cos(ui_manager.odom_orientation)) / ui_manager.map_resolution)
                checkYRight = int((posY + rightSideDistance * math.sin(ui_manager.odom_orientation - math.pi / 2) + rightForwardDistance * math.sin(ui_manager.odom_orientation)) / ui_manager.map_resolution)

                backDistance = ui_manager.free_mode_warn_back_distance  # meters
                checkXBack = int((posX - backDistance * math.cos(ui_manager.odom_orientation)) / ui_manager.map_resolution)
                checkYBack = int((posY - backDistance * math.sin(ui_manager.odom_orientation)) / ui_manager.map_resolution)
                logger.info(f"Checking obstacle costs at positions - Forward: ({checkXForward}, {checkYForward}), Left: ({checkXLeft}, {checkYLeft}), Right: ({checkXRight}, {checkYRight}), Back: ({checkXBack}, {checkYBack})")

                # Determine the direction where there can be an obstacle (costmap not null)
                forwardCost = 100 # out of map is treated as obstacle
                leftCost = 100
                rightCost = 100
                backCost = 100

                if 0 <= checkXForward < ui_manager.map_width and 0 <= checkYForward < ui_manager.map_height:
                    forwardCost = ui_manager.map_data[checkYForward, checkXForward]
                if 0 <= checkXLeft < ui_manager.map_width and 0 <= checkYLeft < ui_manager.map_height:
                    leftCost = ui_manager.map_data[checkYLeft, checkXLeft]
                if 0 <= checkXRight < ui_manager.map_width and 0 <= checkYRight < ui_manager.map_height:
                    rightCost = ui_manager.map_data[checkYRight, checkXRight]
                if 0 <= checkXBack < ui_manager.map_width and 0 <= checkYBack < ui_manager.map_height:
                    backCost = ui_manager.map_data[checkYBack, checkXBack]

                logger.info(f"Obstacle Costs - Forward: {forwardCost}, Left: {leftCost}, Right: {rightCost}, Back: {backCost}")

                warnCostThreshold = ui_manager.free_mode_warn_cost_threshold  # Cost threshold to consider as obstacle level

                textForward = "前方"
                textLeft = "左側"
                textRight = "右側"
                textBack = "後方"

                textAnd = "と"

                obstacleDirections = []
                if forwardCost >= warnCostThreshold:
                    obstacleDirections.append(textForward)
                if leftCost >= warnCostThreshold:
                    obstacleDirections.append(textLeft)
                if rightCost >= warnCostThreshold:
                    obstacleDirections.append(textRight)
                if backCost >= warnCostThreshold:
                    obstacleDirections.append(textBack)

                if len(obstacleDirections) == 0:
                    # Get the direction with the highest cost
                    maxCost = max(forwardCost, leftCost, rightCost, backCost)
                    if maxCost == forwardCost:
                        obstacleDirections.append(textForward)
                    elif maxCost == leftCost:
                        obstacleDirections.append(textLeft)
                    elif maxCost == rightCost:
                        obstacleDirections.append(textRight)
                    elif maxCost == backCost:
                        obstacleDirections.append(textBack)

                if len(obstacleDirections) == 1:
                    speak_text(f"ご注意{obstacleDirections[0]}に障害物があります。", force=True)

                elif len(obstacleDirections) == 2:
                    speak_text(f"ご注意{obstacleDirections[0]}{textAnd}{obstacleDirections[1]}に障害物があります。", force=True)

                elif len(obstacleDirections) > 2:
                    allButLast = "、".join(obstacleDirections[:-1])
                    last = obstacleDirections[-1]
                    speak_text(f"ご注意{allButLast}{textAnd}{last}に障害物があります。", force=True)
                else:
                    speak_text("ご注意障害物を検知しました。", force=True)

                logger.info("Stopping robot due to obstacle in MANUAL mode")

                time.sleep(ui_manager.free_mode_stop_duration)  # wait for 1.0 seconds

                if ui_manager._touchHandle or not ui_manager.free_mode_correction_touch_required:
                    # If there is an obstacle in front but not in back, go backward a bit (Macro 1)
                    if ui_manager.free_mode_correction_back_enabled and textForward in obstacleDirections and textBack not in obstacleDirections:
                        # go backward a bit
                        backward_speed = ui_manager.free_mode_correction_back_speed
                        speed_msg = std_msgs.msg.Float32()
                        speed_msg.data = -backward_speed
                        speedOverwritePub.publish(speed_msg)
                        time.sleep(ui_manager.free_mode_correction_back_duration)

                    # If there is an obstacle on the left but not in front or on the right, turn right a bit and go forward (Macro 2)
                    elif ui_manager.free_mode_correction_side_enabled and textLeft in obstacleDirections and textForward not in obstacleDirections and textRight not in obstacleDirections:
                        # turn right a bit and go forward
                        forward_speed = ui_manager.free_mode_correction_side_speed
                        speed_msg = std_msgs.msg.Float32()
                        speed_msg.data = forward_speed
                        speedOverwritePub.publish(speed_msg)

                        time.sleep(0.05)

                        turn_speed = -ui_manager.free_mode_correction_side_turnspeed
                        turn_speed_msg = std_msgs.msg.Float32()
                        turn_speed_msg.data = turn_speed
                        turnSpeedOverwritePub.publish(turn_speed_msg)

                        time.sleep(ui_manager.free_mode_correction_side_duration)  # wait for 1.0 seconds

                    # If there is an obstacle on the right but not in front or on the left, turn left a bit and go forward (Macro 3)
                    elif ui_manager.free_mode_correction_side_enabled and textRight in obstacleDirections and textForward not in obstacleDirections and textLeft not in obstacleDirections:
                        # turn left a bit and go forward
                        forward_speed = ui_manager.free_mode_correction_side_speed
                        speed_msg = std_msgs.msg.Float32()
                        speed_msg.data = forward_speed
                        speedOverwritePub.publish(speed_msg)

                        time.sleep(0.05)
                        
                        turn_speed = ui_manager.free_mode_correction_side_turnspeed
                        turn_speed_msg = std_msgs.msg.Float32()
                        turn_speed_msg.data = turn_speed
                        turnSpeedOverwritePub.publish(turn_speed_msg)

                        time.sleep(ui_manager.free_mode_correction_side_duration)  # wait for 1.0 seconds

                
                stop_speed_msg = std_msgs.msg.Float32()
                stop_speed_msg.data = 0.0
                speedOverwritePub.publish(stop_speed_msg)
                turn_stop_speed_msg = std_msgs.msg.Float32()
                turn_stop_speed_msg.data = 0.0
                turnSpeedOverwritePub.publish(turn_stop_speed_msg)
                                
                if(not ui_manager.free_mode_switch_autonomous_mode):
                    CabotUIManager.instance._interface.set_pause_control(True)
                    CabotUIManager.instance._navigation.set_pause_control(True)
                    CabotUIManager.instance._exploration.set_pause_control(True)
                else:
                    if self.delegate.free_mode_switch_autonomous_wizard_mode:
                        # Cancel navigation
                        CabotUIManager.instance._navigation.cancel_navigation()
                        self.exploration_mode = ExplorationMode.SHARED
                        speak_text("共有走行モードに切り替えました。", force=True)
                    else:
                        self.exploration_mode = ExplorationMode.AUTONOMOUS
                        speak_text("自律走行モードに切り替えました。", force=True)

                    if ui_manager.free_mode_switch_autonomous_mode_temp:
                        time.sleep(ui_manager.free_mode_switch_autonomous_mode_temp_duration)
                        speak_text("自由走行モードに切り替えました。", force=True)
                        CabotUIManager.instance._interface.set_pause_control(True)
                        CabotUIManager.instance._navigation.set_pause_control(True)
                        CabotUIManager.instance._exploration.set_pause_control(True)
                        self.exploration_mode = ExplorationMode.MANUAL
                               
                #self.exploration_mode = ExplorationMode.MANUAL
                self.wheelsLocked = False
                self.cv.notify_all()

                time.sleep(ui_manager.free_mode_end_userfree_movement_time) # wait for half second before detecting again
                self.clearWaiters = True

            time.sleep(0.2)
            self.clearWaiters = False

                
    def push(self, event, logger):
        # state = self._manager.state

        if event.type != ButtonEvent.TYPE and event.type != ClickEvent.TYPE and \
        event.type != HoldDownEvent.TYPE:
            return

        mevent = None

        # Commented out the code below to enable switching between exploration and navigation modes
        # if self.mode == "exploration":
        #     mevent = self.map_button_to_exploration(event)
        # elif self.mode == "navigation":
        #     mevent = self.map_button_to_navigation(event)


        mevents = self.map_button_to_exploration(event, logger, None)

        if mevents is None or len(mevents) == 0:
            return
        
        for mevent in mevents:
            if mevent:
                self.delegate.process_event(mevent)

    def map_button_to_menu(self, event):
        if event.type == "click" and event.count == 1:
            if event.buttons == cabot_common.button.BUTTON_NEXT:
                return MenuEvent(subtype="next")
            if event.buttons == cabot_common.button.BUTTON_PREV:
                return MenuEvent(subtype="prev")
            if event.buttons == cabot_common.button.BUTTON_SELECT:
                return MenuEvent(subtype="select")
        elif event.type == "click" and event.count == 2:
            if event.buttons == cabot_common.button.BUTTON_SELECT:
                return MenuEvent(subtype="back")
        return None

    def map_button_to_navigation(self, event):
        if event.type == "button" and not event.down and self.description_duration > 0:
            navigation_event = NavigationEvent(subtype="description", param=self.description_duration)
            self.description_duration = 0
            return navigation_event
        if event.type == "button" and event.down:
            # hook button down to triger pause whenever the left button is pushed
            if event.button == cabot_common.button.BUTTON_LEFT:
                return NavigationEvent(subtype="pause")
        if event.type == "click" and event.count == 1:
            if event.buttons == cabot_common.button.BUTTON_RIGHT:
                return NavigationEvent(subtype="resume")
            if event.buttons == cabot_common.button.BUTTON_UP:
                return NavigationEvent(subtype="speedup")
            if event.buttons == cabot_common.button.BUTTON_DOWN:
                return NavigationEvent(subtype="speeddown")
            if event.buttons == cabot_common.button.BUTTON_CENTER:
                return None
        if event.type == HoldDownEvent.TYPE:
            if event.holddown == cabot_common.button.BUTTON_LEFT and event.duration == 3:
                return NavigationEvent(subtype="idle")
            if event.holddown == cabot_common.button.BUTTON_RIGHT:
                # image description is not triggered here, but when button is released
                self.description_duration = event.duration
        '''
        if event.button == cabot_common.button.BUTTON_SELECT:
                return NavigationEvent(subtype="pause")
        if event.type == "click":
            if event.buttons == cabot_common.button.BUTTON_SELECT and event.count == 2:
                return NavigationEvent(subtype="cancel")
            if event.buttons == cabot_common.button.BUTTON_NEXT and event.count == 2:
                return NavigationEvent(subtype="resume")
        '''
        return None

    def map_button_to_exploration(self, event, logger, ui_manager):

        
        if event.type == HoldDownEvent.TYPE: 
            if not self.delegate._allowButtons:
                return

            #if self.current_mode == ExplorationMode.MANUAL:
            if event.holddown == cabot_common.button.BUTTON_UP:
                self.delegate.free_mode_switch_autonomous_mode = True
                speak_text("ウィザードモードに切り替えました。", force=True)

                
            if event.holddown == cabot_common.button.BUTTON_RIGHT:
                self.delegate.free_mode_switch_autonomous_mode = False
                speak_text("ブレーキ・停止モードに切り替えました。", force=True)

            return []
            # else:
            #     if event.holddown == cabot_common.button.BUTTON_UP:
            #         return [ExplorationEvent(subtype="front")]
            #     if event.holddown == cabot_common.button.BUTTON_DOWN:
            #         return [ExplorationEvent(subtype="back")]
            #     if event.holddown == cabot_common.button.BUTTON_LEFT:
            #         return [ExplorationEvent(subtype="left")]
            #     if event.holddown == cabot_common.button.BUTTON_RIGHT:
            #         return [ExplorationEvent(subtype="right")]

        if event.type == "click" and event.count == 1:
            if self.delegate.cabot_vlm_use_button:
                if event.buttons == cabot_common.button.BUTTON_DOWN:
                    self.delegate._vlmButtonPub.publish(std_msgs.msg.Bool(data=True))
                    return []


            with self.lock:
                self.cv.wait_for(lambda: not self.wheelsLocked, timeout=1.0)

                if not self.delegate._allowButtons:
                    return

                new_mode = self.exploration_mode

                if event.buttons == cabot_common.button.BUTTON_DOWN:
                    new_mode = ExplorationMode.TOTAL_FREE
                elif event.buttons == cabot_common.button.BUTTON_LEFT:
                    new_mode = ExplorationMode.MANUAL
                elif event.buttons == cabot_common.button.BUTTON_RIGHT:
                    new_mode = ExplorationMode.SHARED
                elif event.buttons == cabot_common.button.BUTTON_UP:
                    new_mode = ExplorationMode.AUTONOMOUS

                # CHECK THE NEW MODE IS ALLOWED
                if not new_mode in self.delegate.allowed_modes:
                    logger.info(f"Mode request not allowed by the env settings : {new_mode.name}")
                    return

                logger.info(f"Requested exploration mode change to {new_mode.name}")

                if new_mode == ExplorationMode.MANUAL:
                    CabotUIManager.instance._interface.set_pause_control(True)
                    CabotUIManager.instance._navigation.set_pause_control(True)
                    CabotUIManager.instance._exploration.set_pause_control(True)
                    speak_text("自由走行モードに切り替えました。", force=True)
                elif new_mode == ExplorationMode.SHARED:
                    CabotUIManager.instance._interface.set_pause_control(False)
                    CabotUIManager.instance._navigation.set_pause_control(False)
                    CabotUIManager.instance._exploration.set_pause_control(False)
                    speak_text("共有走行モードに切り替えました。", force=True)
                elif new_mode == ExplorationMode.AUTONOMOUS:
                    CabotUIManager.instance._interface.set_pause_control(False)
                    CabotUIManager.instance._navigation.set_pause_control(False)
                    CabotUIManager.instance._exploration.set_pause_control(False)
                    speak_text("自律走行モードに切り替えました。", force=True)
                elif new_mode == ExplorationMode.TOTAL_FREE:
                    # in TOTAL_FREE mode, wheels are always unlocked
                    CabotUIManager.instance._interface.set_pause_control(True)
                    CabotUIManager.instance._navigation.set_pause_control(True)
                    CabotUIManager.instance._exploration.set_pause_control(True)
                    speak_text("全自由走行モードに切り替えました。", force=True)

                if new_mode != self.exploration_mode:
                    events = []

                    if self.exploration_mode == ExplorationMode.SHARED or new_mode == ExplorationMode.SHARED:
                        # switching to or from SHARED mode resets button_control
                        events.append(ExplorationEvent(subtype="button_control"))
                        
                    self.exploration_mode = new_mode
                    return events
            
        return None



class EventMapper2(object):
    def __init__(self):
        self._manager = StatusManager.get_instance()
        self.button_hold_down_duration = 0
        self.button_hold_down_duration_prev = 0

    def push(self, event):
        if event.type not in {ButtonEvent.TYPE, ClickEvent.TYPE, HoldDownEvent.TYPE}:
            return

        mevent = None

        # simplify the control
        mevent = self.map_button_to_navigation(event)

        '''
        if state == State.idle:
            mevent = self.map_button_to_menu(event)

        elif state == State.in_action or state == State.waiting_action:
            mevent = self.map_button_to_navigation(event)

        elif state == State.in_pause or state == State.waiting_pause:
            mevent = self.map_button_to_navigation(event)
        '''

        if mevent:
            self.delegate.process_event(mevent)

    def map_button_to_menu(self, event):
        if event.type == "click" and event.count == 1:
            if event.buttons == cabot_common.button.BUTTON_NEXT:
                return MenuEvent(subtype="next")
            if event.buttons == cabot_common.button.BUTTON_PREV:
                return MenuEvent(subtype="prev")
            if event.buttons == cabot_common.button.BUTTON_SELECT:
                return MenuEvent(subtype="select")
        elif event.type == "click" and event.count == 2:
            if event.buttons == cabot_common.button.BUTTON_SELECT:
                return MenuEvent(subtype="back")
        return None

    def map_button_to_navigation(self, event):
        if event.type == "button" and not event.down and self.button_hold_down_duration > 0:
            self.button_hold_down_duration = 0
            self.button_hold_down_duration_prev = 0
            return None
        if event.type == "click" and event.count == 1:
            if event.buttons == cabot_common.button.BUTTON_RIGHT:
                return NavigationEvent(subtype="resume_or_stop_reason")
            if event.buttons == cabot_common.button.BUTTON_LEFT:
                return NavigationEvent(subtype="toggle_speak_state")
            if event.buttons == cabot_common.button.BUTTON_UP:
                return NavigationEvent(subtype="description_surround", param=event.count)
            if event.buttons == cabot_common.button.BUTTON_DOWN:
                return NavigationEvent(subtype="toggle_conversation")
            if event.buttons == cabot_common.button.BUTTON_CENTER:
                return NavigationEvent(subtype="decision")
        if event.type == "click" and event.count > 1:
            if event.buttons == cabot_common.button.BUTTON_UP:
                description_length = min(event.count, 3)
                return NavigationEvent(subtype="description_surround", param=description_length)
            if event.buttons == cabot_common.button.BUTTON_RIGHT:
                return NavigationEvent(subtype="speaker_alert")
        if event.type == HoldDownEvent.TYPE:
            if event.holddown == cabot_common.button.BUTTON_LEFT and event.duration == 1:
                return NavigationEvent(subtype="pause")
            if event.holddown == cabot_common.button.BUTTON_LEFT and event.duration == 3:
                return NavigationEvent(subtype="idle")
            if event.holddown in {cabot_common.button.BUTTON_DOWN, cabot_common.button.BUTTON_UP}:
                self.button_hold_down_duration = event.duration
                if self.button_hold_down_duration - self.button_hold_down_duration_prev >= 1:
                    self.button_hold_down_duration_prev = self.button_hold_down_duration
                    return NavigationEvent(subtype="speeddown" if event.holddown == cabot_common.button.BUTTON_DOWN else "speedup")
        return None


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    node.destroy_node()
    #for t in threads:
    #    t.join()
    sys.exit()


signal.signal(signal.SIGINT, receiveSignal)


if __name__ == "__main__":
    rclpy.init()
    node = Node('cabot_ui_manager', start_parameter_services=False)
    nav_node = Node("cabot_ui_manager_navigation", start_parameter_services=False)
    tf_node = Node("cabot_ui_manager_tf", start_parameter_services=False)
    srv_node = Node("cabot_ui_manager_navigation_service", start_parameter_services=False)
    act_node = Node("cabot_ui_manager_navigation_actions", start_parameter_services=False)
    soc_node = Node("cabot_ui_manager_navigation_social", start_parameter_services=False)
    desc_node = Node("cabot_ui_manager_description", start_parameter_services=False)
    nodes = [node, nav_node, tf_node, srv_node, act_node, soc_node, desc_node]
    executors = [SingleThreadedExecutor(), #MultiThreadedExecutor
                 SingleThreadedExecutor(), #MultiThreadedExecutor
                 SingleThreadedExecutor(),
                 SingleThreadedExecutor(),
                 SingleThreadedExecutor(),
                 SingleThreadedExecutor(),
                 SingleThreadedExecutor(),
                 ]
    names = ["node", "tf", "nav", "srv", "act", "soc", "desc"]
    manager = CabotUIManager(node, nav_node, tf_node, srv_node, act_node, soc_node, desc_node)

    threads = []
    for tnode, executor, name in zip(nodes, executors, names):
        def run_node(target_node, executor, name):
            def _run_node():
                # debug code to analyze the bottle neck of nodes
                # high frequency spinning node should have smaller number of waits
                #
                # import time
                # count = 0
                # start = time.time()
                executor.add_node(target_node)
                try:
                    while rclpy.ok():
                        # count += 1
                        # target_node.get_logger().info(f"spin rate {name} {count / (time.time()-start):.2f}Hz - \n"
                        #                               f"  subscriptions {[sub.topic_name for sub in list(target_node.subscriptions)]}\n"
                        #                               f"  timers {len(list(target_node.timers))}\n"
                        #                               f"  clients {[cli.srv_name for cli in list(target_node.clients)]}\n"
                        #                               f"  services {len(list(target_node.services))}\n"
                        #                               f"  guards {len(list(target_node.guards))}\n"
                        #                               f"  waitables {len(list(target_node.waitables))}\n",
                        #                               throttle_duration_sec=1.0)
                        executor.spin_once()
                except KeyboardInterrupt:
                    target_node.get_logger().info(f"Shutting down {name} node")
                except:  # noqa: 722
                    target_node.get_logger().error(traceback.format_exc())
                target_node.destroy_node()
            return _run_node

        thread = threading.Thread(target=run_node(tnode, executor, name))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
