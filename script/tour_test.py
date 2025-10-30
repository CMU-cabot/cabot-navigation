#!/usr/bin/env python3

# Copyright (c) 2025  Carnegie Mellon University
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

# system
import argparse
import math
import os
import subprocess
import sys
import threading
import multiprocessing
import signal
import logging
import traceback
import uuid
import time

# ros2
import rclpy
import rclpy.node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from unique_identifier_msgs.msg import UUID
from action_msgs.msg import GoalStatus
import geometry_msgs.msg
import nav_msgs.msg
from mf_localization_msgs.msg import MFLocalizeStatus
import std_msgs.msg
import people_msgs.msg
import cabot_msgs.msg
from rosgraph_msgs.msg import Clock
# launch
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node as LaunchNode

# others
from tourdata import load_tourdata
from cabot_common import util
from cabot_ui import datautil, event, geoutil, geojson, i18n
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.node_manager import NodeManager
from cabot_ui_plugins.navigation import Navigation
from cabot_ui.navgoal import NavGoal, TurnGoal
from cabot_ui.navgoal import ElevatorWaitGoal, ElevatorInGoal, ElevatorTurnGoal, ElevatorFloorGoal, ElevatorOutGoal
from cabot_ui.social_navigation import SNMessage


class TaskScheduler:
    _instance = None

    def __new__(cls, *args, **kwargs):
        raise NotImplementedError("Please use get_instance() method")

    def __init__(self, *args, **kwargs):
        raise NotImplementedError("Please use get_instance() method")

    @classmethod
    def get_instance(cls, node, interval=0.0001):
        if cls._instance is None:
            obj = super().__new__(cls)
            obj._lock = threading.Lock()
            obj._queue = []
            obj.timer = node.create_timer(interval, obj.process_tasks)
            cls._instance = obj
        return cls._instance

    def add_task(self, task):
        """Add a task (function) to the queue in a thread-safe manner"""
        with self._lock:
            self._queue.append(task)

    def process_tasks(self):
        """
        Retrieve and run all tasks in the queue.
        After executing, the queue will be empty.
        """
        with self._lock:
            tasks = self._queue.copy()
            self._queue.clear()
        for task in tasks:
            task()

    @classmethod
    def setInterval(cls):
        def outer_wrap(function):
            def wrap(*args, **kwargs):
                if cls._instance is not None:
                    cls._instance.add_task(lambda: function(*args, **kwargs))
                return None
            return wrap
        return outer_wrap


class StopReasonsNodeLauncher:
    """Launch `stop_reasons_node` in the background via launch service."""

    def __init__(self, use_sim_time=True, announce_no_touch=False, output='log'):
        multiprocessing.set_start_method('spawn')
        self._process = multiprocessing.Process(
            target=StopReasonsNodeLauncher._run_launch,
            args=(use_sim_time, announce_no_touch, output),
            daemon=True,
        )

    def start(self):
        self._process.start()

    @staticmethod
    def _run_launch(use_sim_time, announce_no_touch, output):
        devnull = open(os.devnull, 'w')
        os.dup2(devnull.fileno(), sys.stdout.fileno())
        os.dup2(devnull.fileno(), sys.stderr.fileno())
        launch_service = LaunchService(noninteractive=True)
        node_action = LaunchNode(
            package="cabot_ui",
            executable="stop_reasons_node",
            name="stop_reasons_node",
            output=output,
            parameters=[{
                'use_sim_time': use_sim_time,
                'announce_no_touch': announce_no_touch,
            }],
            ros_arguments=[
                '--log-level', 'stop_reasons_node:=info'
            ],
        )
        description = LaunchDescription([node_action])
        launch_service.include_launch_description(description)
        try:
            launch_service.run()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                launch_service.shutdown()
            except Exception:
                pass

    def shutdown(self, timeout=5.0):
        if self._process is None:
            return
        if self._process.is_alive():
            try:
                os.kill(self._process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            self._process.join(timeout)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout)
        self._process = None


def get_anchor_file():
    cabot_site = os.environ.get("CABOT_SITE", "")
    if not cabot_site:
        print("CABOT_SITE environment variable not set.")
        sys.exit(1)
    sitedir = get_package_share_directory(cabot_site)
    if not sitedir:
        print("Site directory not found.")
        sys.exit(1)
    config_path = os.path.join(sitedir, "config", "config.sh")
    try:
        map_value = subprocess.check_output(
            ["bash", "-c", f"(sitedir='{sitedir}';gazebo=0;source {config_path};echo $map)"],
            universal_newlines=True,
        ).strip()
    except subprocess.CalledProcessError:
        map_value = ""
    if not map_value:
        print(f"Please check config/config.sh in site package ({sitedir}) to set map and world")
        sys.exit(1)
    return map_value


# override Navigation for testing
# replacing out-facing interfaces (TF/paramters/nav2)
class DummyNavigation(Navigation):
    class DummyBuffer:
        def transform(self, pose_stamped, target):
            return pose_stamped

    class DummyParamManager:
        @TaskScheduler.setInterval()
        def change_parameters(self, params, callback):
            callback(True)

        @TaskScheduler.setInterval()
        def request_parameters(self, params, callback):
            callback({})

    class DummyFuture:
        def __init__(self, obj):
            self._callbacks = []
            self._obj = obj
            self._completed = False

        def result(self):
            return self._obj

        def cancelled(self):
            return False

        def add_done_callback(self, callback):
            self._callbacks.append(callback)

        @TaskScheduler.setInterval()
        def delay_done(self):
            self._completed = True
            for callback in self._callbacks:
                callback(self)

        def done(self):
            return self._completed

    class DummyGoalStatus:
        def __init__(self):
            self.status = GoalStatus.STATUS_SUCCEEDED

    class DummyGoalHandle:
        def __init__(self, goal):
            self.goal = goal
            self.goal_id = UUID(uuid=list(uuid.uuid4().bytes))

        def get_result_async(self):
            self.future = DummyNavigation.DummyFuture(DummyNavigation.DummyGoalStatus())
            DummyNavigation.set_goal_handle(self)
            return self.future

        def completed(self):
            self.future.delay_done()

    class DummyActionClient:
        def __init__(self, ns, action):
            self.ns = ns
            self.action = action

        def send_goal_async(self, goal):
            self.future = DummyNavigation.DummyFuture(DummyNavigation.DummyGoalHandle(goal))
            self.future.delay_done()
            return self.future

    def __init__(self, node_manager, interval):
        self._interval = interval
        super().__init__(node_manager)
        self._current_pose = geoutil.Pose(x=0.0, y=0.0, r=0.0)
        self._dummy_poses = None
        self._dummy_poses_lock = threading.Lock()

        # override
        @TaskScheduler.setInterval()
        def dummy_call_delay(self, func):
            func()
        NavGoal.call_delay = dummy_call_delay
        self.buffer = DummyNavigation.DummyBuffer()
        self.param_manager = DummyNavigation.DummyParamManager()
        for ns in Navigation.NS:
            for action in Navigation.ACTIONS:
                name = "/".join([ns, action])
                self._clients[name] = DummyNavigation.DummyActionClient(ns, action)
        self._spin_client = DummyNavigation.DummyActionClient("/", "spin")

        self.localize_status = MFLocalizeStatus.TRACKING
        self._started = False

        self._node.create_subscription(std_msgs.msg.Float32, "/cabot/map_speed", self._map_speed_callback, 10)
        self.map_speed = 2.0
        self._odom_pub = self._node.create_publisher(nav_msgs.msg.Odometry, "/cabot/odom_raw", 10)
        self._cmd_vel_pub = self._node.create_publisher(geometry_msgs.msg.Twist, "/cmd_vel", 10)
        self._people_speed_pub = self._node.create_publisher(std_msgs.msg.Float32, "/cabot/people_speed", 10)
        self._touch_speed_pub = self._node.create_publisher(std_msgs.msg.Float32, "/cabot/touch_speed_switched", 10)
        self._global_plan_pub = self._node.create_publisher(nav_msgs.msg.Path, "/received_global_plan", 10)
        self._local_global_plan_pub = self._node.create_publisher(nav_msgs.msg.Path, "/local/received_global_plan", 10)
        self._signal_state_pub = self._node.create_publisher(cabot_msgs.msg.SignalState, "/cabot/signal_state", 10)

    def _map_speed_callback(self, msg):
        self.map_speed = msg.data
        # keep simulated signals active even if external control not provided


    @staticmethod
    def _map_signal_state(raw_state):
        mapping = {
            "red": "RED_SIGNAL",
            "green_blinking": "GREEN_SIGNAL_SHORT",
            "green": "NO_SIGNAL_INFO",
        }
        return mapping.get(raw_state, "NO_SIGNAL_INFO")

    def _publish_path(self, poses):
        if not poses:
            return
        stamp = self._node.get_clock().now().to_msg()
        path = nav_msgs.msg.Path()
        path.header.stamp = stamp
        path.header.frame_id = "map"
        for pose in poses:
            pose_stamped = geometry_msgs.msg.PoseStamped()
            pose_stamped.header.stamp = stamp
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = pose.x
            pose_stamped.pose.position.y = pose.y
            orientation = pose.orientation
            pose_stamped.pose.orientation = orientation
            path.poses.append(pose_stamped)
        self._global_plan_pub.publish(path)
        self._local_global_plan_pub.publish(path)

    def _publish_motion(self, prev_pose):
        dt = 0.1
        curr_pose = self._current_pose
        dx = curr_pose.x - prev_pose.x
        dy = curr_pose.y - prev_pose.y
        distance = math.hypot(dx, dy)
        linear_speed = distance / dt
        yaw_diff = geoutil.normalize_angle(curr_pose.r - prev_pose.r)
        angular_speed = yaw_diff / dt

        stamp_msg = self._node.get_clock().now().to_msg()
        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = curr_pose.x
        odom.pose.pose.position.y = curr_pose.y
        orientation = curr_pose.orientation
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = linear_speed
        odom.twist.twist.angular.z = angular_speed
        self._odom_pub.publish(odom)

        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        self._cmd_vel_pub.publish(cmd_vel)

        people_speed = std_msgs.msg.Float32()
        people_speed.data = 1.2
        self._people_speed_pub.publish(people_speed)

        touch_speed = std_msgs.msg.Float32()
        touch_speed.data = 1.2
        self._touch_speed_pub.publish(touch_speed)

    # override
    def current_local_pose(self, frame=None):
        return self._current_pose

    # override
    def current_local_odom_pose(self, frame=None):
        return self._current_pose

    # override
    def current_ros_pose(self, frame=None):
        return geometry_msgs.msg.Pose()

    _goal_handle = None

    @classmethod
    def set_goal_handle(cls, goal_handle):
        cls._goal_handle = goal_handle
        # print(f"Goal handle set: {goal_handle}")

    # override to increase check_loop interval from 0.1 to 0.001
    def _start_loop(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        if self.lock.acquire():
            if self._loop_handle is None:
                self._loop_handle = self._node.create_timer(self._interval, self._check_loop, callback_group=self._main_callback_group)
            self.lock.release()

    def _goto_floor(self, floor, gh_callback, callback):
        self._logger.info(F"go to floor {floor}")
        self.delegate.activity_log("cabot/navigation", "go_to_floor", str(floor))
        future = DummyNavigation.DummyFuture(DummyNavigation.DummyGoalStatus())
        future.add_done_callback(callback)
        future.delay_done()

    # override
    def _check_loop(self):
        prev_pose = geoutil.Pose(x=self._current_pose.x, y=self._current_pose.y, r=self._current_pose.r)
        super()._check_loop()
        self._publish_path(self._dummy_poses)

        if DummyNavigation._goal_handle is None:
            return
        if self._current_goal is None:
            return

        if self.signal_task:
            self.signal_task()

        if self.map_speed == 0.0:
            self._logger.warn("Map speed is 0.0, waiting", throttle_duration_sec=1.0)
            self._publish_motion(prev_pose)
            return

        if isinstance(self._current_goal, NavGoal):
            with self._dummy_poses_lock:
                if self._dummy_poses is None:
                    def make_poses(links):
                        poses = []
                        for link in links:
                            if not isinstance(link, geojson.RouteLink):
                                self._logger.error(f"Invalid link type: {link.__class__}")
                                continue
                            start = link.source_node.local_geometry
                            end = link.target_node.local_geometry
                            r = math.atan2(end.y - start.y, end.x - start.x)
                            count = int(start.distance_to(end) / 0.1) + 1
                            for i in range(0, count + 1):
                                x = start.x + (end.x - start.x) * i / count
                                y = start.y + (end.y - start.y) * i / count
                                pose = geoutil.Pose(x=x, y=y, r=r)
                                poses.append(pose)
                        return poses
                    routes = self._current_goal.separated_route
                    self._dummy_poses = make_poses(routes[self._current_goal.route_index])
                    self._logger.warn(
                        f"NavGoal {self._current_goal.route_index}/{len(routes)} Count = {len(self._dummy_poses)} Last pose = {self._dummy_poses[-1]}"
                    )
                    if not self._started:
                        self._started = True
                        self.delegate.start_navigation()
                else:
                    if self._dummy_poses:
                        self._current_pose = self._dummy_poses.pop(0)
                    else:
                        self._logger.warn("NavGoal completed")
                        DummyNavigation._goal_handle.completed()
                        DummyNavigation._goal_handle = None
                        self._dummy_poses = None
        elif isinstance(self._current_goal, TurnGoal):
            with self._dummy_poses_lock:
                if self._dummy_poses is None:
                    def make_poses(end):
                        poses = []
                        start = self._current_pose.r
                        count = int(math.fabs(end - start) / 0.017)
                        for i in range(0, count + 1):
                            r = start + (end - start) * i / count
                            pose = geoutil.Pose(xy=self._current_pose, r=r)
                            poses.append(pose)
                        return poses
                    self._dummy_poses = make_poses(self._current_goal.yaw_target)
                    self._logger.warn(f"TurnGoal Count = {len(self._dummy_poses)} Last pose = {self._dummy_poses[-1]}")
                else:
                    if self._dummy_poses:
                        self._current_pose = self._dummy_poses.pop(0)
                    else:
                        self._logger.warn("TurnGoal completed")
                        DummyNavigation._goal_handle.completed()
                        DummyNavigation._goal_handle = None
                        self._dummy_poses = None
        elif isinstance(self._current_goal, ElevatorWaitGoal):
            with self._dummy_poses_lock:
                if self._dummy_poses is None:
                    def make_poses(end):
                        poses = []
                        start = self._current_pose.r
                        count = int(math.fabs(end - start) / 0.017)
                        for i in range(0, count + 1):
                            r = start + (end - start) * i / count
                            pose = geoutil.Pose(xy=self._current_pose, r=r)
                            poses.append(pose)
                        return poses
                    pose = geoutil.Pose.pose_from_points(self._current_pose, self._current_goal.cab_poi.door_geometry)
                    self._dummy_poses = make_poses(pose.r)
                    self._logger.warn(f"ElevatorWaitGoal Count = {len(self._dummy_poses)} Last pose = {self._dummy_poses[-1]}")
                else:
                    if self._dummy_poses:
                        self._current_pose = self._dummy_poses.pop(0)
                    else:
                        self._logger.warn("ElevatorWaitGoal completed")
                        DummyNavigation._goal_handle.completed()
                        DummyNavigation._goal_handle = None
                        self._dummy_poses = None
        elif isinstance(self._current_goal, ElevatorInGoal):
            with self._dummy_poses_lock:
                if self._dummy_poses is None:
                    def make_poses(start, end):
                        poses = []
                        r = math.atan2(end.y - start.y, end.x - start.x)
                        count = int(start.distance_to(end) / 0.1) + 1
                        for i in range(0, count + 1):
                            x = start.x + (end.x - start.x) * i / count
                            y = start.y + (end.y - start.y) * i / count
                            pose = geoutil.Pose(x=x, y=y, r=r)
                            poses.append(pose)
                        return poses
                    self._dummy_poses = make_poses(self._current_pose, self._current_goal)
                    self._logger.warn(f"ElevatorInGoal Count = {len(self._dummy_poses)} Last pose = {self._dummy_poses[-1]}")
                    self.delegate.take_elevator()
                    self.delegate.elevator_opening()
                else:
                    if self._dummy_poses:
                        self._current_pose = self._dummy_poses.pop(0)
                    else:
                        self._logger.warn("ElevatorInGoal completed")
                        DummyNavigation._goal_handle.completed()
                        DummyNavigation._goal_handle = None
                        self._dummy_poses = None
        elif isinstance(self._current_goal, ElevatorTurnGoal):
            with self._dummy_poses_lock:
                if self._dummy_poses is None:
                    def make_poses(end):
                        poses = []
                        start = self._current_pose.r
                        count = int(math.fabs(end - start) / 0.017)
                        for i in range(0, count + 1):
                            r = start + (end - start) * i / count
                            pose = geoutil.Pose(xy=self._current_pose, r=r)
                            poses.append(pose)
                        return poses
                    self._dummy_poses = make_poses(self._current_goal.cab_poi.r)
                    self._logger.warn(f"ElevatorTurnGoal Count = {len(self._dummy_poses)} Last pose = {self._dummy_poses[-1]}")
                else:
                    if self._dummy_poses:
                        self._current_pose = self._dummy_poses.pop(0)
                    else:
                        self._logger.warn("ElevatorTurnGoal completed")
                        DummyNavigation._goal_handle.completed()
                        DummyNavigation._goal_handle = None
                        self._dummy_poses = None
        elif isinstance(self._current_goal, ElevatorFloorGoal):
            self._logger.warn("ElevatorFloorGoal completed")
            self.current_floor = self._current_goal.cab_poi.floor
            DummyNavigation._goal_handle.completed()
            DummyNavigation._goal_handle = None
            self._dummy_poses = None
            self.floor_changed(self.current_floor)
        elif isinstance(self._current_goal, ElevatorOutGoal):
            with self._dummy_poses_lock:
                if self._dummy_poses is None:
                    def make_poses(start, end):
                        poses = []
                        r = math.atan2(end.y - start.y, end.x - start.x)
                        count = int(start.distance_to(end) / 0.1) + 1
                        for i in range(0, count + 1):
                            x = start.x + (end.x - start.x) * i / count
                            y = start.y + (end.y - start.y) * i / count
                            pose = geoutil.Pose(x=x, y=y, r=r)
                            poses.append(pose)
                        return poses
                    pose = self._current_pose
                    pose = geoutil.Pose(
                        x=pose.x + math.cos(pose.r) * self._current_goal.set_forward[0] - math.sin(pose.r) * self._current_goal.set_forward[1],
                        y=pose.y + math.sin(pose.r) * self._current_goal.set_forward[0] + math.cos(pose.r) * self._current_goal.set_forward[1],
                        r=pose.r
                    )
                    self._dummy_poses = make_poses(self._current_pose, pose)
                    self._logger.warn(f"ElevatorOutGoal Count = {len(self._dummy_poses)} Last pose = {self._dummy_poses[-1]}")
                else:
                    if self._dummy_poses:
                        self._current_pose = self._dummy_poses.pop(0)
                    else:
                        self._logger.warn("ElevatorOutGoal completed")
                        DummyNavigation._goal_handle.completed()
                        DummyNavigation._goal_handle = None
                        self._dummy_poses = None
                        self.delegate.exit_elevator()
        else:
            self._logger.error(f"Unsupported goal type: {self._current_goal.__class__}")

        self._publish_motion(prev_pose)


class TourTestNode(rclpy.node.Node):
    def __init__(self, node_manager, user_speed=1.0):
        super().__init__("tour_test_node")
        self.node_manager = node_manager
        self._tour_manager = None
        self._done_callback = None
        self._user_speed = user_speed

    def initialize(self, lang="en", debug=False, interval=0.001):
        self.debug = debug
        i18n.set_language(lang)
        i18n.load_from_packages(['cabot_ui'])
        self.navigation = DummyNavigation(self.node_manager, interval)
        self.navigation.delegate = self
        self.event_pub = self.create_publisher(std_msgs.msg.String, "/cabot/event", 10)

    def deinitialize(self):
        self.navigation._loop_handle.cancel()

    def tour(self, tour_id):
        print("TBD")

    def head(self, msg):
        print(f"## {msg}")

    def item(self, msg, depth=0):
        print(f"- {msg}")

    def system(self, msg, depth=0):
        print(f"- ({i18n.localized_string('SYSTEM')}) {msg}")

    def navigate(self, start, goal, done_callback):
        self._done_callback = done_callback
        goal_name = goal
        for landmark in datautil.getInstance().landmarks:
            for ent in landmark.entrances:
                if ent.node._id == goal:
                    goal_name = landmark.name

        self.head(f"{goal_name}")

        def callback(node):
            self.navigation.current_floor = node.floor
            self.navigation._current_pose = geoutil.Pose(xy=node.local_geometry, r=0)
            if goal:
                self.navigation._started = False
                self.navigation.set_destination(goal)
        geojson.Object.get_object_by_id(start, callback)

    def user_speed(self):
        return self._user_speed

    def update_pose(self, *args, **kwargs):
        pass

    def activity_log(self, *args):
        # print(f"activity_log: {args=}")
        pass

    def i_am_ready(self):
        pass

    def start_navigation(self):
        CaBotRclpyUtil.warn("start_navigation")
        if self._tour_manager:
            self._tour_manager.start_navigation()
        self.event_pub.publish(std_msgs.msg.String(data="navigation;event;navigation_start"))
        

    def enter_goal(self, goal):
        CaBotRclpyUtil.warn(f"enter_goal {goal.__class__}")
        pass

    def exit_goal(self, goal):
        CaBotRclpyUtil.warn(f"enter_goal {goal.__class__}")
        pass

    def have_arrived(self, goal):
        CaBotRclpyUtil.warn(f"have_arrived {goal.__class__}")
        if self._tour_manager:
            self._tour_manager.have_arrived(goal)
        pass

    def have_completed(self):
        CaBotRclpyUtil.warn("have_completed")
        if self._tour_manager:
            self._tour_manager.have_completed()
        self.event_pub.publish(std_msgs.msg.String(data="navigation_arrived"))
        self._done_callback()
        pass

    def notify_turn(self, device=None, turn=None):
        pass

    def approaching_to_poi(self, poi=None):
        statement = poi.approaching_statement()
        if statement: 
            self.item(f"({poi.__class__.__name__}) {statement}")

    def approached_to_poi(self, poi=None):
        statement = poi.approached_statement()
        if statement:
            self.item(f"({poi.__class__.__name__}) {statement}")

    def passed_poi(self, poi=None):
        statement = poi.passed_statement()
        if statement:
            self.item(f"({poi.__class__.__name__}) {statement}")

    def please_call_elevator(self, pos):
        if pos:
            self.system(i18n.localized_string("CALL_ELEVATOR_PLEASE_ON_YOUR",
                                            i18n.localized_string(pos)))
        else:
            self.system(i18n.localized_string("CALL_ELEVATOR_PLEASE"))

    def take_elevator(self):
        if self._tour_manager:
            self._tour_manager.take_elevator()

    def exit_elevator(self):
        if self._tour_manager:
            self._tour_manager.exit_elevator()

    def elevator_opening(self):
        self.system(i18n.localized_string("ELEVATOR_IS_OPENING"))

    def floor_changed(self, floor):
        self.system(i18n.localized_string("GETTING_OFF_THE_ELEVATOR"))

    def queue_start_arrived(self):
        self.system(i18n.localized_string("GOING_TO_GET_IN_LINE"))

    def queue_proceed(self):
        self.system(i18n.localized_string("DOOR_POI_USER_ACTION"))

    def door_passed(self):
        self.system(i18n.localized_string("DOOR_POI_PASSED"))

    def please_follow_behind(self):
        self.system(i18n.localized_string("FOLLOW_BEHIND_PLEASE_NARROW"))

    def please_return_position(self):
        self.system(i18n.localized_string("RETURN_TO_POSITION_PLEASE"))

    def announce_social(self, message: SNMessage):
        if message.param is not None:
            if isinstance(message.param, list):
                self.item(i18n.localized_string(message.code.name, *message.param))
            else:
                self.item(i18n.localized_string(message.code.name, message.param))
        else:
            self.item(i18n.localized_string(message.code.name))


class TourManager:
    def __init__(self, tester, node, start, tour, tourdata, lang):
        self.tester = tester
        self.node = node
        self.start = start
        self.tour = tour
        self.tourdata = tourdata
        self.lang = lang
        self.index = 0
        self.current_destination = None
        self.timer = self.node.create_timer(0.1, self.check_tour)
        print(f"# {i18nText(tour.title, args.lang)}")

    def check_tour(self):
        def callback():
            self.current_destination = None
            self.index += 1

        if self.current_destination:
            return

        if self.index >= len(self.tour.destinations):
            rclpy.shutdown()
            return
        dest = self.tour.destinations[self.index]
        self.current_destination = self.tourdata.get_destination(dest.ref)
        self.tester.navigate(self.start, self.current_destination.value, callback)
        self.start = self.current_destination.value  # update start

    def _check_message(self, message_type):
        ref = self.current_destination.value
        if self.current_destination.var:
            ref += "#" + self.current_destination.var
        message = self.tourdata.get_message(ref, message_type)
        if message and message.read is False:
            message.read = True
            self.tester.item(f"({message_type}) {i18nText(message.text, self.lang)}")

    def take_elevator(self):
        self.tester.item(f"**{i18n.localized_string('TAKE_ELEVATOR')}**")

    def exit_elevator(self):
        self.tester.item(f"**{i18n.localized_string('EXIT_ELEVATOR')}**")

    def start_navigation(self):
        self.tester.item(f"**{i18n.localized_string('START_NAVIGATION')}**")
        self._check_message("startMessage")
        pass

    def have_arrived(self, goal):
        self.tester.item(f"**{i18n.localized_string('ARRIVE_DESTINATON')}**")
        self._check_message("arriveMessage")
        pass

    def have_completed(self):
        pass


def i18nText(data, lang):
    if lang in data:
        return data[lang]
    return data


def spin(node):
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except:  # noqa E722
        traceback.print_exc()
    node.destroy_node()


def main(args):
    global scheduler, sim_dt, sim_time_nsec

    stop_reasons_launcher = None
    try:
        rclpy.init(args=[
            "--ros-args",
            "-p",
            "map_server_host:=localhost:9090/map",
            "-p",
            "anchor_file:=" + get_anchor_file(),
            "-p",
            "use_sim_time:=true",
            "--log-level",
            "INFO" if args.debug else "ERROR",
        ])
        node_manager = NodeManager()
        tester = TourTestNode(node_manager, user_speed=args.user_speed)
        node = node_manager.get_node()
        CaBotRclpyUtil.initialize(node)
        scheduler = TaskScheduler.get_instance(node, interval=args.interval)

        node.get_logger().info(f"### Tour Test Node initialized (debug={args.debug})")
        tester.initialize(lang=args.lang, debug=args.debug, interval=args.interval)
        tourdata = load_tourdata()
        clock_pub = node.create_publisher(Clock, "/clock", 10)

        def red(msg):
            return f"\033[91m{msg}\033[0m"

        def blue(msg):
            return f"\033[94m{msg}\033[0m"

        def list_tours(tourdata):
            for tour in tourdata.tours:
                print(f"{tour.tour_id}: {i18nText(tour.title, args.lang)}")

        def list_landmarks():
            for landmark in datautil.getInstance().landmarks:
                for ent in landmark.entrances:
                    print(f"{ent.node._id}: {landmark.name} {ent.name}")

        def list_messages(tourdata):
            for message in tourdata.messages:
                print(f"{message.parent}.{message.type}: {i18nText(message.text, args.lang)}")

        if args.landmarks:
            list_landmarks()
            return

        if args.messages:
            list_messages(tourdata)
            return

        if args.signal:
            import json
            from cabot_site_intersection.signals import SignalGeneratorDummy001RedFirst
            signal_generator = SignalGeneratorDummy001RedFirst(0, step=0.1)
            pub = node.create_publisher(std_msgs.msg.String, '/signal_response_intersection_status', 10)

            try:
                stop_reasons_launcher = StopReasonsNodeLauncher()
                stop_reasons_launcher.start()
            except Exception:
                node.get_logger().error("Failed to launch stop_reasons_node\n" + traceback.format_exc())
                raise

            def signal_task():
                info = signal_generator.next()
                pub.publish(std_msgs.msg.String(data=json.dumps(info)))
            tester.navigation.signal_task = signal_task

            def spin():
                node = rclpy.create_node("clock_publisher")
                clock_pub = node.create_publisher(Clock, "/clock", 10)
                sim_dt = 0.1  # seconds
                sim_time_nsec = 0
                try:
                    while rclpy.ok():
                        sim_time_nsec += int(sim_dt * 1e9)
                        clock = Clock()
                        clock.clock.sec = sim_time_nsec // 1_000_000_000
                        clock.clock.nanosec = sim_time_nsec % 1_000_000_000
                        clock_pub.publish(clock)
                        time.sleep(0.01)
                except:  # noqa E722
                    traceback.print_exc()

            threading.Thread(target=spin, daemon=True).start()

        else:
            tester.navigation.signal_task = None

        if args.goal:
            def callback():
                def task():
                    tester.deinitialize()
                    time.sleep(1)
                    rclpy.shutdown()
                scheduler.add_task(task)
                print(blue("Navigation to goal completed"))

            if args.start is None:
                print(red("Please specify both start and goal nodes."))
                return
            tester.navigate(args.start, args.goal, callback)

            node_manager.join()
            return

        if args.tour_id:
            if args.start is None:
                print(red("Please specify both start node and tour id."))
                return
            tour = next((t for t in tourdata.tours if t.tour_id == args.tour_id), None)
            if tour:
                manager = TourManager(tester, node, args.start, tour, tourdata, args.lang)
                tester._tour_manager = manager
                node_manager.join()
                return
            print(red(f"Tour with id '{args.tour_id}' not found."))
            list_tours(tourdata)
            return

        print(red("You need to specify a start and either a goal or a tour ID"))
        list_tours(tourdata)
        rclpy.shutdown()
        node_manager.join()
    finally:
        if stop_reasons_launcher is not None:
            stop_reasons_launcher.shutdown()


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Fetch and display tour data")
    parser.add_argument("-d", "--debug", action="store_true", help="Show configurable variables")
    parser.add_argument("-l", "--lang", type=str, default="en", help="Language code for i18n text")
    parser.add_argument("-t", "--tour_id", type=str, help="Specify a tour_id to show")
    parser.add_argument("-s", "--start", type=str, help="start node")
    parser.add_argument("-g", "--goal", type=str, help="goal node")
    parser.add_argument("-i", "--interval", type=float, default=0.0001, help="Interval for processing queue")
    parser.add_argument("-L", "--landmarks", action="store_true", help="Show landmarks")
    parser.add_argument("-M", "--messages", action="store_true", help="Show messages")
    parser.add_argument("-S", "--site", type=str, default=None, help="Site package name (overrides CABOT_SITE environment variable)")
    # parser.add_argument("-n", "--signal", type=str, default=None, help="Signal class name for testing")
    parser.add_argument("-n", "--signal", action="store_true", help="Signal testing")
    parser.add_argument("-u", "--user_speed", type=float, default=1.0, help="User walking speed m/s")
    args = parser.parse_args()
    os.environ["CABOT_SITE"] = args.site if args.site else os.environ.get("CABOT_SITE", "")
    main(args)
