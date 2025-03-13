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
import traceback
import uuid

# ros2
import rclpy
import rclpy.node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from unique_identifier_msgs.msg import UUID
from action_msgs.msg import GoalStatus
import geometry_msgs.msg
from mf_localization_msgs.msg import MFLocalizeStatus
import std_msgs.msg

# others
from tourdata import load_tourdata
from cabot_common import util
from cabot_ui import datautil, event, geoutil, geojson, i18n
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.navigation import Navigation, NavigationInterface
from cabot_ui.navgoal import NavGoal, TurnGoal, DoorGoal
from cabot_ui.navgoal import ElevatorGoal, ElevatorWaitGoal, ElevatorInGoal, ElevatorTurnGoal, ElevatorFloorGoal, ElevatorOutGoal



INTERVAL = 0.001
_lock = threading.Lock()
_queue = []


def process_queue():
    global _queue
    if len(_queue) > 0:
        data = None
        with _lock:
            data = _queue.pop(0)
        if data and len(data) == 3:
            try:
                data[0](*data[1], **data[2])
            except:  # noqa E722
                traceback.print_exc()


# utility functions
def setInterval():
    def outer_wrap(function):
        def wrap(*args, **kwargs):
            with _lock:
                _queue.append((function, args, kwargs))
            return None
        return wrap
    return outer_wrap


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


# override call_delay for testing
@setInterval()
def dummy_call_delay(self, func):
    func()


NavGoal.call_delay = dummy_call_delay


# override Navigation for testing
# replacing out-facing interfaces (TF/paramters/nav2)
class DummyNavigation(Navigation):
    class DummyBuffer:
        def transform(self, pose_stamped, target):
            return pose_stamped

    class DummyParamManager:
        @setInterval()
        def change_parameters(self, params, callback):
            callback(True)

        @setInterval()
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

        @setInterval()
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

    def __init__(self, node, interval):
        self._interval = interval
        super().__init__(node, node, node, node, node)
        self._current_pose = geoutil.Pose(x=0, y=0, r=0)
        self._dummy_poses = None
        self._dummy_poses_lock = threading.Lock()
        # override
        self.buffer = DummyNavigation.DummyBuffer()
        self.param_manager = DummyNavigation.DummyParamManager()
        for ns in Navigation.NS:
            for action in Navigation.ACTIONS:
                name = "/".join([ns, action])
                self._clients[name] = DummyNavigation.DummyActionClient(ns, action)
        self._spin_client = DummyNavigation.DummyActionClient("/", "spin")

        self.localize_status = MFLocalizeStatus.TRACKING

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
        super()._check_loop()
        if DummyNavigation._goal_handle is None:
            return
        if self._current_goal is None:
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
                    self._logger.warn(f"NavGoal {self._current_goal.route_index}/{len(routes)} Count = {len(self._dummy_poses)} Last pose = {self._dummy_poses[-1]}")
                    if self._current_goal.route_index == 0:
                        self.delegate.start_navigation()
                else:
                    if len(self._dummy_poses) > 0:
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
                    if len(self._dummy_poses) > 0:
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
                    if len(self._dummy_poses) > 0:
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
                    self.delegate.elevator_opening()
                else:
                    if len(self._dummy_poses) > 0:
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
                    if len(self._dummy_poses) > 0:
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
                    if len(self._dummy_poses) > 0:
                        self._current_pose = self._dummy_poses.pop(0)
                    else:
                        self._logger.warn("ElevatorOutGoal completed")
                        DummyNavigation._goal_handle.completed()
                        DummyNavigation._goal_handle = None
                        self._dummy_poses = None
        else:
            self._logger.error(f"Unsupported goal type: {self._current_goal.__class__}")


class TourTestNode(rclpy.node.Node, NavigationInterface):
    def __init__(self):
        super().__init__("tour_test_node")
        self._tour_manager = None
        self._done_callback = None

    def initialize(self, lang="en", debug=False, interval=0.001):
        self.debug = debug
        i18n.set_language(lang)
        i18n.load_from_packages(['cabot_ui'])
        self.navigation = DummyNavigation(self, interval)
        self.navigation.delegate = self

    def tour(self, tour_id):
        print("TBD")

    def head(self, msg):
        print(f"## {msg}")

    def item(self, msg, depth=0):
        print(f"- {msg}")

    def navigate(self, start, goal, callback):
        self._done_callback = callback
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
                self.navigation.set_destination(goal)
        geojson.Object.get_object_by_id(start, callback)

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
        self._done_callback()
        pass

    def notify_turn(self, device=None, turn=None):
        pass

    def approaching_to_poi(self, poi=None):
        statement = poi.approaching_statement()
        if statement:
            self.item(statement)

    def approached_to_poi(self, poi=None):
        statement = poi.approached_statement()
        if statement:
            self.item(statement)

    def passed_poi(self, poi=None):
        statement = poi.passed_statement()
        if statement:
            self.item(statement)

    def please_call_elevator(self, pos):
        if pos:
            self.item(i18n.localized_string("CALL_ELEVATOR_PLEASE_ON_YOUR",
                                            i18n.localized_string(pos)))
        else:
            self.item(i18n.localized_string("CALL_ELEVATOR_PLEASE"))

    def elevator_opening(self):
        self.item(i18n.localized_string("ELEVATOR_IS_OPENING"))

    def floor_changed(self, floor):
        self.item(i18n.localized_string("GETTING_OFF_THE_ELEVATOR"))

    def queue_start_arrived(self):
        self.item(i18n.localized_string("GOING_TO_GET_IN_LINE"))

    def queue_proceed(self):
        self.item(i18n.localized_string("DOOR_POI_USER_ACTION"))

    def door_passed(self):
        self.item(i18n.localized_string("DOOR_POI_PASSED"))

    def please_follow_behind(self):
        self.item(i18n.localized_string("FOLLOW_BEHIND_PLEASE_NARROW"))

    def please_return_position(self):
        self.item(i18n.localized_string("RETURN_TO_POSITION_PLEASE"))


class TourManager:
    def __init__(self, node, start, tour, tourdata, lang):
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
        self.node.navigate(self.start, self.current_destination.value, callback)
        self.start = self.current_destination.value  # update start

    def _check_message(self, message_type):
        ref = self.current_destination.value
        if self.current_destination.var:
            ref += "#" + self.current_destination.var
        message = self.tourdata.get_message(ref, message_type)
        if message and message.read is False:
            message.read = True
            self.node.item(i18nText(message.text, self.lang))

    def start_navigation(self):
        self._check_message("startMessage")
        pass

    def have_arrived(self, goal):
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
    rclpy.init(args=[
        "--ros-args",
        "-p",
        "map_server_host:=localhost:9090/map",
        "-p",
        "anchor_file:=" + get_anchor_file(),
        "--log-level",
        "INFO" if args.debug else "ERROR",
    ])
    node = TourTestNode()
    CaBotRclpyUtil.initialize(node)
    _ = node.create_timer(args.interval, process_queue)
    node.initialize(lang=args.lang, debug=args.debug, interval=args.interval)
    tourdata = load_tourdata()

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

    if args.goal:
        def callback():
            rclpy.shutdown()
        if args.start is None:
            print(red("Please specify both start and goal nodes."))
            return
        node.navigate(args.start, args.goal, callback)
        spin(node)
        return

    if args.tour_id:
        if args.start is None:
            print(red("Please specify both start node and tour id."))
            return
        tour = next((t for t in tourdata.tours if t.tour_id == args.tour_id), None)
        if tour:
            manager = TourManager(node, args.start, tour, tourdata, args.lang)
            node._tour_manager = manager
            spin(node)
            return
        print(red(f"Tour with id '{args.tour_id}' not found."))
        list_tours(tourdata)
        return

    print(red("You need to specify a start and either a goal or a tour ID"))
    list_tours(tourdata)


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Fetch and display tour data")
    parser.add_argument("-d", "--debug", action="store_true", help="Show configurable variables")
    parser.add_argument("-l", "--lang", type=str, default="en", help="Language code for i18n text")
    parser.add_argument("-t", "--tour_id", type=str, help="Specify a tour_id to show")
    parser.add_argument("-s", "--start", type=str, help="start node")
    parser.add_argument("-g", "--goal", type=str, help="goal node")
    parser.add_argument("-i", "--interval", type=float, default=0.001, help="Interval for processing queue")
    parser.add_argument("-L", "--landmarks", action="store_true", help="Show landmarks")
    parser.add_argument("-M", "--messages", action="store_true", help="Show messages")
    args = parser.parse_args()
    main(args)
