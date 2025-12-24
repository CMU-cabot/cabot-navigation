# Copyright (c) 2020, 2025  Carnegie Mellon University
# Copyright (c) 2024, 2025  ALPS ALPINE CO., LTD.
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

import math
import threading
import time
import traceback

# ROS
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import rclpy.duration
import unique_identifier_msgs.msg
import nav2_msgs.action
import nav_msgs.msg
import std_msgs.msg
import tf_transformations

# Other
from cabot_ui import geojson, geoutil, i18n
from cabot_common import util
import cabot_ui.navgoal as navgoal
from cabot_ui.event import NavigationEvent
from cabot_ui.navgoal import GoalInterface
from cabot_ui.param_manager import ParamManager
from cabot_ui_plugins.navigation import ControlBase
from cabot_ui.plugin import NavigationPlugin
from cabot_ui.node_manager import NodeManager
from cabot_ui.process_queue import ProcessQueue
from cabot_ui.status import State, StatusManager
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

from .phone_interface import PhoneInterface, SpeechPriority


class PhoneNavigation(ControlBase, GoalInterface, NavigationPlugin):
    """Phone Navigation"""

    ACTIONS = {"navigate_to_pose": nav2_msgs.action.NavigateToPose}

    NS = ["/phone"]
    TURN_NEARBY_THRESHOLD = 2
    ROUTE_OVERVIEW_LINE_TOLERANCE = 0.4
    ROUTE_OVERVIEW_POINT_EPS = 0.01
    ROUTE_OVERVIEW_TURN_THRESHOLD_DEG = 15
    ROUTE_OVERVIEW_ANGLE_ROUND_DEG = 5
    ROUTE_OVERVIEW_MIN_DISTANCE = 1.0

    def __init__(self, node_manager: NodeManager):
        node = node_manager.get_node("phone")
        tf_node = node_manager.get_node("tf")
        srv_node = node_manager.get_node("srv")
        act_node = node_manager.get_node("act")
        datautil_instance = None
        anchor_file = None

        self.current_floor = None
        self.current_frame = None
        self._ready = True

        self._status_manager = StatusManager.get_instance("phone")
        self._status_manager.delegate = self
        if self._status_manager.state == State.in_preparation:
            self._status_manager.set_state(State.idle)

        super(PhoneNavigation, self).__init__(
            node, tf_node, datautil_instance=datautil_instance, anchor_file=anchor_file)

        self.param_manager = ParamManager(srv_node)

        self.info_pois = []
        self._sub_goals = None
        self._goal_index = -1
        self._current_goal = None
        self.to_id = None

        self._loop_handle = None
        self.lock = threading.Lock()

        self._global_map_name = node.declare_parameter("global_map_name", "map_global").value
        self.visualizer.global_map_name = self._global_map_name

        self._clients: dict[str, ActionClient] = {}

        self._main_callback_group = MutuallyExclusiveCallbackGroup()

        for ns in PhoneNavigation.NS:
            for action in PhoneNavigation.ACTIONS:
                name = "/".join([ns, action])
                # use action for _clients key instead of full name
                self._clients[f"/{action}"] = ActionClient(act_node, PhoneNavigation.ACTIONS[action], name, callback_group=self._main_callback_group)

        self._spin_client = ActionClient(act_node, nav2_msgs.action.Spin, "/phone/spin", callback_group=self._main_callback_group)
        self._user_action_client = ActionClient(act_node, nav2_msgs.action.DummyBehavior, "/phone/user_action", callback_group=self._main_callback_group)

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        path_output = node.declare_parameter("path_topic", "/path").value
        self.path_pub = node.create_publisher(nav_msgs.msg.Path, path_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.goal_id_pub = node.create_publisher(unique_identifier_msgs.msg.UUID, "/debug/goal_id", 10, callback_group=MutuallyExclusiveCallbackGroup())

        self._last_estimated_goal = None
        self._last_estimated_goal_check = None
        self._retry_count = 0

        self.interface = PhoneInterface(node)
        self._process_queue = ProcessQueue(node)
        self._start_loop()

    def process_event(self, event):
        self._logger.info(f"process_event {str(event)}")
        if event.subtype == "phone":
            self.set_destination(event.param)
            return True
        if event.subtype == "phone_cancel":
            self.interface.speak(
                i18n.localized_string("PHONE_CANCEL_NAVIGATION"),
                force=True,
                priority=SpeechPriority.REQUIRED,
            )
            self.cancel_navigation()
            return True
        if event.subtype == "phone_completed":
            def callback():
                self._logger.info("cancel_navigation callback called")
                self._send_user_action("arrived")
                self._sub_goals = []
                self._navigate_next_sub_goal()

            self.cancel_navigation(callback=callback)
            return True

    def _start_loop(self):
        self._logger.info(F"phone.{util.callee_name()} called")
        if self.lock.acquire():
            if self._loop_handle is None:
                self._loop_handle = self._node.create_timer(0.1, self._check_loop, callback_group=self._main_callback_group)
            self.lock.release()

    def current_phone_global_pose(self):
        local = self.current_phone_pose()
        _global = geoutil.local2global(local, self._anchor)
        self._logger.debug(F"current global pose ({_global})", throttle_duration_sec=1.0)
        return _global

    def current_phone_location_id(self):
        """get id string for the current loaction in ROS"""

        _global = self.current_phone_global_pose()
        return F"latlng:{_global.lat:.7f}:{_global.lng:.7f}:{self.current_floor}"

    def current_phone_pose(self, frame=None) -> geoutil.Pose:
        """get current local location"""
        if frame is None:
            frame = self._global_map_name

        try:
            transformStamped = self.buffer.lookup_transform(
                frame, 'phone/base_link', CaBotRclpyUtil.time_zero())
            translation = transformStamped.transform.translation
            rotation = transformStamped.transform.rotation
            euler = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            current_pose = geoutil.Pose(x=translation.x, y=translation.y, r=euler[2])
            return current_pose
        except RuntimeError:
            self._logger.debug("cannot get current_local_pose")
        except:  # noqa: E722
            self._logger.error(traceback.format_exc())
        raise RuntimeError("no transformation")

    def _check_loop(self):
        self._logger.info("_check_loop", throttle_duration_sec=1.0)
        if not rclpy.ok():
            return

        # need a robot position
        try:
            self.current_pose = self.current_phone_pose()
            self._logger.debug(F"current pose {self.current_pose}", throttle_duration_sec=1)
        except RuntimeError:
            self._logger.info("could not get position", throttle_duration_sec=3)
            return
        except:  # noqa: E722
            self._logger.debug(traceback.format_exc())
            return

        try:
            self._check_goal(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)

    def _check_goal(self, current_pose):
        self._logger.info(F"phone.{util.callee_name()} called", throttle_duration_sec=1)
        goal = self._current_goal
        if not goal:
            return

        goal.check(current_pose)

        # estimate next goal
        now = self._node.get_clock().now()
        interval = rclpy.duration.Duration(seconds=1.0)
        if not self._last_estimated_goal_check or now - self._last_estimated_goal_check > interval:
            estimated_goal = navgoal.estimate_next_goal(self._sub_goals, current_pose, self.current_floor)
            if self._last_estimated_goal != estimated_goal:
                self._logger.info(F"Estimated next goal = {estimated_goal}")
                self.delegate.activity_log("cabot/phone", "estimated_next_goal", F"{repr(estimated_goal)}")
            self._last_estimated_goal = estimated_goal
            self._last_estimated_goal_check = now

        if goal.is_canceled:
            self._current_goal = None
            self._goal_index = max(-1, self._goal_index - 1)
            # unexpected cancel, may need to retry
            if self._status_manager.state == State.in_action:
                self._logger.info("NavigationState: canceled (system)")
                self._status_manager.set_state(State.in_pausing)
                self._retry_count += 1
                self._logger.info("NavigationState: retrying (system)")
                self._status_manager.set_state(State.in_action)
                self.retry_navigation()
                self._logger.info("NavigationState: retried (system)")
                return
            self._logger.info("NavigationState: canceled (user)")
            self.delegate.activity_log("cabot/phone", "goal_canceled", F"{goal.__class__.__name__}")
            return

        if not goal.is_completed:
            return

        if goal.is_exiting_goal:
            return

        def goal_exit_callback():
            self.delegate.activity_log("cabot/phone", "goal_completed", F"{goal.__class__.__name__}")
            self._current_goal = None

            self._navigate_next_sub_goal()
        goal.exit(goal_exit_callback)

    # wrap execution by a queue
    def set_destination(self, destination):
        leaving = False
        if destination and destination.endswith(":leaving"):
            leaving = True
            destination = destination[:-len(":leaving")]
        self.destination = destination
        self._status_manager.set_state(State.in_action)
        self._process_queue.add(self._set_destination, destination, leaving)

    def reset_destination(self):
        if self.destination:
            self.set_destination(self.destination)

    def _route_point_from_node(self, node):
        if node is None:
            return None
        if node.local_geometry is not None:
            return node.local_geometry
        if self._anchor is not None and node.geometry is not None:
            try:
                return geoutil.global2local(node.geometry, self._anchor)
            except:  # noqa: E722
                self._logger.debug("failed to convert node geometry to local")
        return None

    def _append_route_point(self, points, point):
        if point is None:
            return
        if points and points[-1].distance_to(point) < self.ROUTE_OVERVIEW_POINT_EPS:
            return
        points.append(point)

    def _extract_route_points(self, groute):
        points = []
        for item in groute:
            if isinstance(item, geojson.RouteLink):
                if not points:
                    self._append_route_point(points, self._route_point_from_node(item.source_node))
                self._append_route_point(points, self._route_point_from_node(item.target_node))
            elif isinstance(item, geojson.Node):
                self._append_route_point(points, self._route_point_from_node(item))
        return points

    def _segment_length(self, points, start_idx, end_idx):
        length = 0.0
        for i in range(start_idx, end_idx):
            length += points[i].distance_to(points[i + 1])
        return length

    def _segment_angle(self, start_point, end_point):
        return math.atan2(end_point.y - start_point.y, end_point.x - start_point.x)

    def _split_route_points(self, points):
        segments = []
        start_idx = 0
        last_index = len(points) - 1
        while start_idx < last_index:
            last_valid_end = start_idx + 1
            candidate_end = last_valid_end
            while candidate_end <= last_index:
                if points[start_idx].distance_to(points[candidate_end]) < self.ROUTE_OVERVIEW_POINT_EPS:
                    candidate_end += 1
                    continue
                line = geoutil.Line(start=points[start_idx], end=points[candidate_end])
                max_dist = 0.0
                for i in range(start_idx + 1, candidate_end):
                    dist = line.distance_to(points[i])
                    if dist > max_dist:
                        max_dist = dist
                        if max_dist > self.ROUTE_OVERVIEW_LINE_TOLERANCE:
                            break
                if max_dist <= self.ROUTE_OVERVIEW_LINE_TOLERANCE:
                    last_valid_end = candidate_end
                    candidate_end += 1
                else:
                    break
            segments.append((start_idx, last_valid_end))
            start_idx = last_valid_end
        return segments

    def _merge_small_turns(self, segments):
        if not segments:
            return []
        merged = [segments[0]]
        for seg in segments[1:]:
            prev = merged[-1]
            angle_diff = geoutil.normalize_angle(seg["angle"] - prev["angle"])
            if abs(math.degrees(angle_diff)) < self.ROUTE_OVERVIEW_TURN_THRESHOLD_DEG:
                prev["end_idx"] = seg["end_idx"]
                prev["end"] = seg["end"]
                prev["length"] += seg["length"]
                prev["angle"] = self._segment_angle(prev["start"], prev["end"])
            else:
                merged.append(seg)
        return merged

    def _format_distance(self, distance_m):
        rounded = int(round(distance_m))
        if rounded == 0 and distance_m > 0:
            return 1
        return rounded

    def _format_angle(self, angle_rad):
        angle_deg = abs(math.degrees(angle_rad))
        rounded = int(round(angle_deg / self.ROUTE_OVERVIEW_ANGLE_ROUND_DEG) * self.ROUTE_OVERVIEW_ANGLE_ROUND_DEG)
        if rounded == 0:
            rounded = self.ROUTE_OVERVIEW_ANGLE_ROUND_DEG
        return rounded

    def _build_route_overview(self, groute):
        if not groute:
            return None
        points = self._extract_route_points(groute)
        if len(points) < 2:
            return None

        segments = []
        for start_idx, end_idx in self._split_route_points(points):
            segment = {
                "start_idx": start_idx,
                "end_idx": end_idx,
                "start": points[start_idx],
                "end": points[end_idx],
                "length": self._segment_length(points, start_idx, end_idx),
                "angle": self._segment_angle(points[start_idx], points[end_idx]),
            }
            segments.append(segment)

        segments = self._merge_small_turns(segments)
        if not segments:
            return None
        total_length = sum(seg["length"] for seg in segments)
        if total_length < self.ROUTE_OVERVIEW_MIN_DISTANCE:
            return None

        steps = []
        steps.append(i18n.localized_string(
            "PHONE_ROUTE_OVERVIEW_STRAIGHT",
            self._format_distance(segments[0]["length"])
        ))
        for i in range(1, len(segments)):
            prev = segments[i - 1]
            seg = segments[i]
            angle_diff = geoutil.normalize_angle(seg["angle"] - prev["angle"])
            direction_key = "LEFT" if angle_diff > 0 else "RIGHT"
            steps.append(i18n.localized_string(
                "PHONE_ROUTE_OVERVIEW_TURN_AND_STRAIGHT",
                i18n.localized_string(direction_key),
                self._format_angle(angle_diff),
                self._format_distance(seg["length"])
            ))
        joiner = i18n.localized_string("PHONE_ROUTE_OVERVIEW_JOINER")
        return i18n.localized_string("PHONE_ROUTE_OVERVIEW_PREFIX", joiner.join(steps))

    def _set_destination(self, destination, leaving=False):
        """
        memo: current logic is not beautiful.
        1. get a NavCog route from the server
        2. get the last point of the route
        3. set goal pose via actionlib
        ## issues
          - cannot use NavCog topology
          - NavCog topology needs to be fixed
        """
        self._logger.info(F"navigation.{util.callee_name()} called")
        try:
            from_id = self.current_phone_location_id()
        except RuntimeError:
            self._logger.error("could not get current phone location")
            self.delegate.could_not_get_current_location()
            self._status_manager.set_state(State.idle)
            return

        self.delegate.activity_log("cabot/phone", "from", from_id)
        self.delegate.activity_log("cabot/phone", "to", destination)

        # specify last orientation
        if destination.find("@") > -1:
            (to_id, yaw_str) = destination.rsplit("@", 1)
            yaw = float(yaw_str)
            groute = self._datautil.get_route(from_id, to_id)
            self._sub_goals = navgoal.make_goals(self, groute, self._anchor, yaw=yaw, separate_route=False)
        else:
            to_id = destination
            groute = self._datautil.get_route(from_id, to_id)
            self._sub_goals = navgoal.make_goals(self, groute, self._anchor, separate_route=False)

        self._logger.info(F"{groute=}")
        route_overview = self._build_route_overview(groute)
        self._logger.info(F"route overview: {route_overview}")
        self._goal_index = -1
        self.to_id = to_id

        def check_facilities_task():
            # check facilities
            self.nearby_facilities = []
            links = list(filter(lambda x: isinstance(x, geojson.RouteLink), groute))
            target = groute[-1]
            if len(links) > 0:
                kdtree = geojson.LinkKDTree()
                kdtree.build(links)

                start = time.time()
                facilities = geojson.Object.get_objects_by_exact_type(geojson.Facility)
                for facility in facilities:
                    # self._logger.debug(f"facility {facility._id}: {facility.name}")
                    if not facility.name:
                        continue
                    if not facility.is_read:
                        continue

                    for ent in facility.entrances:
                        min_link, min_dist = kdtree.get_nearest_link(ent.node)
                        if min_link is None or min_dist >= 5.0:
                            continue
                        # self._logger.debug(f"Facility - Link {facility._id}, {min_dist}, {facility.name}:{ent.name}, {min_link._id}")
                        gp = ent.set_target(min_link)
                        dist = target.geometry.distance_to(gp)
                        if dist < 1.0:
                            self._logger.info(f"{facility._id=}: {facility.name=} is close to the node {target._id=}, {dist=}")
                            continue
                        self.nearby_facilities.append({
                            "facility": facility,
                            "entrance": ent
                        })
                end = time.time()
                self._logger.info(F"Check Facilities {end - start:.3f}.sec")
        threading.Thread(target=check_facilities_task).start()

        def after_speak(_result):
            self._logger.info("start navigation after speak")
            self._navigate_next_sub_goal()

        self.interface.start_navigation(to_id, route_overview=route_overview, leaving=leaving, callback=after_speak)

    # wrap execution by a queue
    def retry_navigation(self):
        self._process_queue.add(self._retry_navigation)

    def _retry_navigation(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/phone", "retry")
        self.turns = []

        self._logger.info(f"{self._current_goal=}, {self._goal_index}")
        self._navigate_next_sub_goal()

    # wrap execution by a queue
    def pause_navigation(self, callback):
        self._status_manager.set_state(State.in_pausing)

        def done_callback():
            self._status_manager.set_state(State.in_pause)
            if callback:
                callback()

        self._process_queue.add(self._pause_navigation, done_callback)

    def _pause_navigation(self, callback):
        self._logger.info(F"phone.{util.callee_name()} called")
        self.delegate.activity_log("cabot/phone", "pause")

        if self._current_goal:
            self._current_goal.cancel(callback)
        else:
            callback()

        self.turns = []
        self.notified_turns = {"directional_indicator": [], "vibrator": []}

        if self._current_goal:
            self._goal_index -= 1

    # wrap execution by a queue
    def resume_navigation(self, callback=None):
        self._status_manager.set_state(State.in_action)
        self._process_queue.add(self._resume_navigation, callback)

    def _resume_navigation(self, callback):
        self._logger.info(F"phone.{util.callee_name()} called")
        self.delegate.activity_log("cabot/phone", "resume")

        current_pose = self.current_local_pose()
        goal, index = navgoal.estimate_next_goal(self._sub_goals, current_pose, self.current_floor)
        self._logger.info(F"phone.{util.callee_name()} estimated next goal index={index}: {goal}")
        if goal:
            goal.estimate_inner_goal(current_pose, self.current_floor)
            self._goal_index = index - 1
            self._last_estimated_goal = None
            self._navigate_next_sub_goal()
        else:
            self.reset_destination()

    # wrap execution by a queue
    def cancel_navigation(self, callback=None):
        if self._status_manager.state != State.idle:
            self._status_manager.set_state(State.in_pausing)

        def done_callback():
            self._status_manager.set_state(State.idle)
            if callback:
                callback()

        self._process_queue.add(self._cancel_navigation, done_callback)

    def _cancel_navigation(self, callback):
        """callback for cancel topic"""
        self._logger.info(F"phone.{util.callee_name()} called")
        self.delegate.activity_log("cabot/phone", "cancel")
        self._sub_goals = None
        self._goal_index = -1
        if self._current_goal:
            self._current_goal.cancel(callback)
            self._current_goal = None
        else:
            callback()

    # private methods for navigation
    def _navigate_next_sub_goal(self):
        self._logger.info(F"phone.{util.callee_name()} called")
        if self._sub_goals is None:
            self._logger.info("phone is canceled")
            return

        if self._sub_goals and self._goal_index+1 < len(self._sub_goals):
            self.delegate.activity_log("cabot/phone", "next_sub_goal")
            self._goal_index += 1
            self._current_goal = self._sub_goals[self._goal_index]
            self._current_goal.reset()
            self._navigate_sub_goal(self._current_goal)
            return

        self._current_goal = None
        self._status_manager.set_state(State.idle)

        # keep this for test
        # do nothing actually, supposed to be handled in cabot-ios-app
        self.delegate.activity_log("cabot/phone", "navigation", "arrived")
        self.interface.have_completed(self.to_id)
        self.delegate.have_completed()
        # notify external nodes about arrival
        self.delegate.activity_log("cabot/phone", "completed")
        self._send_user_action("completed")

    def _navigate_sub_goal(self, goal):
        self._logger.info(F"phone.{util.callee_name()} called")
        self.delegate.activity_log("cabot/phone", "sub_goal")
        self.visualizer.reset()
        if isinstance(goal, navgoal.NavGoal):
            self.visualizer.pois = goal.pois
            self.visualizer.visualize()
            self.speed_pois = [x for x in goal.pois if isinstance(x, geojson.SpeedPOI)]
            self.info_pois = [x for x in goal.pois if not isinstance(x, geojson.SpeedPOI)]
            self.queue_wait_pois = [x for x in goal.pois if isinstance(x, geojson.QueueWaitPOI)]
            self.gradient = goal.gradient
        else:
            self.visualizer.pois = []
            self.speed_poi = []
            self.info_pois = []
            self.queue_wait_pois = []
            self.gradient = []
        self.turns = []
        self.notified_turns = {"directional_indicator": [], "vibrator": []}

        self._logger.info(F"goal: {goal}")
        try:
            goal.enter()
        except:  # noqa: E722
            import traceback
            self._logger.error(traceback.format_exc())
        self._start_loop()

    def _send_user_action(self, command):
        try:
            self._logger.info(F"send user_action {command}")
            if not self._user_action_client.wait_for_server(timeout_sec=1.0):
                self._logger.warning("user_action action server not available")
                return
            goal = nav2_msgs.action.DummyBehavior.Goal()
            goal.command = std_msgs.msg.String()
            goal.command.data = command
            self._user_action_client.send_goal_async(goal)
        except:
            self._logger.error(traceback.format_exc())

    # GoalInterface

    def activity_log(self, category="", text="", memo=""):
        self.delegate.activity_log(category, text, memo)

    def get_logger(self):
        return self._logger

    def enter_goal(self, goal):
        self.delegate.enter_goal(goal)

    def exit_goal(self, goal):
        self.delegate.exit_goal(goal)

    def navigate_to_pose(self, goal_pose, behavior_tree, gh_cb, done_cb, namespace=""):
        self._logger.info(F"{namespace}/navigate_to_pose")
        self.delegate.activity_log("cabot/phone", "navigate_to_pose")
        client = self._clients["/".join([namespace, "navigate_to_pose"])]
        goal = nav2_msgs.action.NavigateToPose.Goal()

        # do not set behavior tree to use default one
        # goal.behavior_tree = behavior_tree

        if namespace == "":
            goal.pose = self.buffer.transform(goal_pose, self._global_map_name)
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.header.frame_id = self._global_map_name
        elif namespace == "/local":
            goal.pose = goal_pose
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.header.frame_id = "local/odom"
        else:
            self._logger.info(F"unknown namespace {namespace}")
            return

        self._logger.info("visualize goal")
        self.visualizer.goal = goal
        self.visualizer.visualize()

        self._logger.info("sending goal")
        future = client.send_goal_async(goal)
        self._logger.info("add done callback")
        future.add_done_callback(lambda future: self._navigate_to_pose_sent_goal(goal, future, gh_cb, done_cb))
        self._logger.info("added done callback")

        def timeout_watcher(future, timeout_duration):
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_duration:
                    self._logger.error("Timeout occurred while waiting for sending goal future to be completed")
                    future.cancel()
                    return
                time.sleep(0.1)

        timeout_tread = threading.Thread(target=timeout_watcher, args=(future, 5))
        timeout_tread.start()
        return future

    def _navigate_to_pose_sent_goal(self, goal, future, gh_cb, done_cb):
        if future.cancelled():
            try:
                done_cb(None)
            except:  # noqa: E722
                self._logger.error(traceback.format_exc())
            return
        self._logger.info("_navigate_to_pose_sent_goal")
        self._logger.info(F"navigate to pose, threading.get_ident {threading.get_ident()}")
        goal_handle = future.result()
        self.goal_id_pub.publish(goal_handle.goal_id)
        gh_cb(goal_handle)
        self._logger.info(F"get goal handle {goal_handle}")
        get_result_future = goal_handle.get_result_async()
        self._logger.info("add done callback")
        get_result_future.add_done_callback(done_cb)

        self._logger.info("visualize goal")
        self.visualizer.goal = goal
        self.visualizer.visualize()

    def publish_path(self, global_path, convert=True):
        local_path = global_path
        if convert:
            local_path = nav_msgs.msg.Path()
            local_path.header = global_path.header
            local_path.header.frame_id = "map"

            for pose in global_path.poses:
                local_path.poses.append(self.buffer.transform(pose, "map"))
                local_path.poses[-1].pose.position.z = 0.0

        self.path_pub.publish(local_path)

    def global_map_name(self):
        return self._global_map_name

    def change_parameters(self, params, callback):
        self.param_manager.change_parameters(params, callback)

    def turn_towards(self, orientation, gh_callback, callback, clockwise=0, time_limit=15.0):
        self._logger.info("turn_towards")
        self.delegate.activity_log("cabot/navigation", "turn_towards",
                                   str(geoutil.get_yaw(geoutil.q_from_msg(orientation))))
        self.turn_towards_count = 0
        self.turn_towards_last_diff = None
        self._turn_towards(orientation, gh_callback, callback, clockwise, time_limit)

    def _turn_towards(self, orientation, gh_callback, callback, clockwise=0, time_limit=30.0):
        goal = nav2_msgs.action.Spin.Goal()
        diff = geoutil.diff_angle(self.current_pose.orientation, orientation)
        time_allowance = max(30.0, min(time_limit, abs(diff) / 0.05))
        goal.time_allowance = rclpy.duration.Duration(seconds=time_allowance).to_msg()

        self._logger.info(F"current pose {self.current_pose}, diff {diff:.2f}")
        if (clockwise < 0 and diff < - math.pi / 4) or \
           (clockwise > 0 and diff > + math.pi / 4):
            diff = diff - clockwise * math.pi * 2
        turn_yaw = diff - (diff / abs(diff) * 0.05)
        goal.target_yaw = turn_yaw

        msg = std_msgs.msg.Float32()
        msg.data = geoutil.normalize_angle(turn_yaw)

        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(lambda future: self._turn_towards_sent_goal(goal, future, orientation, gh_callback, callback, clockwise, turn_yaw, time_limit))
        return future

    def _turn_towards_sent_goal(self, goal, future, orientation, gh_callback, callback, clockwise, turn_yaw, time_limit):
        self._logger.info(F"_turn_towards_sent_goal: {goal=} {goal.time_allowance=} {turn_yaw=}")
        goal_handle = future.result()

        def cancel_callback():
            self._logger.info(F"_turn_towards_sent_goal cancel_callback: {goal=}")
            self.turn_towards_count = 0
            self.turn_towards_last_diff = None
        gh_callback(goal_handle, cancel_callback)
        self._logger.info(F"get goal handle {goal_handle}")
        get_result_future = goal_handle.get_result_async()

        def done_callback(future):
            self._logger.info(F"_turn_towards_sent_goal done_callback: {future.result().status=}, {self.turn_towards_last_diff=}")
            if future.result().status == GoalStatus.STATUS_CANCELED:
                return
            diff = geoutil.diff_angle(self.current_pose.orientation, orientation)
            if self.turn_towards_last_diff and abs(self.turn_towards_last_diff - diff) < 0.1:
                self.turn_towards_count += 1

            if abs(diff) > 0.05 and self.turn_towards_count < 10:
                self._logger.info(F"send turn {diff:.2f}")
                self.turn_towards_last_diff = diff
                self._turn_towards(orientation, gh_callback, callback, clockwise, time_limit)
            else:
                self._logger.info(F"turn completed {diff=}, {self.turn_towards_count=}")
                callback(True)
        get_result_future.add_done_callback(done_callback)
