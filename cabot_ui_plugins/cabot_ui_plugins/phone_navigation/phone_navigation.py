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

import threading
import time
import traceback

# ROS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import unique_identifier_msgs.msg
import nav2_msgs.action
import nav_msgs.msg
from ament_index_python.packages import get_package_share_directory

# Other
from cabot_ui import geojson
from cabot_common import util
from cabot_ui.event import NavigationEvent
import cabot_ui.navgoal as navgoal
from cabot_ui.navgoal import GoalInterface
from cabot_ui.param_manager import ParamManager
from cabot_ui_plugins.navigation import ControlBase
from cabot_ui.plugin import NavigationPlugin
from cabot_ui.node_manager import NodeManager
from cabot_ui.process_queue import ProcessQueue


class PhoneNavigation(ControlBase, GoalInterface, NavigationPlugin):
    """Phone Navigation"""

    ACTIONS = {"navigate_to_pose": nav2_msgs.action.NavigateToPose}

    NS = ["/phone"]
    TURN_NEARBY_THRESHOLD = 2

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

        super(PhoneNavigation, self).__init__(
            node, tf_node, datautil_instance=datautil_instance, anchor_file=anchor_file)

        self.param_manager = ParamManager(srv_node)

        self.info_pois = []
        self._sub_goals = None
        self._goal_index = -1
        self._current_goal = None

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

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        path_output = node.declare_parameter("path_topic", "/path").value
        self.path_pub = node.create_publisher(nav_msgs.msg.Path, path_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.goal_id_pub = node.create_publisher(unique_identifier_msgs.msg.UUID, "/debug/goal_id", 10, callback_group=MutuallyExclusiveCallbackGroup())

        self._process_queue = ProcessQueue(node)
        self._start_loop()

    def process_event(self, event) -> None:
        self._logger.info(f"process_event {str(event)}")
        if event.subtype == "phone":
            self.set_destination(event.param)

    def _start_loop(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        if self.lock.acquire():
            if self._loop_handle is None:
                self._loop_handle = self._node.create_timer(0.1, self._check_loop, callback_group=self._main_callback_group)
            self.lock.release()

    def _check_loop(self):
        self._logger.info("_check_loop", throttle_duration_sec=1.0)
        if not rclpy.ok():
            return

        # need a robot position
        try:
            self.current_pose = self.current_local_pose()
            self._logger.debug(F"current pose {self.current_pose}", throttle_duration_sec=1)
        except RuntimeError:
            self._logger.info("could not get position", throttle_duration_sec=3)
            return
        except:  # noqa: E722
            self._logger.debug(traceback.format_exc())
            return

    # wrap execution by a queue
    def set_destination(self, destination):
        self.destination = destination
        self._process_queue.add(self._set_destination, destination)

    def reset_destination(self):
        if self.destination:
            self.set_destination(self.destination)

    def _set_destination(self, destination):
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
            from_id = self.current_location_id()
        except RuntimeError:
            self._logger.error("could not get current location")
            self.delegate.could_not_get_current_location()
            return

        self.delegate.activity_log("cabot/navigation", "from", from_id)
        self.delegate.activity_log("cabot/navigation", "to", destination)

        # specify last orientation
        if destination.find("@") > -1:
            (to_id, yaw_str) = destination.split("@")
            yaw = float(yaw_str)
            groute = self._datautil.get_route(from_id, to_id)
            self._sub_goals = navgoal.make_goals(self, groute, self._anchor, yaw=yaw)
        else:
            to_id = destination
            groute = self._datautil.get_route(from_id, to_id)
            self._sub_goals = navgoal.make_goals(self, groute, self._anchor)
        self._goal_index = -1

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

        # navigate from the first path
        self._navigate_next_sub_goal()

    # wrap execution by a queue
    def retry_navigation(self):
        self._process_queue.add(self._retry_navigation)

    def _retry_navigation(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "retry")
        self.turns = []

        self._logger.info(f"{self._current_goal=}, {self._goal_index}")
        self._navigate_next_sub_goal()

    # wrap execution by a queue
    def pause_navigation(self, callback):
        self._process_queue.add(self._pause_navigation, callback)

    def _pause_navigation(self, callback):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "pause")

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
        self._process_queue.add(self._resume_navigation, callback)

    def _resume_navigation(self, callback):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "resume")

        current_pose = self.current_local_pose()
        goal, index = navgoal.estimate_next_goal(self._sub_goals, current_pose, self.current_floor)
        self._logger.info(F"navigation.{util.callee_name()} estimated next goal index={index}: {goal}")
        if goal:
            goal.estimate_inner_goal(current_pose, self.current_floor)
            self._goal_index = index - 1
            self._last_estimated_goal = None
            self._navigate_next_sub_goal()
        else:
            self.reset_destination()

    # wrap execution by a queue
    def cancel_navigation(self, callback=None):
        self._process_queue.add(self._cancel_navigation, callback)

    def _cancel_navigation(self, callback):
        """callback for cancel topic"""
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "cancel")
        self._sub_goals = None
        self._goal_index = -1
        self._stop_loop()
        if self._current_goal:
            self._current_goal.cancel(callback)
            self._current_goal = None
        else:
            callback()

    # private methods for navigation
    def _navigate_next_sub_goal(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        if self._sub_goals is None:
            self._logger.info("navigation is canceled")
            return

        if self._sub_goals and self._goal_index+1 < len(self._sub_goals):
            self.delegate.activity_log("cabot/navigation", "next_sub_goal")
            self._goal_index += 1
            self._current_goal = self._sub_goals[self._goal_index]
            self._current_goal.reset()
            self._navigate_sub_goal(self._current_goal)
            return

        self._current_goal = None
        self.delegate.have_completed()
        self.delegate.activity_log("cabot/navigation", "completed")

    def _navigate_sub_goal(self, goal):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "sub_goal")
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
        self.delegate.activity_log("cabot/navigation", "navigate_to_pose")
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
