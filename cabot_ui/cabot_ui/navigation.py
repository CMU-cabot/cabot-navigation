# Copyright (c) 2020  Carnegie Mellon University
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

import copy
import math
import inspect
import numpy
import numpy.linalg
import os
import threading
import time
import traceback

# ROS
import rclpy
from rcl_interfaces.srv import SetParameters, GetParameters
import rcl_interfaces.msg
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf2_ros
import tf2_geometry_msgs  # noqa: to register class for transform
import tf_transformations
import nav2_msgs.action
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import unique_identifier_msgs.msg
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus

# Other
from cabot_common import util
from cabot_ui import visualizer, geoutil, geojson, datautil
from cabot_ui.turn_detector import TurnDetector, Turn
from cabot_ui import navgoal
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.social_navigation import SocialNavigation, SNMessage
from cabot_msgs.srv import LookupTransform
import queue_msgs.msg
from mf_localization_msgs.msg import MFLocalizeStatus


class NavigationInterface(object):
    def activity_log(self, category="", text="", memo=""):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def i_am_ready(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def start_navigation(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def update_pose(self, **kwargs):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def notify_turn(self, device=None, turn=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def notify_human(self, angle=0):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def goal_canceled(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def have_arrived(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def have_completed(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def approaching_to_poi(self, poi=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def approached_to_poi(self, poi=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def passed_poi(self, poi=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def enter_goal(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def exit_goal(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def could_not_get_current_location(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_call_elevator(self, pos):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def elevator_opening(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def floor_changed(self, floor):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def queue_start_arrived(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def queue_proceed(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_pass_door(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def door_passed(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_follow_behind(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_return_position(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def announce_social(self, message: SNMessage):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def request_sound(self, message: SNMessage):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def system_pause_navigation(self, callback):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")


class BufferProxy():
    def __init__(self, node):
        self._clock = node.get_clock()
        self._logger = node.get_logger()
        self.lookup_transform_service = node.create_client(LookupTransform, 'lookup_transform', callback_group=MutuallyExclusiveCallbackGroup())
        self.countPub = node.create_publisher(std_msgs.msg.Int32, "transform_count", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.transformMap = {}
        self.min_interval = rclpy.duration.Duration(seconds=0.2)

    def debug(self):
        if not hasattr(self, "count"):
            self.count = 0
        self.count += 1
        msg = std_msgs.msg.Int32()
        msg.data = self.count
        self.countPub.publish(msg)

    # buffer interface
    def lookup_transform(self, target, source, time=None):
        # find the latest saved transform first
        key = f"{target}-{source}"
        now = self._clock.now()
        if key in self.transformMap:
            (transform, last_time) = self.transformMap[key]
            if now - last_time < self.min_interval:
                self._logger.debug(f"found old lookup_transform({target}, {source}, {(now - last_time).nanoseconds/1000000000:.2f}sec)")
                return transform

        if __debug__:
            self.debug()
        self._logger.debug(f"lookup_transform({target}, {source})")
        req = LookupTransform.Request()
        req.target_frame = target
        req.source_frame = source
        if not self.lookup_transform_service.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("lookup transform service is not available")
        # usage of call() can cause SIGSEGV at exit due to infinite wait
        # however, call(req, timeout_sec=1.0) is implemented in J-turtle release not in humble
        #
        # result = self.lookup_transform_service.call(req)
        #
        # so implemented call with timeout from here
        # reference implementation is
        # https://github.com/ros2/rclpy/blob/7a7f23e0d7e51dd244001ef606b97f1153e5a97e/rclpy/rclpy/client.py#L72-L110
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()
        future = self.lookup_transform_service.call_async(req)
        future.add_done_callback(unblock)
        # Check future.done() before waiting on the event.
        # The callback might have been added after the future is completed,
        # resulting in the event never being set.
        if not future.done():
            if not event.wait(1.0):
                # Timed out. remove_pending_request() to free resources
                self.lookup_transform_service.remove_pending_request(future)
                raise RuntimeError("timeout")
        if future.exception() is not None:
            raise future.exception()
        result = future.result()
        # sync call end here

        if result.error.error > 0:
            raise RuntimeError(result.error.error_string)
        self.transformMap[key] = (result.transform, now)
        return result.transform

    def transform(self, pose_stamped, target):
        do_transform = tf2_ros.TransformRegistration().get(type(pose_stamped))
        transform = self.lookup_transform(target, pose_stamped.header.frame_id)
        return do_transform(pose_stamped, transform)


class ControlBase(object):
    # _anchor = geoutil.Anchor(lat=40.443228, lng=-79.945705, rotate=15) # NSH NavCog anchor
    # _anchor = geoutil.Anchor(lat=40.443262, lng=-79.945888, rotate=15.1) # 4fr
    # _anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=15.1) # 4fr-gazebo
    # _anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=-164.9) # 4fr-gazebo
    # _anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=16) # 4fr-gazebo

    def __init__(self, node: Node, tf_node: Node, datautil_instance=None, anchor_file=''):
        self._node = node
        self._logger = node.get_logger()
        self.visualizer = visualizer.instance(node)

        self.delegate: NavigationInterface = NavigationInterface()
        self.buffer = BufferProxy(tf_node)

        # TF listening is at high frequency and increases the CPU usage substantially
        # so the code is using a service to get TF transform
        # self.buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.buffer, tf_node)

        self.post_location_enabled = os.environ.get("CABOT_POST_LOCATION", "false").lower() == "true"
        self.current_pose = None
        self.last_log_time = node.get_clock().now()
        self.current_odom_pose = None
        self.current_floor = node.declare_parameter("initial_floor", 1).value
        self.floor_is_changed_at = node.get_clock().now()
        self._logger.info(F"current_floor is {self.current_floor}")

        # for current location
        self._anchor = None
        anchor_file = node.declare_parameter("anchor_file", anchor_file).value
        self._logger.info(F"Anchor file is {anchor_file}")
        if anchor_file is not None:
            temp = geoutil.get_anchor(anchor_file)
            if temp is not None:
                self._anchor = temp
            else:
                self._logger.warn(F"could not load anchor_file \"{anchor_file}\"")

        self._logger.info("set anchor and analyze")
        self._logger.info(F"anchor={str(self._anchor)}")
        # for data
        if datautil_instance is not None:
            self._datautil = datautil_instance
            self._datautil.set_anchor(self._anchor)
        else:
            self._datautil = datautil.getInstance(node)
            self._datautil.set_anchor(self._anchor)
            self._datautil.init_by_server()
            self._datautil.set_anchor(self._anchor)

    # current location
    def current_ros_pose(self, frame=None):
        """get current local location"""
        if frame is None:
            frame = self._global_map_name

        try:
            transformStamped = self.buffer.lookup_transform(
                frame, 'base_footprint', CaBotRclpyUtil.time_zero())
            ros_pose = geometry_msgs.msg.Pose()
            ros_pose.position.x = transformStamped.transform.translation.x
            ros_pose.position.y = transformStamped.transform.translation.y
            ros_pose.position.z = transformStamped.transform.translation.z
            ros_pose.orientation.x = transformStamped.transform.rotation.x
            ros_pose.orientation.y = transformStamped.transform.rotation.y
            ros_pose.orientation.z = transformStamped.transform.rotation.z
            ros_pose.orientation.w = transformStamped.transform.rotation.w
            return ros_pose
        except RuntimeError:
            self._logger.debug("cannot get current_ros_pose")
        except:  # noqa: E722
            self._logger.debug(traceback.format_exc())
        raise RuntimeError("no transformation")

    def current_local_pose(self, frame=None) -> geoutil.Pose:
        """get current local location"""
        if frame is None:
            frame = self._global_map_name

        try:
            transformStamped = self.buffer.lookup_transform(
                frame, 'base_footprint', CaBotRclpyUtil.time_zero())
            translation = transformStamped.transform.translation
            rotation = transformStamped.transform.rotation
            euler = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            current_pose = geoutil.Pose(x=translation.x, y=translation.y, r=euler[2])
            return current_pose
        except RuntimeError:
            self._logger.debug("cannot get current_local_pose")
        except:  # noqa: E722
            self._logger.debug(traceback.format_exc())
        raise RuntimeError("no transformation")

    def current_local_odom_pose(self):
        """get current local odom location"""
        try:
            transformStamped = self.buffer.lookup_transform(
                'local/odom', 'local/base_footprint', CaBotRclpyUtil.time_zero())
            translation = transformStamped.transform.translation
            rotation = transformStamped.transform.rotation
            euler = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            current_pose = geoutil.Pose(x=translation.x, y=translation.y, r=euler[2])
            return current_pose
        except RuntimeError:
            self._logger.debug("cannot get current_local_odom_pose")
        except:  # noqa: E722
            self._logger.debug(traceback.format_exc())
        raise RuntimeError("no transformation")

    def current_global_pose(self):
        local = self.current_local_pose()
        _global = geoutil.local2global(local, self._anchor)
        self._logger.debug(F"current global pose ({_global})", throttle_duration_sec=1.0)
        return _global

    def current_location_id(self):
        """get id string for the current loaction in ROS"""

        _global = self.current_global_pose()
        return F"latlng:{_global.lat:.7f}:{_global.lng:.7f}:{self.current_floor}"


class Navigation(ControlBase, navgoal.GoalInterface):
    """Navigation node for Cabot"""

    ACTIONS = {"navigate_to_pose": nav2_msgs.action.NavigateToPose,
               "navigate_through_poses": nav2_msgs.action.NavigateThroughPoses}
    NS = ["", "/local"]
    TURN_NEARBY_THRESHOLD = 2

    def __init__(self, node: Node, tf_node: Node, srv_node: Node, act_node: Node, soc_node: Node,
                 datautil_instance=None, anchor_file='', wait_for_action=True):

        self.current_floor = None
        self.current_frame = None

        super(Navigation, self).__init__(
            node, tf_node, datautil_instance=datautil_instance, anchor_file=anchor_file)

        self.param_manager = NavigationParamManager(srv_node)

        self.info_pois = []
        self.queue_wait_pois = []
        self.speed_pois = []
        self.turns = []
        self.gradient = []
        self.notified_turns = {"directional_indicator": [], "vibrator": []}

        self.i_am_ready = False
        self._sub_goals = None
        self._goal_index = -1
        self._current_goal = None
        self._last_estimated_goal_check = None
        self._last_estimated_goal = None

        # speed = 0.50 m/sec
        self._notify_vib_threshold = 0.85
        self._notify_di_threshold = 0.25

        # self.client = None
        self._loop_handle = None
        self.pause_control_msg = std_msgs.msg.Bool()
        self.pause_control_msg.data = True
        self.pause_control_loop_handler = None
        self.lock = threading.Lock()

        self.restart_navigation_threthold_sec = node.declare_parameter("restart_navigation_threthold_sec", 5.0).value

        self._max_speed = node.declare_parameter("max_speed", 1.1).value
        self._max_acc = node.declare_parameter("max_acc", 0.3).value
        self._speed_poi_params = node.declare_parameter("speed_poi_params", [0.5, 0.5, 0.5]).value

        self._global_map_name = node.declare_parameter("global_map_name", "map_global").value
        self.visualizer.global_map_name = self._global_map_name

        self.social_navigation = SocialNavigation(soc_node, self.buffer)

        self._clients: dict[str, ActionClient] = {}

        self._main_callback_group = MutuallyExclusiveCallbackGroup()

        for ns in Navigation.NS:
            for action in Navigation.ACTIONS:
                name = "/".join([ns, action])
                self._clients[name] = ActionClient(act_node, Navigation.ACTIONS[action], name, callback_group=self._main_callback_group)

        self._spin_client = ActionClient(act_node, nav2_msgs.action.Spin, "/spin", callback_group=self._main_callback_group)

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        pause_control_output = node.declare_parameter("pause_control_topic", "/cabot/pause_control").value
        self.pause_control_pub = node.create_publisher(std_msgs.msg.Bool, pause_control_output, 10, callback_group=MutuallyExclusiveCallbackGroup())
        map_speed_output = node.declare_parameter("map_speed_topic", "/cabot/map_speed").value
        self.speed_limit_pub = node.create_publisher(std_msgs.msg.Float32, map_speed_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.gradient_pub = node.create_publisher(std_msgs.msg.Float32, "/cabot/gradient", 10, callback_group=MutuallyExclusiveCallbackGroup())

        current_floor_input = node.declare_parameter("current_floor_topic", "/current_floor").value
        self.current_floor_sub = node.create_subscription(std_msgs.msg.Int64, current_floor_input, self._current_floor_callback, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        current_frame_input = node.declare_parameter("current_frame_topic", "/current_frame").value
        self.current_frame_sub = node.create_subscription(std_msgs.msg.String, current_frame_input, self._current_frame_callback, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self._localize_status_sub = node.create_subscription(MFLocalizeStatus, "/localize_status", self._localize_status_callback, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.localize_status = MFLocalizeStatus.UNKNOWN

        plan_input = node.declare_parameter("plan_topic", "/move_base/NavfnROS/plan").value
        self.plan_sub = node.create_subscription(nav_msgs.msg.Path, plan_input, self._plan_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        path_output = node.declare_parameter("path_topic", "/path").value
        self.path_pub = node.create_publisher(nav_msgs.msg.Path, path_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        path_all_output = node.declare_parameter("path_all_topic", "/path_all").value
        self.path_all_pub = node.create_publisher(nav_msgs.msg.Path, path_all_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.goal_id_pub = node.create_publisher(unique_identifier_msgs.msg.UUID, "/debug/goal_id", 10, callback_group=MutuallyExclusiveCallbackGroup())

        self.updated_goal_sub = node.create_subscription(geometry_msgs.msg.PoseStamped, "/updated_goal", self._goal_updated_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())

        self.current_queue_msg = None
        self.need_queue_start_arrived_info = False
        self.need_queue_proceed_info = False
        queue_input = node.declare_parameter("queue_topic", "/queue").value
        self.queue_sub = node.create_subscription(queue_msgs.msg.Queue, queue_input, self._queue_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        queue_speed_output = node.declare_parameter("queue_speed_topic", "/cabot/queue_speed").value
        self.queue_speed_limit_pub = node.create_publisher(std_msgs.msg.Float32, queue_speed_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        self._queue_tail_ignore_path_dist = node.declare_parameter("queue_tail_ignore_path_dist", 0.8).value
        self._queue_wait_pass_tolerance = node.declare_parameter("queue_wait_pass_tolerance", 0.3).value
        self._queue_wait_arrive_tolerance = node.declare_parameter("queue_wait_arrive_tolerance", 0.2)
        self._queue_tail_dist_error_tolerance = node.declare_parameter("queue_tail_dist_error_tolerance", 0.5).value
        self._queue_wait_position_offset = node.declare_parameter("queue_wait_position_offset", 0.2).value
        self.initial_queue_interval = node.declare_parameter("initial_queue_interval", 1.0).value
        self.current_queue_interval = self.initial_queue_interval

        turn_end_output = node.declare_parameter("turn_end_topic", "/cabot/turn_end").value
        self.turn_end_pub = node.create_publisher(std_msgs.msg.Bool, turn_end_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        rotation_complete_output = node.declare_parameter("rotation_complete_topic", "/cabot/rotation_complete").value
        self.rotation_complete_pub = node.create_publisher(std_msgs.msg.Bool, rotation_complete_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())
        turn_angle_output = node.declare_parameter("turn_angle_topic", "/cabot/turn_angle").value
        self.turn_angle_pub = node.create_publisher(std_msgs.msg.Float32, turn_angle_output, transient_local_qos, callback_group=MutuallyExclusiveCallbackGroup())

        self._process_queue = []
        self._process_queue_lock = threading.Lock()
        self._process_timer = act_node.create_timer(0.01, self._process_queue_func, callback_group=MutuallyExclusiveCallbackGroup())
        self._start_loop()

    def _localize_status_callback(self, msg):
        self._logger.info(F"_localize_status_callback {msg}")
        self.localize_status = msg.status

    def process_event(self, event):
        '''cabot navigation event'''
        # do not provide social navigation messages while queue navigation
        if isinstance(self._current_goal, navgoal.QueueNavGoal):
            return

        self._logger.info(F"{event}")
        if event.param == "elevator_door_may_be_ready":
            self.delegate.elevator_opening()
        elif event.param == "navigation_start":
            self.delegate.start_navigation()
        elif self.social_navigation is not None:
            self.social_navigation.event = event

    # callback functions
    def _current_floor_callback(self, msg):
        prev = self.current_floor
        self.current_floor = msg.data
        if msg.data >= 0:
            self.current_floor = msg.data + 1
        if self.current_floor != prev:
            self.floor_is_changed_at = self._node.get_clock().now()
        self._logger.info(F"Current floor is {self.current_floor}")

    def _current_frame_callback(self, msg):
        if self.current_frame != msg.data:
            self.wait_for_restart_navigation()
        self.current_frame = msg.data
        self._logger.info(F"Current frame is {self.current_frame}")

    def wait_for_restart_navigation(self):
        now = self._node.get_clock().now()
        duration_in_sec = CaBotRclpyUtil.to_sec(now - self.floor_is_changed_at)
        self._logger.info(F"wait_for_restart_navigation {duration_in_sec:.2f}")
        if self._current_goal is None:
            return
        if duration_in_sec > self.restart_navigation_threthold_sec:
            self._stop_loop()

            def done_callback():
                self.resume_navigation()
            self.delegate.system_pause_navigation(done_callback)

    def _plan_callback(self, path):
        try:
            msg = nav_msgs.msg.Path()
            msg.header.frame_id = self._global_map_name
            for pose in path.poses:
                pose.header.frame_id = path.header.frame_id
                msg.poses.append(self.buffer.transform(pose, self._global_map_name))
                # msg.poses[-1].pose.position.z = 0.0
            path = msg
        except:  # noqa: E722
            self._logger.info(traceback.format_exc())
        try:
            start = time.time()
            self.turns = TurnDetector.detects(path, current_pose=self.current_pose)
            end = time.time()
            self.visualizer.turns = self.turns
            if self.social_navigation is not None:
                self.social_navigation.path = path

            self._logger.info(F"turns: {self.turns}, it takes {(end - start)*1000:.0f}ms")
            """
            for i in range(len(self.turns)-1, 0, -1):
            t1 = self.turns[i]
            if abs(t1.angle) < math.pi/8:
                self.turns.pop(i)
            """
            for i in range(len(self.turns)-2, 0, -1):
                t1 = self.turns[i]
                t2 = self.turns[i+1]
                if (t1.angle < 0 and 0 < t2.angle) or \
                   (t2.angle < 0 and 0 < t1.angle):
                    if 0 < abs(t1.angle) and abs(t1.angle) < math.pi/3 and \
                       0 < abs(t2.angle) and abs(t2.angle) < math.pi/3:
                        self.turns.pop(i+1)
        except:  # noqa: 722
            self._logger.error(traceback.format_exc())

        self.visualizer.visualize()

        def path_length(path):
            last = None
            d = 0
            for p in path.poses:
                curr = numpy.array([p.pose.position.x, p.pose.position.y])
                if last is None:
                    last = curr
                d += numpy.linalg.norm(last-curr)
                last = curr
            return d

        self._logger.info(F"path-length {path_length(path):.2f}")

    def _queue_callback(self, msg):
        self.current_queue_msg = msg
        names = [person.name for person in self.current_queue_msg.people]
        self._logger.info(F"Current people in queue {names}", throttle_duration_sec=1)

    def _get_queue_interval_callback(self, msg):
        self.current_queue_interval = msg.data
        if self.initial_queue_interval is None:
            self.initial_queue_interval = self.current_queue_interval
        self._logger.info(F"Current queue interval parameter is {self.current_queue_interval}",
                          throttle_duration_sec=3)

    def _goal_updated_callback(self, msg):
        if self._current_goal:
            self._current_goal.update_goal(msg)

    def request_defult_params(self):
        with self._process_queue_lock:
            self._process_queue.append((self._request_default_params, ))

    def _request_default_params(self):
        # dummy Goal instance
        goal = navgoal.Goal(self, x=0, y=0, r=0, angle=0, floor=0)

        def dummy_nav_params_key():
            return navgoal.Nav2Params.all_keys()
        goal.nav_params_keys = dummy_nav_params_key

        def done_callback():
            self._logger.info(f"done_callback f{goal._current_params}")
            navgoal.Goal.default_params = copy.deepcopy(goal._current_params)
        goal._save_params(done_callback)

    # public interfaces

    def set_handle_side(self, side):
        self._logger.info("set_handle_side is called")
        with self._process_queue_lock:
            self._process_queue.append((self._set_handle_side, side))

    def _set_handle_side(self, side):
        self._logger.info("_set_handle_side is called")

        def callback(result):
            self.delegate.activity_log("cabot/navigation", "set_handle_side", "side")
            self._logger.info(f"set_handle_side {side=}, {result=}")
        if side == "left":
            offset_sign = +1.0
        elif side == "right":
            offset_sign = -1.0
        else:
            self._logger.info(f"set_handle_side {side=} should be 'left' or 'right'")
            return
        self.change_parameters({
            "/footprint_publisher": {
                "offset_sign": offset_sign
            }
        }, callback)

    def set_touch_mode(self, mode):
        self._logger.info("set_touch_mode is called")
        with self._process_queue_lock:
            self._process_queue.append((self._set_touch_mode, mode))

    def _set_touch_mode(self, mode):
        self._logger.info("_set_touch_mode is called")

        def callback(result):
            self.delegate.activity_log("cabot/navigation", "set_touch_mode", "mode")
            self._logger.info(f"set_touch_mode {mode=}, {result=}")
        self.change_parameters({
            "/cabot/cabot_can": {
                "touch_mode": mode
            }
        }, callback)

    # wrap execution by a queue
    def set_destination(self, destination):
        self.destination = destination
        self.social_navigation.set_active(True)
        with self._process_queue_lock:
            self._process_queue.append((self._set_destination, destination))

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

        # for dashboad
        (gpath, _, _) = navgoal.create_ros_path(groute, self._anchor, self.global_map_name())
        msg = nav_msgs.msg.Path()
        msg.header = gpath.header
        msg.header.frame_id = self._global_map_name
        for pose in gpath.poses:
            msg.poses.append(self.buffer.transform(pose, self._global_map_name))
            msg.poses[-1].pose.position.z = 0.0
        self.path_all_pub.publish(msg)

        # navigate from the first path
        self._navigate_next_sub_goal()

    # wrap execution by a queue
    def retry_navigation(self):
        with self._process_queue_lock:
            self._process_queue.append((self._retry_navigation,))

    def _retry_navigation(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "retry")
        self.turns = []

        self._logger.info(f"{self._current_goal=}, {self._goal_index}")
        self._navigate_next_sub_goal()

    # wrap execution by a queue
    def pause_navigation(self, callback):
        with self._process_queue_lock:
            self._process_queue.append((self._pause_navigation, callback))

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
        with self._process_queue_lock:
            self._process_queue.append((self._resume_navigation, callback))

    def _resume_navigation(self, callback):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "resume")

        current_pose = self.current_local_pose()
        goal, index = navgoal.estimate_next_goal(self._sub_goals, current_pose, self.current_floor)
        self._logger.info(F"navigation.{util.callee_name()} estimated next goal index={index}: {goal}")
        if goal:
            goal.estimate_inner_goal(current_pose, self.current_floor)
            self._goal_index = index-1
            self._last_estimated_goal = None
            self._navigate_next_sub_goal()
        else:
            self.reset_destination()

    # wrap execution by a queue
    def cancel_navigation(self, callback=None):
        with self._process_queue_lock:
            self._process_queue.append((self._cancel_navigation, callback))
        self.social_navigation.set_active(False)

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
        self.social_navigation.set_active(False)

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

    def _start_loop(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        if self.lock.acquire():
            if self._loop_handle is None:
                self._loop_handle = self._node.create_timer(0.1, self._check_loop, callback_group=self._main_callback_group)
            self.lock.release()

    def _stop_loop(self):
        return
        self._logger.info(F"navigation.{util.callee_name()} called")
        if self.lock.acquire():
            if self._loop_handle is not None:
                self._loop_handle.cancel()
                self._loop_handle = None
            self.lock.release()

    def _process_queue_func(self):
        process = None
        with self._process_queue_lock:
            if len(self._process_queue) > 0:
                process = self._process_queue.pop(0)
        try:
            if process:
                process[0](*process[1:])
        except:  # noqa: 722
            self._logger.error(traceback.format_exc())

    # Main loop of navigation
    GOAL_POSITION_TORELANCE = 1

    def _check_loop(self):
        self._logger.info("_check_loop", throttle_duration_sec=1.0)
        if not rclpy.ok():
            self._stop_loop()
            return

        # need a robot position
        try:
            self.current_pose = self.current_local_pose()
            self.delegate.update_pose(ros_pose=self.current_ros_pose(),
                                      current_pose=self.current_pose,
                                      global_position=self.current_global_pose(),
                                      current_floor=self.current_floor,
                                      global_frame=self._global_map_name
                                      )
            if self.post_location_enabled:
                if self._node.get_clock().now() - self.last_log_time > rclpy.duration.Duration(seconds=1.0):
                    self._datautil.post_location(self.current_global_pose(), self.current_floor)
                    self.last_log_time = self._node.get_clock().now()
            self._logger.debug(F"current pose {self.current_pose}", throttle_duration_sec=1)
            self.current_odom_pose = self.current_local_odom_pose()
        except RuntimeError:
            self._logger.info("could not get position", throttle_duration_sec=3)
            return
        except:  # noqa: E722
            self._logger.debug(traceback.format_exc())
            return

        # wait data is analyzed
        if not self._datautil.is_analyzed:
            self._logger.debug("datautil is not analyzed", throttle_duration_sec=1)
            return

        if not self.i_am_ready and \
           self.localize_status != MFLocalizeStatus.TRACKING:
            self._logger.debug("not ready and initialize localization", throttle_duration_sec=1)
            return

        # say I am ready once
        if not self.i_am_ready:
            self.i_am_ready = True
            self._logger.debug("i am ready")
            self.delegate.i_am_ready()
            self.request_defult_params()

        if self._current_goal is None:
            self._logger.debug("_current_goal is not set", throttle_duration_sec=1)
            return

        # cabot is active now
        self._logger.debug("cabot is active", throttle_duration_sec=1)

        # isolate error handling
        try:
            self._check_info_poi(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)
        try:
            self._check_gradient(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)
        try:
            self._check_nearby_facility(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)
        try:
            self._check_speed_limit(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)
        try:
            self._check_turn(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)
        try:
            self._check_queue_wait(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)
        try:
            self._check_social(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)
        try:
            self._check_goal(self.current_pose)
        except:  # noqa: E722
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)

    def _check_info_poi(self, current_pose):
        if not self.info_pois:
            return

        poi = min(self.info_pois, key=lambda p, c=current_pose: p.distance_to(c))

        if poi is not None and poi.distance_to(current_pose) < 8:
            # self._logger.info(F"{poi._id}, {poi.local_geometry}, {current_pose}")
            if poi.is_approaching(current_pose):
                self._logger.info(F"approaching {poi._id}")
                self.delegate.approaching_to_poi(poi=poi)
            elif poi.is_approached(current_pose):
                self._logger.info(F"approached {poi._id}")
                self.delegate.approached_to_poi(poi=poi)
            elif poi.is_passed(current_pose):
                self._logger.info(F"passed {poi._id}")
                self.delegate.passed_poi(poi=poi)

    def _check_gradient(self, current_pose):
        msg = std_msgs.msg.Float32()
        msg.data = 0.0
        for g in self.gradient:
            if g.within_link(current_pose):
                sign = 1.0 if g.gradient == geojson.Gradient.Up else -1.0
                msg.data = sign * float(g.max_gradient if g.max_gradient else 5.0)
                break
        self.gradient_pub.publish(msg)

    def _check_nearby_facility(self, current_pose):
        if not self.nearby_facilities:
            return
        entry = min(self.nearby_facilities, key=lambda p, c=current_pose: abs(p["entrance"].distance_to(c)))
        if entry is None:
            return
        entrance = entry["entrance"]

        if entrance is not None and entrance.distance_to(current_pose) < 8:
            # self._logger.info(F"_check_nearby_facility: {entrance._id}, {entrance}, {current_pose}")
            if entrance.is_approaching(current_pose):
                self._logger.info(F"_check_nearby_facility approaching {entrance._id}")
                self.delegate.approaching_to_poi(poi=entrance)
            elif entrance.is_approached(current_pose):
                self._logger.info(F"_check_nearby_facility approached {entrance._id}")
                self.delegate.approached_to_poi(poi=entrance)
            elif entrance.is_passed(current_pose):
                self._logger.info(F"_check_nearby_facility passed {entrance._id}")
                self.delegate.passed_poi(poi=entrance)

    """
     robot              target     POI
     o->                     |--D--o
     The robot will decrease the speed towards the target and keep speed until passing the POI
     THe robot velocity is calculated based on expected deceleration and delay
    """
    def _check_speed_limit(self, current_pose):
        target_distance = self._speed_poi_params[0]  # meters
        expected_deceleration = self._speed_poi_params[1]  # meters/seconds^2
        expected_delay = self._speed_poi_params[2]  # seconds

        def max_v(D, A, d):
            return (-2 * A * d + math.sqrt(4 * A * A * d * d + 8 * A * D)) / 2

        # check speed limit
        limit = self._max_speed
        for poi in self.speed_pois:
            dist = poi.distance_to(current_pose, adjusted=True)  # distance adjusted by angle
            if dist < 5.0:
                if poi.in_angle(current_pose):  # and poi.in_angle(c2p):
                    limit = min(limit, max(poi.limit, max_v(max(0, dist-target_distance), expected_deceleration, expected_delay)))
                    self._logger.debug(f"SpeedPOI {max_v(max(0, dist-target_distance), expected_deceleration, expected_delay)=}")
                    self._logger.debug(f"SpeedPOI {dist=}, {target_distance=}, {expected_deceleration=}, {expected_delay=}, {limit=}, {poi.limit=}")
                if limit < self._max_speed:
                    self._logger.debug(F"speed poi dist={dist:.2f}m, limit={limit:.2f}")
                    self.delegate.activity_log("cabot/navigation", "speed_poi", f"{limit}")

        msg = std_msgs.msg.Float32()
        msg.data = limit
        self.speed_limit_pub.publish(msg)

    def _check_turn(self, current_pose):
        # provide turn tactile notification
        if not self.turns:
            return

        self._logger.info("check turn", throttle_duration_sec=1)
        if self.turns is not None:
            for turn in self.turns:
                if turn.passed_vibrator and turn.passed_directional_indicator:
                    dist_to_end = numpy.sqrt((current_pose.x - turn.end.pose.position.x)**2 + (current_pose.y - turn.end.pose.position.y)**2)
                    self._logger.info(F"Distance to turn end: {dist_to_end}")
                    if dist_to_end < 0.75:
                        msg = std_msgs.msg.Bool()
                        msg.data = True
                        self.turn_end_pub.publish(msg)
                    continue
                try:
                    dist = turn.distance_to(current_pose)
                    if dist < self._notify_vib_threshold and not turn.passed_vibrator:
                        turn.passed_vibrator = True
                        self._logger.info(F"notify turn by vibrator {turn}")

                        if turn.turn_type == Turn.Type.Avoiding:
                            # give avoiding announce
                            self._logger.info("social_navigation avoiding turn")
                            self.social_navigation.turn = turn

                        if self._check_already_notified_turn_nearby(device="vibrator", turn=turn):
                            self.delegate.notify_turn(device="vibrator", turn=turn)
                    elif dist < self._notify_di_threshold and not turn.passed_directional_indicator:
                        turn.passed_directional_indicator = True
                        self._logger.info(F"notify turn by directional indicator {turn}")

                        if self._check_already_notified_turn_nearby(device="directional_indicator", turn=turn):
                            self.delegate.notify_turn(device="directional_indicator", turn=turn)
                except:  # noqa: E722
                    import traceback
                    self._logger.error(traceback.format_exc())
                    self._logger.error("could not convert pose for checking turn POI",
                                       throttle_duration_sec=3)

    def _check_already_notified_turn_nearby(self, device: str, turn: Turn):
        result = True
        for other in self.notified_turns[device]:
            if other.distance_to(turn) < Navigation.TURN_NEARBY_THRESHOLD:
                result = False
        self.notified_turns[device].append(turn)
        self._logger.info(f"_check_already_notified_turn_nearby, {device}, {result}, {turn}")
        return result

    def _check_queue_wait(self, current_pose):
        if not isinstance(self._current_goal, navgoal.QueueNavGoal) or not self.queue_wait_pois:
            return

        # Select queue wait POI which robot did not pass yet.
        # Even if POI is marked as passed, add POI which is closer than _queue_wait_pass_tolerance.
        poi = None
        forward_queue_wait_pois = [x for x in self.queue_wait_pois if geoutil.is_forward_point(current_pose, x) or x.distance_to(current_pose) < self._queue_wait_pass_tolerance]
        if len(forward_queue_wait_pois) > 0:
            poi = min(forward_queue_wait_pois, key=lambda p, c=current_pose: p.distance_to(c))

        # control speed by Queue POI
        limit = self._max_speed
        if poi is not None:
            poi_pose = poi.to_pose_msg()
            poi_position = numpy.array([poi_pose.position.x, poi_pose.position.y])

            current_position = numpy.array([current_pose.x, current_pose.y])
            current_position_on_queue_path = geoutil.get_projected_point_to_line(current_position, poi_position, poi.link_orientation)

            if self.current_queue_msg and len(self.current_queue_msg.people) > 0:
                tail_pose = geometry_msgs.msg.PoseStamped()
                tail_pose.header = self.current_queue_msg.header
                tail_pose.pose.position = self.current_queue_msg.people[-1].position
                try:
                    tail_global_pose = self.buffer.transform(tail_pose, self._global_map_name)
                    tail_global_position = numpy.array([tail_global_pose.pose.position.x, tail_global_pose.pose.position.y])
                    tail_global_position_on_queue_path = geoutil.get_projected_point_to_line(tail_global_position, poi_position, poi.link_orientation)
                    dist_robot_to_tail = numpy.linalg.norm(tail_global_position_on_queue_path - current_position_on_queue_path)
                    if dist_robot_to_tail <= self.current_queue_interval:
                        # if distance to queue tail is smaller than queue wait interval, stop immediately
                        limit = 0.0
                        self._logger.info(F"Stop at current position, tail is closer than next queue wait POI, Set speed limit={limit}, dist_robot_to_tail={dist_robot_to_tail}",
                                          throttle_duration_sec=1)
                    else:
                        # adjust Queue POI position by moving closer to link target node
                        poi_orientation = tf_transformations.euler_from_quaternion([poi.link_orientation.x, poi.link_orientation.y, poi.link_orientation.z, poi.link_orientation.w])
                        poi_fixed_position = poi_position + numpy.array([self._queue_wait_position_offset*math.cos(poi_orientation[2]), self._queue_wait_position_offset*math.sin(poi_orientation[2])])
                        dist_tail_to_queue_wait = numpy.linalg.norm(tail_global_position_on_queue_path - poi_fixed_position)
                        if dist_tail_to_queue_wait > max(2.0*self.current_queue_interval - self._queue_tail_dist_error_tolerance, self.current_queue_interval):
                            # If distance from next queue wait point to queue tail is larger than twice of queue wait interval,
                            # there should be another queue wait point. Skip next queue wait point.
                            self._logger.info(F"Skip next queue wait POI, POI is far from robot. dist_tail_to_queue_wait={dist_tail_to_queue_wait}, dist_robot_to_tail={dist_robot_to_tail}",
                                              throttle_duration_sec=1)
                        else:
                            dist_tail_to_queue_path = numpy.linalg.norm(tail_global_position_on_queue_path - tail_global_position)
                            if (dist_tail_to_queue_path > self._queue_tail_ignore_path_dist):
                                # If distance from queue tail to queue path is larger than queue_tail_ignore_path_dist, robot and queue tail person should be on different queue paths.
                                # Skip next queue wait point.
                                self._logger.info(F"Skip next queue wait POI, tail is far from path. dist_tail_to_queue_path={dist_tail_to_queue_path}", throttle_duration_sec=1)
                            else:
                                # limit speed by using adjusted Queue POI
                                dist_robot_to_queue_wait = numpy.linalg.norm(poi_fixed_position - current_position_on_queue_path)
                                limit = min(limit, math.sqrt(2.0 * max(0.0, dist_robot_to_queue_wait - self._queue_wait_arrive_tolerance) * self._max_acc))
                                self._logger.info(F"Set speed limit={limit}, dist_robot_to_queue_wait={dist_robot_to_queue_wait}", throttle_duration_sec=1)
                except:  # noqa: E722
                    self._logger.error("could not convert pose for checking queue POI", throttle_duration_sec=3)
            else:
                self._logger.info("No queue people", throttle_duration_sec=1)
        msg = std_msgs.msg.Float32()
        msg.data = limit
        self.queue_speed_limit_pub.publish(msg)

        # announce beginning of the queue
        if self.need_queue_start_arrived_info and limit < self._max_speed:
            self.delegate.queue_start_arrived()
            self.need_queue_start_arrived_info = False
        # announce proceed in queue navigation
        if not self.need_queue_proceed_info and limit == 0.0:
            self.need_queue_proceed_info = True
        if self.need_queue_proceed_info and limit == self._max_speed:
            self.delegate.queue_proceed()
            self.need_queue_proceed_info = False

    def _check_social(self, current_pose):
        if self.social_navigation is None:
            return
        self._logger.info(F"navigation.{util.callee_name()} called", throttle_duration_sec=1)

        # do not provide social navigation messages while queue navigation
        if self._current_goal and not self._current_goal.is_social_navigation_enabled:
            self._logger.info("social navigation is disabled")
            return

        self.social_navigation.current_pose = current_pose
        message = self.social_navigation.get_message()
        if message is not None:
            self.delegate.announce_social(message)
        sound = self.social_navigation.get_sound()
        if sound is not None:
            self.delegate.request_sound(sound)

    def _check_goal(self, current_pose):
        self._logger.info(F"navigation.{util.callee_name()} called", throttle_duration_sec=1)
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
                self.delegate.activity_log("cabot/navigation", "estimated_next_goal", F"{repr(estimated_goal)}")
            self._last_estimated_goal = estimated_goal
            self._last_estimated_goal_check = now

        if goal.is_canceled:
            self._stop_loop()
            self._current_goal = None
            self._goal_index = max(-1, self._goal_index - 1)
            self.delegate.goal_canceled(goal)
            self.delegate.activity_log("cabot/navigation", "goal_canceled", F"{goal.__class__.__name__}")
            return

        if not goal.is_completed:
            return

        if goal.is_exiting_goal:
            return

        def goal_exit_callback():
            self.delegate.activity_log("cabot/navigation", "goal_completed", F"{goal.__class__.__name__}")
            self._current_goal = None
            if goal.is_last:
                # keep this for test
                self.delegate.activity_log("cabot/navigation", "navigation", "arrived")
                self.delegate.have_arrived(goal)
            self._navigate_next_sub_goal()
        goal.exit(goal_exit_callback)

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

        if behavior_tree.startswith("package://"):
            start = len("package://")
            end = behavior_tree.find("/", len("pacakge://"))
            package = behavior_tree[start:end]
            behavior_tree = get_package_share_directory(package) + behavior_tree[end:]
            self._logger.info(F"package={package}, behavior_tree={behavior_tree}")

        goal.behavior_tree = behavior_tree

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

    def turn_towards(self, orientation, gh_callback, callback, clockwise=0, time_limit=10.0):
        self._logger.info("turn_towards")
        self.delegate.activity_log("cabot/navigation", "turn_towards",
                                   str(geoutil.get_yaw(geoutil.q_from_msg(orientation))))
        self.turn_towards_count = 0
        self.turn_towards_last_diff = None
        self._turn_towards(orientation, gh_callback, callback, clockwise, time_limit)

    def _turn_towards(self, orientation, gh_callback, callback, clockwise=0, time_limit=10.0):
        goal = nav2_msgs.action.Spin.Goal()
        diff = geoutil.diff_angle(self.current_pose.orientation, orientation)
        time_allowance = max(3.0, min(time_limit, abs(diff)/0.3))
        goal.time_allowance = rclpy.duration.Duration(seconds=time_allowance).to_msg()

        self._logger.info(F"current pose {self.current_pose}, diff {diff:.2f}")
        if (clockwise < 0 and diff < - math.pi / 4) or \
           (clockwise > 0 and diff > + math.pi / 4):
            diff = diff - clockwise * math.pi * 2
        turn_yaw = diff - (diff / abs(diff) * 0.05)
        goal.target_yaw = turn_yaw

        msg = std_msgs.msg.Float32()
        msg.data = geoutil.normalize_angle(turn_yaw)
        self.turn_angle_pub.publish(msg)

        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(lambda future: self._turn_towards_sent_goal(goal, future, orientation, gh_callback, callback, clockwise, turn_yaw, time_limit))
        return future

    def _turn_towards_sent_goal(self, goal, future, orientation, gh_callback, callback, clockwise, turn_yaw, time_limit):
        self._logger.info(F"_turn_towards_sent_goal: {goal=}")
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

            if abs(diff) > 0.05 and self.turn_towards_count < 3:
                self._logger.info(F"send turn {diff:.2f}")
                self.turn_towards_last_diff = diff
                self._turn_towards(orientation, gh_callback, callback, clockwise, time_limit)
            else:
                self._logger.info(F"turn completed {diff=}, {self.turn_towards_count=}")
                msg = std_msgs.msg.Bool()
                msg.data = True
                self.rotation_complete_pub.publish(msg)
                callback(True)
        get_result_future.add_done_callback(done_callback)

        # add position and use quaternion to visualize
        # self.visualizer.goal = goal
        # self.visualizer.visualize()
        # self._logger.info(F"visualize goal {goal}")
        angle = turn_yaw * 180 / math.pi
        pose = self.current_pose.to_pose_stamped_msg(self._global_map_name)
        if abs(angle) >= 180/3:
            self.delegate.notify_turn(device="vibrator", turn=Turn(pose, angle, Turn.Type.Normal))
        elif abs(angle) >= 180/6:
            self.delegate.notify_turn(device="vibrator", turn=Turn(pose, angle, Turn.Type.Avoiding))
        self._logger.info(F"notify turn {turn_yaw}")

    def goto_floor(self, floor, gh_callback, callback):
        try:
            self._goto_floor(floor, gh_callback, callback)
        except:  # noqa: #722
            self._logger.error(traceback.format_exc())

    def _goto_floor(self, floor, gh_callback, callback):
        self._logger.info(F"go to floor {floor}")
        self.delegate.activity_log("cabot/navigation", "go_to_floor", str(floor))

        class GotoFloorTask():
            def __init__(self, nav):
                self._nav = nav
                self._node = nav._node
                self._logger = nav._logger
                self.buffer = nav.buffer
                self.delegate = nav.delegate
                self.future = self._node.executor.create_task(self.handler)
                self.future.add_done_callback(lambda x: callback(True))
                self.rate = self._nav._node.create_rate(2)
                self.cancelled = False

            def cancel_goal_async(self):
                def dummy():
                    self._logger.info("cancel goal async wait")
                    self.rate.sleep()
                    self._logger.info("cancel goal async done")
                    self.cancel_future.done()
                self.cancel_future = self._node.executor.create_task(dummy)
                self.cancelled = True
                return self.cancel_future

            def handler(self):
                try:
                    self._handle()
                except:  # noqa: #722
                    self._logger.error(traceback.format_exc())

            def _handle(self):
                first = True
                while rclpy.ok():
                    self.rate.sleep()
                    self._logger.info(F"GotoFloorTask loop cancelled={self.cancelled}")
                    if self.cancelled:
                        self._logger.info("GotoFloorTask cancelled")
                        break
                    if self._nav.current_floor != floor:
                        continue
                    if first:
                        first = False
                        self.delegate.floor_changed(self._nav.current_floor)

                    self._logger.info(F"trying to find tf from map to {self._nav.current_frame}")
                    try:
                        self.buffer.lookup_transform("map", self._nav.current_frame, CaBotRclpyUtil.time_zero())
                        break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException):
                        self._logger.warn(F"Could not find tf from map to {self._nav.current_frame}")
                self.future.done()

        task = GotoFloorTask(self)
        gh_callback(task)

    def set_pause_control(self, flag):
        self._logger.info("set_pause_control")
        self.delegate.activity_log("cabot/navigation", "pause_control", str(flag))
        self.pause_control_msg.data = flag
        self.pause_control_pub.publish(self.pause_control_msg)
        if self.pause_control_loop_handler is None:
            self.pause_control_loop_handler = self._node.create_timer(1, self.pause_control_loop, callback_group=MutuallyExclusiveCallbackGroup())

    def pause_control_loop(self):
        self.pause_control_pub.publish(self.pause_control_msg)

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

    def please_call_elevator(self, pos):
        self.delegate.please_call_elevator(pos)

    def please_pass_door(self):
        self.delegate.please_pass_door()

    def door_passed(self):
        self.delegate.door_passed()

    def global_map_name(self):
        return self._global_map_name

    def please_follow_behind(self):
        self.delegate.please_follow_behind()

    def please_return_position(self):
        self.delegate.please_return_position()

    def change_parameters(self, params, callback):
        self.param_manager.change_parameters(params, callback)

    def request_parameters(self, params, callback):
        self.param_manager.request_parameters(params, callback)


class NavigationParamManager:
    def __init__(self, node):
        self.node = node
        self.clients = {}
        self.callback_group = MutuallyExclusiveCallbackGroup()

    def get_client(self, node_name, service_type, service_name):
        key = f'{node_name}/{service_name}'
        if key not in self.clients:
            for i in range(10):
                client = self.node.create_client(service_type, key, callback_group=self.callback_group)
                if client.wait_for_service(timeout_sec=1.0):
                    self.node.get_logger().info(f'{key} is available')
                    self.clients[key] = client
                    break
                self.node.get_logger().error(f'{key} is not available (retry {i+1})...')
        return self.clients[key]

    def change_parameter(self, node_name, param_dict, callback):
        def done_callback(future):
            callback(node_name, future)
        request = SetParameters.Request()
        for param_name, param_value in param_dict.items():
            new_parameter = Parameter(param_name, value=param_value)
            request.parameters.append(new_parameter.to_parameter_msg())
        client = self.get_client(node_name, SetParameters, "set_parameters")
        if client:
            future = client.call_async(request)
            future.add_done_callback(done_callback)
        else:
            done_callback(None)

    def change_parameters(self, params, callback):
        params = copy.deepcopy(params)

        def sub_callback(node_name, future):
            del params[node_name]
            self.node.get_logger().info(f"change_parameter sub_callback {node_name} {len(params)} {future.result() if future else None}")
            if len(params) == 0:
                if future:
                    callback(future.result())
                else:
                    callback(None)
            else:
                self.change_parameters(params, callback)
        for node_name, param_dict in params.items():
            self.node.get_logger().info(f"call change_parameter {node_name}, {param_dict}")
            try:
                self.change_parameter(node_name, param_dict, sub_callback)
            except:  # noqa: 722
                self.node.get_logger().error(traceback.format_exc())
            break

    def request_parameter(self, node_name, param_list, callback):
        def done_callback(future):
            callback(node_name, param_list, future)
        request = GetParameters.Request()
        request.names = param_list
        client = self.get_client(node_name, GetParameters, "get_parameters")
        if client:
            future = client.call_async(request)
            future.add_done_callback(done_callback)
        else:
            done_callback(None)

    def request_parameters(self, params, callback):
        self.rcount = 0
        self.result = {}

        def sub_callback(node_name, param_list, future):
            self.rcount += 1
            self.result[node_name] = {}
            if future:
                for name, value in zip(param_list, future.result().values):
                    msg = rcl_interfaces.msg.Parameter()
                    msg.name = name
                    msg.value = value
                    param = Parameter.from_parameter_msg(msg)
                    self.result[node_name][name] = param.value
            if self.rcount == len(params):
                if future:
                    self.node.get_logger().info(f"request_parameter sub_callback {self.rcount} {len(params)} {node_name}, {param_list}, {future.result()}")
                else:
                    self.node.get_logger().info(f"request_parameter sub_callback {self.rcount} {len(params)} {node_name}, {param_list}, None")
                callback(self.result)
        for node_name, param_list in params.items():
            self.node.get_logger().info(f"call request_parameter {node_name}, {param_list}")
            try:
                self.request_parameter(node_name, param_list, sub_callback)
            except:  # noqa: 722
                self.node.get_logger().error(traceback.format_exc())
