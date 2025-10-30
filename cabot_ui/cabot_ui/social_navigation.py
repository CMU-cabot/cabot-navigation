# Copyright (c) 2020, 2022  Carnegie Mellon University
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

import traceback
import rclpy.node
import rclpy.time
import rclpy.clock
from rclpy.duration import Duration
# from cabot_ui.turn_detector import Turn
from people_msgs.msg import People
import tf2_geometry_msgs  # noqa: to register class for transform
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from cabot_msgs.msg import StopReason, SignalState
from cabot_ui.geojson import Signal

from dataclasses import dataclass
import enum


@dataclass
class SNMessage:
    class Type(enum.Enum):
        Message = enum.auto()
        Sound = enum.auto()

    class Code(enum.Enum):
        PERSON_AHEAD = enum.auto()
        OBSTACLE_AHEAD = enum.auto()
        PLEASE_WAIT_FOR_A_SECOND = enum.auto()
        NOT_DETECT_TOUCH = enum.auto()
        # not used, avoid
        AVOIDING_A_PERSON = enum.auto()
        AVOIDING_PEOPLE = enum.auto()
        AVOIDING_AN_OBSTACLE = enum.auto()
        AVOIDING_OBSTACLES = enum.auto()
        # not used, stop
        TRYING_TO_AVOID_PEOPLE = enum.auto()
        PEOPLE_ARE_IN_MY_WAY = enum.auto()
        TRYING_TO_AVOID_OBSTACLE = enum.auto()
        # not used, speed control
        A_PERSON_IN_THE_WAY = enum.auto()
        PEOPLE_IN_THE_WAY = enum.auto()
        # not used, follow
        FOLLOWING_A_PERSON = enum.auto()
        FOLLOWING_PEOPLE = enum.auto()
        # signal
        RED_SIGNAL = enum.auto()
        RED_SIGNAL_DETAIL = enum.auto()
        GREEN_SIGNAL_SHORT = enum.auto()
        NO_SIGNAL_INFO = enum.auto()
        GREEN_SIGNAL = enum.auto()

    class Category(enum.Enum):
        AVOID = enum.auto()
        STOP = enum.auto()
        IN_THE_WAY = enum.auto()
        FOLLOWING = enum.auto()

    type: Type
    code: Code
    param: str
    category: Category
    time: rclpy.time.Time
    priority: int

    @staticmethod
    def empty(type: Type, clock: rclpy.clock.Clock):
        return SNMessage(type=type, code=None, param=None, category=None, time=clock.now(), priority=0)

    @staticmethod
    def empty_message(clock: rclpy.clock.Clock):
        return SNMessage.empty(SNMessage.Type.Message, clock)

    @staticmethod
    def empty_sound(clock: rclpy.clock.Clock):
        return SNMessage.empty(SNMessage.Type.Sound, clock)


class SocialNavigation(object):
    def __init__(self, node: rclpy.node.Node, buffer):
        self._node = node
        self._logger = node.get_logger()
        self._buffer = buffer
        self._path = None
        self._turn = None
        self._current_pose = None
        self._latest_odom = None
        self._latest_people = None
        self._latest_obstacles = None
        self._people_count = 0
        self._obstacles_count = 0
        self._event = None
        self._latest_signal_state = None

        self._message: SNMessage = SNMessage.empty_message(node.get_clock())
        self._last_message: SNMessage = SNMessage.empty_message(node.get_clock())
        self._sound: SNMessage = SNMessage.empty_sound(node.get_clock())
        self._last_sound: SNMessage = SNMessage.empty_sound(node.get_clock())

        self._stop_reason = None
        self._last_stop_reason = None
        self._is_active = False
        odom_topic = node.declare_parameter("odom_topic", "/odom").value
        people_topic = node.declare_parameter("people_topic", "/people").value
        obstacles_topic = node.declare_parameter("obstacles_topic", "/obstacles").value
        stop_reason_topic = node.declare_parameter("stop_reason_topic", "/stop_reason").value
        signal_state_topic = node.declare_parameter("signal_state_topic", "/cabot/signal_state").value
        self.odom_topic = node.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.people_sub = node.create_subscription(People, people_topic, self._people_callback, 10)
        self.obstacles_sub = node.create_subscription(People, obstacles_topic, self._obstacles_callback, 10)
        self.stop_reason_sub = node.create_subscription(StopReason, stop_reason_topic, self._stop_reason_callback, 10)
        self.signal_state_sub = node.create_subscription(SignalState, signal_state_topic, self._signal_state_callback, 10)

        self.timer = node.create_timer(0.5, self._update)

    def set_active(self, active):
        self._is_active = active

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _people_callback(self, msg):
        self._latest_people = msg

        if self._buffer is None:
            return

        count = 0
        try:
            for person in msg.people:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id
                point_stamped.point = person.position
                point_stamped = self._buffer.transform(point_stamped, "base_footprint")
                if abs(point_stamped.point.y) < 1.5 and \
                   abs(point_stamped.point.y) < point_stamped.point.x and \
                   0 < point_stamped.point.x and point_stamped.point.x < 5:
                    count += 1
        except:  # noqa: #722
            pass

        self._people_count = count
        # self._update()

    def _obstacles_callback(self, msg):
        self._latest_obstacles = msg

        if self._buffer is None:
            return

        count = 0
        # using person as obstacle
        try:
            for person in msg.people:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id
                point_stamped.point = person.position
                point_stamped = self._buffer.transform(point_stamped, "base_footprint")
                if abs(point_stamped.point.y) < 1.5 and \
                   abs(point_stamped.point.y) < point_stamped.point.x and \
                   0 < point_stamped.point.x and point_stamped.point.x < 5:
                    count += 1
        except:  # noqa: #722
            pass

        self._obstacles_count = count
        # self._update()

    def _stop_reason_callback(self, msg: StopReason):
        try:
            # if msg.summary:
            self._stop_reason = msg.reason
            self._logger.info("_stop_reason_callback summary is True and calling _update()")
            # self._update()
        except:  # noqa: 722
            self._logger.error(traceback.format_exc())

    def _signal_state_callback(self, msg: SignalState):
        self._latest_signal_state = msg
        # self._update()

    def _update(self):
        '''
        Message types:
          - Avoiding something
          - Stop due to
          - Speed is control by
            - Person (People)      - always say PERSON_AHEAD since 2024.03.28
            - Obstacle (Obstacles) - always play sound       since 2024.03.28
        '''
        # disable message when avoiding something
        '''
        if self._turn is not None and self._turn.turn_type == Turn.Type.Avoiding:
            self._logger.info(F"social navigation update turn={self._turn}")
            self._logger.info(F"avoiding turn, people count = {self._people_count}, "
                              F"obstacle count = {self._obstacles_count}")
            if self._people_count == 1:
                self._set_message(SNMessage.Code.PERSON_AHEAD, SNMessage.Category.AVOID, 10)
                # self._set_message(SNMessage.Code.AVOIDING_A_PERSON, SNMessage.Category.AVOID, 10)
            elif self._people_count > 1:
                self._set_message(SNMessage.Code.PERSON_AHEAD, SNMessage.Category.AVOID, 10)
                # self._set_message(SNMessage.Code.AVOIDING_PEOPLE, SNMessage.Category.AVOID, 10)
            elif self._obstacles_count == 1:
                self._set_sound(SNMessage.Code.OBSTACLE_AHEAD, SNMessage.Category.AVOID, 10)
                # self._set_sound(SNMessage.Code.AVOIDING_AN_OBSTACLE, SNMessage.Category.AVOID)
            elif self._obstacles_count > 1:
                self._set_sound(SNMessage.Code.OBSTACLE_AHEAD, SNMessage.Category.AVOID, 10)
                # self._set_sound(SNMessage.Code.AVOIDING_OBSTACLES, SNMessage.Category.AVOID)
            self._turn = None
        '''

        if self._stop_reason is not None:
            self._logger.info(F"social-navigation stop_reason {self._stop_reason}, last_stop_reason {self._last_stop_reason}")
            code = self._stop_reason
            skip = False
            if code == "AVOIDING_PEOPLE":
                self._set_message(SNMessage.Code.PERSON_AHEAD, SNMessage.Category.AVOID, 7)
                # self._set_message(SNMessage.Code.TRYING_TO_AVOID_PEOPLE, SNMessage.STOP, 7)
            elif code == "THERE_ARE_PEOPLE_IN_THE_PATH":
                self._set_message(SNMessage.Code.PERSON_AHEAD, SNMessage.Category.STOP, 7)
                # self._set_message(SNMessage.Code.PEOPLE_ARE_IN_MY_WAY, SNMessage.STOP, 7)
            elif code == "AVOIDING_OBSTACLE":
                self._set_sound(SNMessage.Code.OBSTACLE_AHEAD, SNMessage.Category.AVOID, 7)
                # self._set_sound(SNMessage.Code.TRYING_TO_AVOID_OBSTACLE, SNMessage.STOP, 7)
            elif code == "RED_SIGNAL":
                remaining_time = round(self._latest_signal_state.remaining_time) if self._latest_signal_state is not None else None
                if remaining_time > 0 and remaining_time % 5 == 0:
                    if self._last_stop_reason != "RED_SIGNAL":
                        user_speed = round(self._latest_signal_state.user_speed, 2) if self._latest_signal_state is not None else None
                        distance = round(self._latest_signal_state.distance) if self._latest_signal_state is not None else None
                        expected_time = round(self._latest_signal_state.expected_time) if self._latest_signal_state is not None else None
                        next_programmed_seconds = round(self._latest_signal_state.next_programmed_seconds) if self._latest_signal_state is not None else None
                        self._set_message(SNMessage.Code.RED_SIGNAL_DETAIL, SNMessage.Category.STOP, 7,
                                          param=[remaining_time, user_speed, distance, expected_time, next_programmed_seconds])
                    else:
                        self._set_message(SNMessage.Code.RED_SIGNAL, SNMessage.Category.STOP, 7, param=remaining_time)
                else:
                    skip = True
            elif code == "GREEN_SIGNAL_SHORT":
                param = round(self._latest_signal_state.remaining_time) if self._latest_signal_state is not None else None
                if param > 0 and param % 5 == 0:
                    self._set_message(SNMessage.Code.GREEN_SIGNAL_SHORT, SNMessage.Category.STOP, 7, param=param)
            elif code == "NO_SIGNAL_INFO":
                self._set_message(SNMessage.Code.NO_SIGNAL_INFO, SNMessage.Category.STOP, 7)
            elif code == "UNKNOWN":
                # self._set_message(SNMessage.Code.PLEASE_WAIT_FOR_A_SECOND, SNMessage.Category.STOP, 7)
                pass
            elif code == "NO_TOUCH":
                self._set_message(SNMessage.Code.NOT_DETECT_TOUCH, SNMessage.Category.STOP, 7)
            elif code == "NOT_STOPPED":
                # announce green signal only when changing from red to green
                if self._last_stop_reason == "RED_SIGNAL":
                    if self._latest_signal_state.state == "GREEN_SIGNAL":
                        param = round(self._latest_signal_state.remaining_time) if self._latest_signal_state is not None else None
                        self._set_message(SNMessage.Code.GREEN_SIGNAL, SNMessage.Category.STOP, 7, param=param)
                    else:
                        skip = True
            if not skip:
                self._last_stop_reason = self._stop_reason
                self._stop_reason = None
            self._logger.info(F"social-navigation {self._message}")

        # check event
        if self._event is not None:
            self._logger.info(F"social navigation event {self._event}")
            param = self._event.param

            # stops due to people on the path
            if param == "people_speed_stopped":
                if self._people_count == 1:
                    self._set_message(SNMessage.Code.PERSON_AHEAD, SNMessage.Category.IN_THE_WAY, 5)
                    # self._set_message(SNMessage.Code.A_PERSON_IN_THE_WAY, SNMessage.Category.IN_THE_WAY, 5)
                elif self._people_count > 1:
                    self._set_message(SNMessage.Code.PERSON_AHEAD, SNMessage.Category.IN_THE_WAY, 5)
                    # self._set_message(SNMessage.Code.PEOPLE_IN_THE_WAY, SNMessage.Category.IN_THE_WAY, 5)

            # following people
            if param == "people_speed_following":
                if self._people_count == 1:
                    self._set_message(SNMessage.Code.FOLLOWING_A_PERSON, SNMessage.Category.FOLLOWING, 1)
                elif self._people_count > 1:
                    self._set_message(SNMessage.Code.FOLLOWING_PEOPLE, SNMessage.Category.FOLLOWING, 1)

            # delete event after check
            self._event = None

    def _set_message(self, code, category, priority, param=None):
        now = self._node.get_clock().now()
        self._logger.info(F"set_message {code} {category} {priority} {self._last_message} {now - self._last_message.time}")
        if (self._last_message.priority < priority and self._last_message.category != category) or \
           (now - self._last_message.time) > Duration(seconds=4.5):
            self._message.code = code
            self._message.category = category
            self._message.priority = priority
            self._message.time = now
            self._message.param = param

    def _set_sound(self, code, category, priority):
        self._logger.info(F"set_sound {code} {category} {priority}")
        now = self._node.get_clock().now()
        if (self._last_sound.priority < priority and self._last_sound.category != category) or \
           (now - self._last_sound.time) > Duration(seconds=4.5):
            self._sound.code = code
            self._sound.category = category
            self._sound.priority = priority
            self._sound.time = now

    def get_message(self) -> SNMessage:
        if not self._is_active:
            return
        now = self._node.get_clock().now()
        if self._message.code is not None and (now - self._last_message.time) > Duration(seconds=4.9):
            self._logger.info(f"get_message {self._message}")
            self._last_message = self._message
            self._message = SNMessage.empty_message(self._node.get_clock())
            return self._last_message
        return None

    def get_sound(self) -> SNMessage:
        if not self._is_active:
            return
        now = self._node.get_clock().now()

        if self._sound.code is not None and (now - self._last_sound.time) > Duration(seconds=5.0):
            self._last_sound = self._sound
            self._sound = SNMessage.empty_sound(self._node.get_clock())
            return self._last_sound
        return None

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, path):
        self._path = path
        # self._update()

    @path.deleter
    def path(self):
        del self._path

    @property
    def turn(self):
        return self._turn

    @turn.setter
    def turn(self, turn):
        self._turn = turn
        # self._update()

    @turn.deleter
    def turn(self):
        del self._turn

    @property
    def event(self):
        return self._event

    @event.setter
    def event(self, event):
        self._event = event
        # self._update()

    @event.deleter
    def event(self):
        del self._event

    @property
    def current_pose(self):
        return self._current_pose

    @current_pose.setter
    def current_pose(self, current_pose):
        self._current_pose = current_pose
        # self._update()

    @current_pose.deleter
    def current_pose(self):
        del self._current_pose
