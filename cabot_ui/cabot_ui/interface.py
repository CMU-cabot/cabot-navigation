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

import os

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.exceptions import InvalidServiceNameException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

import std_msgs.msg
import cabot_msgs.msg
import cabot_msgs.srv
from cabot_ui import visualizer, i18n
from cabot_ui.event import NavigationEvent
from cabot_ui.turn_detector import Turn
from cabot_ui.social_navigation import SNMessage
from cabot_common import vibration


class UserInterface(object):
    SOCIAL_ANNOUNCE_INTERVAL = Duration(seconds=15.0)
    NOTIFY_TURN_INTERVAL = Duration(seconds=5.0)

    def __init__(self, node: Node):
        self._node = node
        self.visualizer = visualizer.instance(node)

        self.note_pub = node.create_publisher(std_msgs.msg.Int8, "/cabot/notification", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.activity_log_pub = node.create_publisher(cabot_msgs.msg.Log, "/cabot/activity_log", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.pose_log_pub = node.create_publisher(cabot_msgs.msg.PoseLog, "/cabot/pose_log", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.event_pub = self._node.create_publisher(std_msgs.msg.String, "/cabot/event", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.turn_angle_pub = node.create_publisher(std_msgs.msg.Float32, "/cabot/turn_angle", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.turn_type_pub = node.create_publisher(std_msgs.msg.String, "/cabot/turn_type", 10, callback_group=MutuallyExclusiveCallbackGroup())

        self.lang = node.declare_parameter("language", "en").value
        self.site = node.declare_parameter("site", '').value

        self.last_pose = None
        self.read_aloud = False
        if "CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION" in os.environ:
            if os.environ["CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION"] == "1":
                self.read_aloud = True
                CaBotRclpyUtil.info("Read aloud vib notification is set to True")
            else:
                CaBotRclpyUtil.info("Read aloud vib notification is set to False")
        else:
            CaBotRclpyUtil.info("Read aloud vib notification is set to False (no CABOT_OPTION_READ_ALOUD_VIB_NOTIFICATION value)")

        i18n.set_language(self.lang)

        packages = ['cabot_ui']
        if self.site:
            packages.append(self.site)
        i18n.load_from_packages(packages)

        self.last_social_announce = None
        self.last_notify_turn = None

    def _activity_log(self, category="", text="", memo="", visualize=False):
        log = cabot_msgs.msg.Log()
        log.header.stamp = self._node.get_clock().now().to_msg()
        log.category = category
        log.text = text
        log.memo = memo
        self.activity_log_pub.publish(log)
        CaBotRclpyUtil.info(F"{category}:{text}:{memo}")

        if visualize and self.last_pose is not None:
            self.visualizer.spoken.append((self.last_pose['ros_pose'], F"{text}, {memo}", category))
            self.visualizer.visualize()

    def _pose_log(self):
        if not self.last_pose:
            return
        log = cabot_msgs.msg.PoseLog()
        log.header.stamp = self._node.get_clock().now().to_msg()
        log.header.frame_id = self.last_pose['global_frame']
        log.pose = self.last_pose['ros_pose']
        log.lat = self.last_pose['global_position'].lat
        log.lng = self.last_pose['global_position'].lng
        log.floor = self.last_pose['current_floor']
        self.pose_log_pub.publish(log)

    def change_language(self, lang):
        self._activity_log("change language", lang, f"previous={self.lang}")
        self.lang = lang
        i18n.set_language(self.lang)

    def speak(self, text, force=True, pitch=50, volume=50, rate=50):
        if text is None:
            return

        self._activity_log("speech request", text, self.lang, visualize=True)

        # TODO:
        voice = 'male'
        rate = 50

        speak_proxy = self._node.create_client(cabot_msgs.srv.Speak, '/speak', callback_group=MutuallyExclusiveCallbackGroup())
        try:
            if not speak_proxy.wait_for_service(timeout_sec=1):
                CaBotRclpyUtil.error("Service cannot be found")
                return
            CaBotRclpyUtil.info(F"try to speak {text} (v={voice}, r={rate}, p={pitch}) {force}")
            request = cabot_msgs.srv.Speak.Request()
            request.text = text
            request.rate = rate
            request.pitch = pitch
            request.volume = volume
            request.lang = self.lang
            request.voice = voice
            request.force = force
            request.priority = 50
            request.timeout = 2.0
            request.channels = cabot_msgs.srv.Speak.Request.CHANNEL_BOTH
            speak_proxy.call_async(request)
            CaBotRclpyUtil.info("speak requested")
        except InvalidServiceNameException as e:
            CaBotRclpyUtil.error(F"Service call failed: {e}")

    def vibrate(self, pattern=vibration.UNKNOWN):
        self._activity_log("cabot/interface", "vibration", vibration.get_name(pattern), visualize=True)
        msg = std_msgs.msg.Int8()
        msg.data = pattern
        self.note_pub.publish(msg)

    def read_aloud_vibration(self, pattern=vibration.UNKNOWN):
        if not self.read_aloud:
            return

        if pattern == vibration.FRONT:
            self.speak(i18n.localized_string("HANDLE_START"))
        elif pattern == vibration.RIGHT_ABOUT_TURN:
            self.speak(i18n.localized_string("HANDLE_RIGHT_ABOUT_TURN"))
        elif pattern == vibration.RIGHT_TURN:
            self.speak(i18n.localized_string("HANDLE_RIGHT_TURN"))
        elif pattern == vibration.RIGHT_DEV:
            self.speak(i18n.localized_string("HANDLE_RIGHT_DEV"))
        elif pattern == vibration.LEFT_ABOUT_TURN:
            self.speak(i18n.localized_string("HANDLE_LEFT_ABOUT_TURN"))
        elif pattern == vibration.LEFT_TURN:
            self.speak(i18n.localized_string("HANDLE_LEFT_TURN"))
        elif pattern == vibration.LEFT_DEV:
            self.speak(i18n.localized_string("HANDLE_LEFT_DEV"))

    # menu interface

    def menu_changed(self, menu=None, backed=False, usage=False):
        if menu is None:
            return

        if backed:
            self.speak(menu.title, force=True)

        self.speak(menu.description, force=not backed)
        if usage and menu.usage:
            self.speak("__pose__", force=False)
            self.speak(menu.usage, force=False, pitch=25)

    def pause_navigation(self):
        self._activity_log("cabot/interface", "navigation", "pause")
        self.speak(i18n.localized_string("PAUSE_NAVIGATION"))

    def cancel_navigation(self):
        pass  # self.speak(i18n.localized_string("CANCEL_NAVIGATION"))

    def resume_navigation(self):
        self._activity_log("cabot/interface", "navigation", "resume")
        self.speak(i18n.localized_string("RESUME_NAVIGATION"))

    def start_exploration(self):
        pass  # self.speak(i18n.localized_string("START_EXPLORATION"))

    # navigate interface

    def activity_log(self, category="", text="", memo=""):
        self._activity_log(category, text, memo)

    def in_preparation(self):
        self._activity_log("cabot/interface", "status", "prepare")
        self.speak(i18n.localized_string("IN_PRERARATION"))

    def i_am_ready(self):
        self._activity_log("cabot/interface", "status", "ready")
        self.speak(i18n.localized_string("I_AM_READY"))

    def start_navigation(self):
        self._activity_log("cabot/interface", "navigation", "start")
        self.vibrate(vibration.FRONT)
        self.read_aloud_vibration(vibration.FRONT)

    def update_pose(self, **kwargs):
        self.last_pose = kwargs
        self._pose_log()

    def notify_turn(self, turn=None, pose=None):
        if self.last_notify_turn and \
           self._node.get_clock().now() - self.last_notify_turn < UserInterface.NOTIFY_TURN_INTERVAL:
            return
        pattern = vibration.UNKNOWN
        text = ""
        msgs = [std_msgs.msg.String(), std_msgs.msg.Float32()]
        if turn.turn_type == Turn.Type.Normal:
            if turn.angle <= -180/4*3:
                pattern = vibration.RIGHT_ABOUT_TURN
                text = "right about turn"
            elif turn.angle <= -180/3:
                pattern = vibration.RIGHT_TURN
                text = "right turn"
            elif turn.angle >= 180/4*3:
                pattern = vibration.LEFT_ABOUT_TURN
                text = "left about turn"
            elif turn.angle >= 180/3:
                pattern = vibration.LEFT_TURN
                text = "left turn"
            msgs[0].data = str(Turn.Type.Normal)
            self._activity_log("cabot/turn_type", "Type.Normal")
        elif turn.turn_type == Turn.Type.Avoiding:
            if turn.angle <= 0:
                pattern = vibration.RIGHT_DEV
                text = "slight right"
            if turn.angle >= 0:
                pattern = vibration.LEFT_DEV
                text = "slight left"
            msgs[0].data = str(Turn.Type.Avoiding)
            self._activity_log("cabot/turn_type", "Type.Avoiding")
        msgs[1].data = turn.angle
        self.turn_type_pub.publish(msgs[0])
        self.turn_angle_pub.publish(msgs[1])
        self.last_notify_turn = self._node.get_clock().now()
        self._activity_log("cabot/interface", "turn angle", str(turn.angle))
        self._activity_log("cabot/interface", "notify", text)
        self.vibrate(pattern)
        self.read_aloud_vibration(pattern)

    def notify_human(self, angle=0):
        vib = vibration.RIGHT_DEV
        if angle > 0:
            vib = vibration.LEFT_DEV

        self._activity_log("cabot/interface", "human")
        self.vibrate(pattern=vib)
        self.speak(i18n.localized_string("AVOIDING_A_PERSON"))

    def have_arrived(self, goal):
        raise RuntimeError("Should no use this func")
        name = goal.goal_name_pron
        desc = goal.goal_description

        if name:
            if desc:
                self.speak(i18n.localized_string("YOU_HAVE_ARRIVED_WITH_NAME_AND_DESCRIPTION").format(name, desc))
            else:
                self.speak(i18n.localized_string("YOU_HAVE_ARRIVED_WITH_NAME").format(name))
        else:
            self.speak(i18n.localized_string("YOU_HAVE_ARRIVED"))
        self._activity_log("cabot/interface", "navigation", "arrived")

    def approaching_to_poi(self, poi=None):
        statement = poi.approaching_statement()
        if statement:
            self.speak(statement)
            self._activity_log("cabot/interface", "poi", "approaching")

    def approached_to_poi(self, poi=None):
        statement = poi.approached_statement()
        if statement:
            self.speak(statement)
            self._activity_log("cabot/interface", "poi", "approached")

    def passed_poi(self, poi=None):
        statement = poi.passed_statement()
        if statement:
            self.speak(statement)
            self._activity_log("cabot/interface", "poi", "passed")

    def could_not_get_current_location(self):
        self.speak(i18n.localized_string("COULD_NOT_GET_CURRENT_LOCATION"))

    def enter_goal(self, goal):
        pass

    def exit_goal(self, goal):
        pass

    def announce_social(self, message: SNMessage):
        self._activity_log("cabot/interface", message.type.name, message.code.name)
        self.speak(i18n.localized_string(message.code.name))

    def request_sound(self, sound: SNMessage):
        self._activity_log("cabot/interface", sound.type.name, sound.code.name)
        e = NavigationEvent("sound", sound.code.name)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self.event_pub.publish(msg)

    def set_pause_control(self, flag):
        self._activity_log("cabot/interface", "pause_control", str(flag))
        if flag:
            self.speak(i18n.localized_string("PAUSE_CONTROL"))

    def please_call_elevator(self, pos):
        self._activity_log("cabot/interface", "navigation", "elevator button")
        if pos:
            self.speak(i18n.localized_string("CALL_ELEVATOR_PLEASE_ON_YOUR",
                                             i18n.localized_string(pos)))
        else:
            self.speak(i18n.localized_string("CALL_ELEVATOR_PLEASE"))

    def elevator_opening(self):
        self._activity_log("cabot/interface", "navigation", "elevator opening")
        self.vibrate(vibration.FRONT)
        self.speak(i18n.localized_string("ELEVATOR_IS_OPENING"))

    def floor_changed(self, floor):
        self._activity_log("cabot/interface", "navigation", "floor_changed")
        self.speak(i18n.localized_string("GETTING_OFF_THE_ELEVATOR"))

    def queue_start_arrived(self):
        self._activity_log("cabot/interface", "queue", "start arrived")
        self.speak(i18n.localized_string("GOING_TO_GET_IN_LINE"))

    def queue_proceed(self):
        self._activity_log("cabot/interface", "queue", "proceed")
        self.vibrate(vibration.FRONT)

    def please_pass_door(self):
        self._activity_log("cabot/interface", "navigation", "manual door")
        self.speak(i18n.localized_string("DOOR_POI_USER_ACTION"))

    def door_passed(self):
        self._activity_log("cabot/interface", "navigation", "door passed")
        self.speak(i18n.localized_string("DOOR_POI_PASSED"))

    def please_follow_behind(self):
        self._activity_log("cabot/interface", "navigation", "please_follow_behind")
        self.speak(i18n.localized_string("FOLLOW_BEHIND_PLEASE_NARROW"))

    def please_return_position(self):
        self._activity_log("cabot/interface", "navigation", "please_return_position")
        self.speak(i18n.localized_string("RETURN_TO_POSITION_PLEASE"))
