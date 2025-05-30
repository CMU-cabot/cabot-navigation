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

import signal
import sys
import threading
import traceback
import yaml

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
import rclpy.client
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import std_msgs.msg
import std_srvs.srv

import cabot_common.button
from cabot_common.event import BaseEvent, ButtonEvent, ClickEvent, HoldDownEvent
from cabot_ui.event import MenuEvent, NavigationEvent, ExplorationEvent
from cabot_ui.menu import Menu
from cabot_ui.status import State, StatusManager
from cabot_ui.interface import UserInterface
from cabot_ui.navigation import Navigation, NavigationInterface
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.description import Description

from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus


class CabotUIManager(NavigationInterface, object):
    def __init__(self, node, nav_node, tf_node, srv_node, act_node, soc_node, desc_node):
        self._node = node
        self._logger = self._node.get_logger()
        CaBotRclpyUtil.initialize(self._node)

        self.in_navigation = False
        self.destination = None

        self.reset()

        self._handle_button_mapping = node.declare_parameter('handle_button_mapping', 2).value
        if self._handle_button_mapping == 2:
            self._event_mapper = EventMapper2()
        else:
            self._event_mapper = EventMapper1()
        self._event_mapper.delegate = self
        self._status_manager = StatusManager.get_instance()
        self._status_manager.delegate = self
        self._interface = UserInterface(self._node)
        self._interface.delegate = self
        self._navigation = Navigation(nav_node, tf_node, srv_node, act_node, soc_node)
        self._navigation.delegate = self
        self._description = Description(desc_node)
        # self._exploration = Exploration()
        # self._exploration.delegate = self

        self._retry_count = 0

        self._node.create_subscription(std_msgs.msg.String, "/cabot/event", self._event_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self._eventPub = self._node.create_publisher(std_msgs.msg.String, "/cabot/event", 10, callback_group=MutuallyExclusiveCallbackGroup())

        # request language
        e = NavigationEvent("getlanguage", None)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

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
        e = NavigationEvent("getspeakeraudiofiles", self._interface.audio_files)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

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
        # self._logger.info(f"process_event {str(event)}")

        self._event_mapper.push(event)
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
        if event.type != ExplorationEvent.TYPE:
            return

        if event.subtype == "start":
            self._interface.start_exploration()
            self._exploration.start_exploration()


class EventMapper1(object):
    def __init__(self):
        self._manager = StatusManager.get_instance()
        self.description_duration = 0

    def push(self, event):
        # state = self._manager.state

        if event.type != ButtonEvent.TYPE and event.type != ClickEvent.TYPE and \
           event.type != HoldDownEvent.TYPE:
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
                return NavigationEvent(subtype="decision")
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
    for t in threads:
        t.join()
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
    executors = [MultiThreadedExecutor(),
                 MultiThreadedExecutor(),
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
                except:  # noqa: 722
                    pass
                target_node.destroy_node()
            return _run_node

        thread = threading.Thread(target=run_node(tnode, executor, name))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
