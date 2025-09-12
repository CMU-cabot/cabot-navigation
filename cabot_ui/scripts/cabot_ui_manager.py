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

from importlib.metadata import entry_points
import signal
import sys
import threading
import traceback
import yaml

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import std_msgs.msg
import std_srvs.srv

import cabot_common.button
from cabot_common.event import BaseEvent, ButtonEvent, ClickEvent, HoldDownEvent
from cabot_ui.node_manager import NodeManager
from cabot_ui.event import MenuEvent, NavigationEvent, ExplorationEvent
from cabot_ui.menu import Menu
from cabot_ui.interface import UserInterface
from cabot_ui.plugin import NavigationInterface
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus


class NavigationPlugins():
    def __init__(self, plugins, node_manager, delegate):
        eps = entry_points(group="cabot_ui.plugins")
        self._plugins = []

        for name in plugins:
            for ep in eps:
                if ep.name != name:
                    continue
                CaBotRclpyUtil.info(f"Loading plugin {ep.name} {ep.value}")
                cls = ep.load()
                instance = cls(node_manager)
                instance.delegate = delegate
                self._plugins.append(instance)

    @property
    def plugins(self):
        return self._plugins

    @property
    def ready(self):
        for p in self._plugins:
            if not p.ready:
                return False
        return True

    def process_event(self, event):
        for p in self._plugins:
            if p.process_event(event):
                break


class CabotUIManager(NavigationInterface, object):
    def __init__(self, node_manager):
        self._node = node_manager.get_node()
        self._logger = self._node.get_logger()
        CaBotRclpyUtil.initialize(self._node)

        self.in_navigation = False

        self._handle_button_mapping = self._node.declare_parameter('handle_button_mapping', 2).value
        if self._handle_button_mapping == 2:
            self._event_mapper = EventMapper2()
        else:
            self._event_mapper = EventMapper1()
        self._event_mapper.delegate = self
        self._interface = UserInterface(self._node)
        self._interface.delegate = self

        plugins = self._node.declare_parameter('navigation_plugins', ["navigation", "feature", "description", "speaker"]).value
        self._navigation_plugins = NavigationPlugins(plugins, node_manager, self)
        # self._exploration = Exploration()
        # self._exploration.delegate = self

        self._retry_count = 0

        self._node.create_subscription(std_msgs.msg.String, "/cabot/event", self._event_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self._eventPub = self._node.create_publisher(std_msgs.msg.String, "/cabot/event", 10, callback_group=MutuallyExclusiveCallbackGroup())

        self.updater = Updater(self._node)

        def manager_status(stat):
            if self._navigation_plugins.ready:
                stat.summary(DiagnosticStatus.OK, "Ready")
            else:
                stat.summary(DiagnosticStatus.ERROR, "Not ready")
            return stat
        self.updater.add(FunctionDiagnosticTask("UI Manager", manager_status))

        self.create_menu_timer = self._node.create_timer(1.0, self.create_menu, callback_group=MutuallyExclusiveCallbackGroup())

    def create_menu(self):
        try:
            self.create_menu_timer.cancel()
            menu_file = self._node.declare_parameter('menu_file', '').value
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

    def change_language(self, lang):
        self._interface.change_language(lang)

    def i_am_ready(self):
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

    def have_arrived(self, goal):
        # do not read arrival message from robot
        # self._logger.info("delegate have_arrived called")
        # self._interface.have_arrived(goal)
        pass

    def have_completed(self):
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

    def set_pause_control(self, pause: bool):
        self._interface.set_pause_control(pause)

    def in_preparation(self):
        self._interface.in_preparation()

    def pause_navigation(self):
        self._interface.pause_navigation()

    def pausing_navigation(self):
        self._interface.pausing_navigation()

    def cancel_navigation(self):
        self._interface.cancel_navigation()

    def resume_navigation(self):
        self._interface.resume_navigation()

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

    def menu_selected(self, menu):
        self._logger.debug(F"menu_selected, {menu.identifier}, {menu.type}")
        if menu.identifier == "destination_menu":
            event = NavigationEvent("destination", menu.value.value)
            self.process_event(event)

    # event delegate method
    def process_event(self, event):
        '''
        all events go through this method
        '''
        # self._logger.info(f"process_event {str(event)}")

        self._event_mapper.push(event)
        self._process_navigation_event(event)
        self._process_exploration_event(event)

    def _process_navigation_event(self, event):
        if event.type != NavigationEvent.TYPE:
            return
        self._logger.info(f"_process_navigation_event {event}")

        if self._navigation_plugins.process_event(event):
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

    def _process_exploration_event(self, event):
        if event.type != ExplorationEvent.TYPE:
            return

        if event.subtype == "start":
            self._interface.start_exploration()
            self._exploration.start_exploration()


class EventMapper1(object):
    def __init__(self):
        self.description_duration = 0

    def push(self, event):
        if event.type != ButtonEvent.TYPE and event.type != ClickEvent.TYPE and \
           event.type != HoldDownEvent.TYPE:
            return

        # simplify the control
        mevent = self.map_button_to_navigation(event)
        if mevent:
            self.delegate.process_event(mevent)

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
        return None


class EventMapper2(object):
    def __init__(self):
        self.button_hold_down_duration = 0
        self.button_hold_down_duration_prev = 0

    def push(self, event):
        if event.type not in {ButtonEvent.TYPE, ClickEvent.TYPE, HoldDownEvent.TYPE}:
            return

        # simplify the control
        mevent = self.map_button_to_navigation(event)
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
    # node.destroy_node()
    node_manager.join()
    sys.exit()


signal.signal(signal.SIGINT, receiveSignal)


if __name__ == "__main__":
    rclpy.init()

    node_manager = NodeManager()
    ui_manager = CabotUIManager(node_manager)
    node_manager.join()
    node_manager.get_node().get_logger().info("cabot_ui_manager terminated")

