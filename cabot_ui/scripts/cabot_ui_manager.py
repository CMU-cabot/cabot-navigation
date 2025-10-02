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
import traceback
import types
import yaml

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import std_msgs.msg

from cabot_common.event import BaseEvent
from cabot_ui.node_manager import NodeManager
from cabot_ui.event import NavigationEvent, ExplorationEvent
from cabot_ui.menu import Menu
from cabot_ui.interface import UserInterface
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.plugin import NavigationPlugins, EventMapperPlugins

from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus


class CabotUIManager(object):
    def __init__(self, node_manager):
        self._node = node_manager.get_node()
        self._logger = self._node.get_logger()
        CaBotRclpyUtil.initialize(self._node)

        self.in_navigation = False

        self._handle_button_mapping = self._node.declare_parameter('handle_button_mapping', 2).value
        self._event_mapper_plugins = EventMapperPlugins([f"event_mapper{self._handle_button_mapping}"], callback=self.process_event)

        self._interface = UserInterface(self._node)

        # workaround to make user_speed method to access the speed_menu instance
        def user_speed(self):
            return self.speed_menu.value
        self._interface.user_speed = types.MethodType(user_speed, self)

        plugins = os.environ.get('CABOT_UI_PLUGINS', "navigation,feature,description,speaker").split(",")
        self._navigation_plugins = NavigationPlugins(plugins, node_manager, self._interface)
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

        self._event_mapper_plugins.push(event)
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


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    # node.destroy_node()
    node_manager.join()
    sys.exit()


signal.signal(signal.SIGINT, receiveSignal)


if __name__ == "__main__":
    rclpy.init()

    node_manager = NodeManager()
    try:
        ui_manager = CabotUIManager(node_manager)
        # FIXME
        from cabot_ui.elevator_controller import elevator_controller
        elevator_controller._client.subscribe_settings()
    except:  # noqa: #722
        node_manager.get_node().get_logger().error(traceback.format_exc())
        rclpy.shutdown()
    node_manager.join()
    node_manager.get_node().get_logger().info("cabot_ui_manager terminated")
