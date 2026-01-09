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

from abc import ABC, abstractmethod
from importlib.metadata import entry_points
import traceback

from cabot_ui.node_manager import NodeManager
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class EventMapperPlugin(ABC):
    name: str

    def __init__(self, callback):
        pass

    @abstractmethod
    def push(self, event) -> None:
        pass


class EventMapperPlugins():
    def __init__(self, plugins, callback):
        eps = entry_points(group="event_mapper.plugins")
        self._plugins = []

        for name in plugins:
            for ep in eps:
                if ep.name != name:
                    continue
                CaBotRclpyUtil.info(f"Loading plugin {ep.name} {ep.value}")
                cls = ep.load()
                instance = cls(callback)
                self._plugins.append(instance)

    @property
    def plugins(self):
        return self._plugins

    def push(self, event):
        for p in self._plugins:
            try:
                p.push(event)
            except:  # noqa
                self._logger.error(traceback.format_exc())


class NavcogMapPlugin(ABC):
    name: str

    def __init__(self):
        pass

    @abstractmethod
    def init_menu(self, menu_handler) -> None:
        pass


class NavcogMapPlugins():
    def __init__(self, plugins, node):
        self._node = node
        self._logger = self._node.get_logger()
        eps = entry_points(group="navcog_map.plugins")
        self._plugins = []

        for name in plugins:
            for ep in eps:
                if ep.name != name:
                    continue
                CaBotRclpyUtil.info(f"Loading plugin {ep.name} {ep.value}")
                cls = ep.load()
                instance = cls(self._node)
                self._plugins.append(instance)

    @property
    def plugins(self):
        return self._plugins

    def init_menu(self, menu_handler) -> None:
        for plugin in self._plugins:
            plugin.init_menu(menu_handler)


class NavigationPlugin(ABC):
    name: str

    def __init__(self, node_manager: NodeManager,
                 datautil_instance=None, anchor_file='', wait_for_action=True):
        self._ready = False
        pass

    @property
    def ready(self) -> bool:
        return self._ready

    @abstractmethod
    def process_event(self, event) -> None:
        pass


class NavigationPlugins():
    def __init__(self, plugins, node_manager, delegate):
        self._node = node_manager.get_node()
        self._logger = self._node.get_logger()
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
            try:
                result = p.process_event(event)
                self._logger.info(f"process_event {result=}, {type(p)=}, {str(event)}")
                if result:
                    break
            except:  # noqa
                self._logger.error(traceback.format_exc())
