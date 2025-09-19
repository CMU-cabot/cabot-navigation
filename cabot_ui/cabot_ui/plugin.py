from abc import ABC, abstractmethod
import inspect

from rclpy.node import Node

from cabot_ui.node_manager import NodeManager
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.social_navigation import SNMessage


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
