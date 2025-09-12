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


class NavigationInterface(object):
    def activity_log(self, category="", text="", memo=""):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def change_language(self, lang):
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

    def set_pause_control(self, pause: bool):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def in_preparation(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def pause_navigation(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def pausing_navigation(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def cancel_navigation(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def resume_navigation(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")
