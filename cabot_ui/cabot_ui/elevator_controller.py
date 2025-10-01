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

from datetime import datetime, timedelta
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class ElevatorController:
    def __init__(self):
        self._enabled = True
        self._in_control = False
        self._allow_door_hold = False
        self._last_door_hold = datetime.now()

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool):
        CaBotRclpyUtil.info(f"ElevatorController: enabled = {value}")
        self._enabled = value

    @property
    def in_control(self):
        return self._enabled and self._in_control

    @in_control.setter
    def in_control(self, value: bool):
        CaBotRclpyUtil.info(f"ElevatorController: in_control = {value}")
        self._in_control = value

    def call_elevator(self, from_floor, to_floor):
        if self._enabled:
            CaBotRclpyUtil.info(f"ElevatorController: call elevator from {from_floor} to {to_floor}")
            self._in_control = True
            self._allow_door_hold = False

    def open_door(self, duration=5):
        if self._enabled and self._in_control:
            CaBotRclpyUtil.info(f"ElevatorController: open door for {duration} seconds")
            self._allow_door_hold = True
            self._last_door_hold = datetime.now()

    def hold_door(self, duration=5):
        if self._enabled and self._in_control and self._allow_door_hold:
            now = datetime.now()
            if (now - self._last_door_hold) < timedelta(seconds=duration/2):
                return
            CaBotRclpyUtil.info(f"ElevatorController: hold door for {duration} seconds")
            self._last_door_hold = now

    def close_door(self, after=5):
        if self._enabled or self._in_control:
            CaBotRclpyUtil.info(f"ElevatorController: close door after {after} seconds")
            self._allow_door_hold = False


elevator_controller = ElevatorController()
