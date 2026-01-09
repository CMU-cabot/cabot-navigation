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

from cabot_common.button import BUTTON_LEFT, BUTTON_RIGHT, BUTTON_UP, BUTTON_DOWN, BUTTON_CENTER
from cabot_common.event import HoldDownEvent, ClickEvent, ButtonEvent
from cabot_ui.event import NavigationEvent


class EventMapper2(object):
    def __init__(self, callback):
        self.button_hold_down_duration = 0
        self.button_hold_down_duration_prev = 0
        self.callback = callback

    def push(self, event):
        if event.type not in {ButtonEvent.TYPE, ClickEvent.TYPE, HoldDownEvent.TYPE}:
            return

        # simplify the control
        mevent = self.map_button_to_navigation(event)
        if mevent:
            self.callback(mevent)

    def map_button_to_navigation(self, event):
        if event.type == "button" and not event.down and self.button_hold_down_duration > 0:
            self.button_hold_down_duration = 0
            self.button_hold_down_duration_prev = 0
            return None
        if event.type == "click" and event.count == 1:
            if event.buttons == BUTTON_RIGHT:
                return NavigationEvent(subtype="resume_or_stop_reason")
            if event.buttons == BUTTON_LEFT:
                return NavigationEvent(subtype="toggle_speak_state")
            if event.buttons == BUTTON_UP:
                return NavigationEvent(subtype="description_surround", param=event.count)
            if event.buttons == BUTTON_DOWN:
                return NavigationEvent(subtype="toggle_conversation")
            if event.buttons == BUTTON_CENTER:
                return NavigationEvent(subtype="decision")
        if event.type == "click" and event.count > 1:
            if event.buttons == BUTTON_UP:
                description_length = min(event.count, 3)
                return NavigationEvent(subtype="description_surround", param=description_length)
            if event.buttons == BUTTON_RIGHT:
                return NavigationEvent(subtype="speaker_alert")
        if event.type == HoldDownEvent.TYPE:
            if event.holddown == BUTTON_LEFT and event.duration == 1:
                return NavigationEvent(subtype="pause")
            if event.holddown == BUTTON_LEFT and event.duration == 3:
                return NavigationEvent(subtype="idle")
            if event.holddown in {BUTTON_DOWN, BUTTON_UP}:
                self.button_hold_down_duration = event.duration
                if self.button_hold_down_duration - self.button_hold_down_duration_prev >= 1:
                    self.button_hold_down_duration_prev = self.button_hold_down_duration
                    return NavigationEvent(subtype="speeddown" if event.holddown == BUTTON_DOWN else "speedup")
        return None
