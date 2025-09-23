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


class EventMapper1(object):
    def __init__(self, callback):
        self.description_duration = 0
        self.callback = callback

    def push(self, event):
        if event.type != ButtonEvent.TYPE and event.type != ClickEvent.TYPE and \
           event.type != HoldDownEvent.TYPE:
            return

        # simplify the control
        mevent = self.map_button_to_navigation(event)
        if mevent:
            self.callback(mevent)

    def map_button_to_navigation(self, event):
        if event.type == "button" and not event.down and self.description_duration > 0:
            navigation_event = NavigationEvent(subtype="description", param=self.description_duration)
            self.description_duration = 0
            return navigation_event
        if event.type == "button" and event.down:
            # hook button down to triger pause whenever the left button is pushed
            if event.button == BUTTON_LEFT:
                return NavigationEvent(subtype="pause")
        if event.type == "click" and event.count == 1:
            if event.buttons == BUTTON_RIGHT:
                return NavigationEvent(subtype="resume")
            if event.buttons == BUTTON_UP:
                return NavigationEvent(subtype="speedup")
            if event.buttons == BUTTON_DOWN:
                return NavigationEvent(subtype="speeddown")
            if event.buttons == BUTTON_CENTER:
                return NavigationEvent(subtype="decision")
        if event.type == HoldDownEvent.TYPE:
            if event.holddown == BUTTON_LEFT and event.duration == 3:
                return NavigationEvent(subtype="idle")
            if event.holddown == BUTTON_RIGHT:
                # image description is not triggered here, but when button is released
                self.description_duration = event.duration
        return None
