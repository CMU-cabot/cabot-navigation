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

import std_msgs.msg
from cabot_ui.plugin import NavcogMapPlugin


class PhoneMenu(NavcogMapPlugin):
    name = "Phone Menu Plugin"

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        self.event_pub = node.create_publisher(std_msgs.msg.String, "/cabot/event", 1)

    def menu_callback(self, feedback):
        self.logger.info(F"{feedback}")
        msg = std_msgs.msg.String()
        msg.data = "navigation;phone;" + feedback.marker_name
        self.event_pub.publish(msg)

    def init_menu(self, menu_handler):
        self.navigate_menu = menu_handler.insert("Navigate to Here (phone)", callback=self.menu_callback)
