#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
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

import tty
import termios
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import std_msgs.msg

import cabot_common.event
import cabot_common.button
from cabot_common.util import setInterval

interval = 0.25
NKeys = 12
lastUp = [None]*NKeys
upCount = [0]*NKeys
btnDwn = [False]*NKeys
hold_active = [False]*NKeys
node = None
eventPub = None


def getchar():
    # Returns a single character from standard input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ord(ch) == 3:
        quit()  # handle ctrl+C
    return ch


button = -1
hold_duration = 0


def reset_click_state(index):
    lastUp[index] = None
    upCount[index] = 0
    hold_active[index] = False


@setInterval(0.01)
def process():
    global button, hold_duration
    now = node.get_clock().now()
    event = None
    temp = [False]*NKeys
    for i in range(0, len(temp)):
        temp[i] = (button == i)

        if temp[i] and not btnDwn[i]:
            event = cabot_common.event.ButtonEvent(button=i, up=False)
            btnDwn[i] = True

        if not temp[i] and btnDwn[i]:
            event = cabot_common.event.ButtonEvent(button=i, up=True)
            btnDwn[i] = False
            if hold_active[i]:
                reset_click_state(i)
            else:
                upCount[i] += 1
                lastUp[i] = now

        if lastUp[i] is not None and now - lastUp[i] > Duration(seconds=interval):
            if not hold_active[1]:
                event = cabot_common.event.ClickEvent(buttons=i, count=upCount[i])
            reset_click_state(i)

        # node.get_logger().info(upCount)
        # node.get_logger().info(temp)
        # node.get_logger().info(btnDwn)

    if button != -1 and hold_duration > 0:
        hold_active[button] = True
        event = cabot_common.event.HoldDownEvent(holddown=button, duration=hold_duration)
        hold_duration = 0

    button = -1
    if event is not None:
        node.get_logger().info(str(event)+"\r")
        msg = std_msgs.msg.String()
        msg.data = str(event)
        eventPub.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = Node("cabot_keyboard_node")
    eventPub = node.create_publisher(std_msgs.msg.String, "/cabot/event", 1)
    process()

    '''
    node.get_logger().info("type 'j', 'k', or 'l' for 'up', 'center', 'down' buttons")
    while rclpy.ok:
        key = ord(getchar())
        button = -1
        if key == 106: #j
            button = cabot_common.button.BUTTON_NEXT
        elif key == 107: #k
            button = cabot_common.button.BUTTON_SELECT
        elif key == 108: #l
            button = cabot_common.button.BUTTON_PREV
    '''
    node.get_logger().info("type 'cursor keys' for 'up', 'down', 'left', and 'right' buttons")
    node.get_logger().info("type '1-5' for hold duration (seconds), then type 'cursor keys'")
    while rclpy.ok:
        key = ord(getchar())
        button = -1
        if key in range(49, 54):  # Numbers 1 to 5 for hold duration
            hold_duration = key - 48
            node.get_logger().info(F"Set hold duration to {hold_duration} seconds")
        if key == 65:    # arrow up
            button = cabot_common.button.BUTTON_UP
        elif key == 66:  # arrow down
            button = cabot_common.button.BUTTON_DOWN
        elif key == 67:  # arrow right
            button = cabot_common.button.BUTTON_RIGHT
        elif key == 68:  # arrow left
            button = cabot_common.button.BUTTON_LEFT

        if button > 0:
            node.get_logger().info(F"button {button}")
        else:
            node.get_logger().info(F"key {key}")

    rclpy.spin(node)
