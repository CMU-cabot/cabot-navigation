#!/usr/bin/env python3

# Copyright (c) 2022  Carnegie Mellon University
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

import rclpy
from rclpy.time import Duration
from rclpy.node import Node
import tf2_ros
import nav_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import cabot_msgs.msg
import people_msgs.msg
import threading

from cabot_ui.stop_reasoner import StopReasonFilter, StopReasoner
from cabot_ui.event import NavigationEvent
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


reasoner = None
stop_reason_pub = None
event_pub = None


def main():
    global reasoner, stop_reason_pub, event_pub, stop_reason_filter
    ODOM_TOPIC = "/cabot/odom_raw"
    EVENT_TOPIC = "/cabot/event"
    CMD_VEL_TOPIC = "/cmd_vel"
    PEOPLE_SPEED_TOPIC = "/cabot/people_speed"
    TF_SPEED_TOPIC = "/cabot/tf_speed"
    TOUCH_SPEED_TOPIC = "/cabot/touch_speed_switched"
    RECEIVED_GLOBAL_PLAN = "/received_global_plan"
    LOCAL_PREFIX = "/local"
    REPLAN_REASON_TOPIC = "/replan_reason"
    CURRENT_FRAME_TOPIC = "/current_frame"

    rclpy.init()
    node = Node("stop_reason_node")
    CaBotRclpyUtil.initialize(node)
    tf_buffer = tf2_ros.Buffer(Duration(seconds=10), node)
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)  # noqa: #841
    reasoner = StopReasoner(tf_buffer)

    stop_reason_pub = node.create_publisher(cabot_msgs.msg.StopReason, "/stop_reason", 10)
    event_pub = node.create_publisher(std_msgs.msg.String, "/cabot/event", 10)

    announce_no_touch = node.declare_parameter("announce_no_touch", False).value
    ignore_list = ["NO_NAVIGATION", "NOT_STOPPED", "NO_TOUCH", "STOPPED_BUT_UNDER_THRESHOLD"]
    if announce_no_touch:
        ignore_list = ["NO_NAVIGATION", "NOT_STOPPED", "STOPPED_BUT_UNDER_THRESHOLD"]
    stop_reason_filter = StopReasonFilter(ignore_list)

    node.create_subscription(nav_msgs.msg.Odometry, ODOM_TOPIC, odom_callback, 10)
    node.create_subscription(std_msgs.msg.String, EVENT_TOPIC, event_callback, 10)
    node.create_subscription(geometry_msgs.msg.Twist, CMD_VEL_TOPIC, cmd_vel_callback, 10)
    node.create_subscription(std_msgs.msg.Float32, PEOPLE_SPEED_TOPIC, people_speed_callback, 10)
    node.create_subscription(std_msgs.msg.Float32, TF_SPEED_TOPIC, tf_speed_callback, 10)
    node.create_subscription(std_msgs.msg.Float32, TOUCH_SPEED_TOPIC, touch_speed_callback, 10)
    node.create_subscription(nav_msgs.msg.Path, RECEIVED_GLOBAL_PLAN, global_plan_callback, 10)
    node.create_subscription(nav_msgs.msg.Path, LOCAL_PREFIX+RECEIVED_GLOBAL_PLAN, global_plan_callback, 10)
    node.create_subscription(people_msgs.msg.Person, REPLAN_REASON_TOPIC, replan_reason_callback, 10)
    node.create_subscription(std_msgs.msg.String, CURRENT_FRAME_TOPIC, current_frame_callback, 10)

    node.create_timer(0.1, timer_callback)

    rclpy.spin(node)


def timer_callback():
    update()


lock = threading.Lock()


def update():
    with lock:
        global prev_code, prev_duration
        (duration, code) = reasoner.update()
        stop_reason_filter.update(duration, code)
        (duration, code) = stop_reason_filter.event()
        print(duration, code)
        if code:
            msg = cabot_msgs.msg.StopReason()
            msg.header.stamp = CaBotRclpyUtil.now().to_msg()
            msg.reason = code.name
            msg.duration = duration
            stop_reason_pub.publish(msg)
        (duration, code) = stop_reason_filter.summary()
        if code:
            event = NavigationEvent("stop-reason", code.name)
            msg = std_msgs.msg.String()
            msg.data = str(event)
            event_pub.publish(msg)
            CaBotRclpyUtil.info(F"{CaBotRclpyUtil.now().nanoseconds/1e9}, {code.name}, {duration}")
        stop_reason_filter.conclude()


def odom_callback(msg):
    reasoner.input_odom(msg)


def event_callback(msg):
    reasoner.input_event(msg)


def global_plan_callback(msg):
    reasoner.input_global_plan(msg)


def cmd_vel_callback(msg):
    reasoner.input_cmd_vel(msg)


def people_speed_callback(msg):
    reasoner.input_people_speed(msg)


def tf_speed_callback(msg):
    pass


def touch_speed_callback(msg):
    reasoner.input_touch_speed(msg)


def replan_reason_callback(msg):
    reasoner.input_replan_reason(msg)


def current_frame_callback(msg):
    reasoner.input_current_frame(msg)


if __name__ == "__main__":
    main()
