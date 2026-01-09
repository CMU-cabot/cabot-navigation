#!/usr/bin/env python3

# Copyright (c) 2020, 2025  Carnegie Mellon University
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

import os
import sys
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from cabot_ui import geoutil, geojson, datautil
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarkerControl, InteractiveMarker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import Point
import std_msgs.msg

from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.plugin import NavcogMapPlugins
from mf_localization_msgs.msg import MFLocalizeStatus
from cabot_msgs.msg import Anchor


class NavCogMap(Node):
    def __init__(self):
        super(NavCogMap, self).__init__('navcog_map')
        CaBotRclpyUtil.initialize(self)

        plugins = os.environ.get('CABOT_UI_PLUGINS', "navigation,feature,description,speaker").split(",")
        self._plugins = NavcogMapPlugins(plugins, self)

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.current_floor_sub = self.create_subscription(std_msgs.msg.Int64, "/current_floor", self.current_floor_callback, qos)
        self.raw_current_floor = 0
        self.current_floor = 0
        self.meters_per_floor = 5
        self.last_floor = None

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.localize_status_sub = self.create_subscription(MFLocalizeStatus, "/localize_status", self.localize_status_callback, transient_local_qos)
        self.localize_status = MFLocalizeStatus.UNKNOWN

        self.anchor_pub = self.create_publisher(Anchor, "/anchor", transient_local_qos)

        self.current_floor = self.declare_parameter("initial_floor", 0).value
        self.map_frame = self.declare_parameter("map_frame", "map_global").value

        anchor = None
        anchor_file = self.declare_parameter("anchor_file", "").value
        self.get_logger().info(F"Anchor file is {anchor_file}")
        anchor_tmp = geoutil.get_anchor(anchor_file=anchor_file)
        if anchor_tmp is not None:
            anchor = anchor_tmp
            anchor_msg = Anchor()
            anchor_msg.lat = float(anchor.lat)
            anchor_msg.lng = float(anchor.lng)
            anchor_msg.rotate = float(anchor.rotate)
            self.anchor_pub.publish(anchor_msg)
        else:
            self.get_logger().info(F"Could not load anchor_file {anchor_file}")

        self.datautil = datautil.DataUtil(self)
        self.datautil.set_anchor(anchor)
        self.datautil.init_by_server()
        self.datautil.set_anchor(anchor)
        self.data_ready = True
        self.get_logger().info("data ready")
        self.get_logger().info(F"initial floor = {self.current_floor}")

        self.server = InteractiveMarkerServer(self, "menu")
        self.menu_handler = MenuHandler()
        self._plugins.init_menu(self.menu_handler)

        self.timer = self.create_timer(1, self.check_update)

    def current_floor_callback(self, msg):
        self.raw_current_floor = msg.data
        self.current_floor = msg.data + 1 if msg.data >= 0 else msg.data

    def localize_status_callback(self, msg):
        self.localize_status = msg.status

    def check_update(self):
        if not self.data_ready:
            self.get_logger().info("data is not ready")
            return
        if self.localize_status != MFLocalizeStatus.TRACKING:
            self.get_logger().info("localization is not tracking")
            return
        if self.current_floor == self.last_floor:
            self.get_logger().info("same floor")
            return
        self.server.clear()
        if not self.visualize_features(self.datautil.features, self.datautil.node_map):
            self.get_logger().info("fail to visualize features")
            return
        self.server.applyChanges()
        self.last_floor = self.current_floor

    def visualize_features(self, features, node_map):
        if features is None:
            self.get_logger().error("navcog_map: features is None")
            return False

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        vis_pub = self.create_publisher(MarkerArray, "links", qos)

        array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "links"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for f in features:
            if not isinstance(f, geojson.Link) or f.floor != self.current_floor:
                continue
            s = Point()
            s.x = f.start_node.local_geometry.x
            s.y = f.start_node.local_geometry.y
            s.z = self.raw_current_floor * self.meters_per_floor + 0.1
            e = Point()
            e.x = f.end_node.local_geometry.x
            e.y = f.end_node.local_geometry.y
            e.z = self.raw_current_floor * self.meters_per_floor + 0.1
            marker.points.append(s)
            marker.points.append(e)

        array.markers.append(marker)

        for k, f in node_map.items():
            if isinstance(f, geojson.Landmark):
                continue
            if f.floor != self.current_floor:
                continue

            control = InteractiveMarkerControl()
            control.interaction_mode = InteractiveMarkerControl.BUTTON
            control.always_visible = True

            marker = InteractiveMarker()
            marker.header.frame_id = self.map_frame
            marker.name = k
            marker.pose.position.x = f.local_geometry.x
            marker.pose.position.y = f.local_geometry.y
            marker.pose.position.z = self.raw_current_floor * self.meters_per_floor + 0.1
            marker.scale = 1.0

            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.a = 1.0
            sphere.color.r = 0.0
            sphere.color.g = 0.0
            sphere.color.b = 1.0

            control.markers.append(sphere)
            marker.controls.append(control)
            self.server.insert(marker, feedback_callback=self.process_feedback)
            self.menu_handler.apply(self.server, marker.name)

        vis_pub.publish(array)
        return True

    def process_feedback(self, feedback):
        self.get_logger().info(F"{feedback}")


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    node.destroy_node()
    sys.exit()


signal.signal(signal.SIGINT, receiveSignal)


if __name__ == "__main__":
    rclpy.init()
    node = NavCogMap()
    rclpy.spin(node)
