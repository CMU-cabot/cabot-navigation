#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: MIT

"""Display controller output; never publish commands or change controller state."""

import math
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


COLORS = {
    'follow': (0.0, 1.0, 1.0),
    'crowdattn': (1.0, 0.55, 0.0),
    'hybrid': (1.0, 0.15, 0.7),
    'sm': (0.7, 0.4, 1.0),
    'mpc': (0.15, 0.45, 1.0),
    'rl': (1.0, 0.85, 0.0),
}

CONTROLLERS = {
    'crowdattn': 'RLFollowPath', 'rl': 'RLFollowPath',
    'hybrid': 'HybridRLFollowPath', 'sm': 'SocialMomentumFollowPath',
    'mpc': 'MPCFollowPath',
}


def command_points(linear, angular, horizon):
    """Constant-twist projection in the robot frame, not a planned trajectory."""
    if not all(math.isfinite(v) for v in (linear, angular, horizon)):
        return []
    points = []
    for i in range(31):
        t = horizon * i / 30
        if abs(angular) < 1e-6:
            x, y = linear * t, 0.0
        else:
            x = linear / angular * math.sin(angular * t)
            y = linear / angular * (1.0 - math.cos(angular * t))
        points.append(Point(x=x, y=y, z=0.12))
    return points


class ControllerVisualizer(Node):
    def __init__(self):
        super().__init__('controller_visualizer')
        self.mode = self.declare_parameter('initial_controller', 'follow').value
        self.width = self.declare_parameter('line_width', 0.08).value
        self.horizon = self.declare_parameter('command_horizon', 1.5).value
        self.timeout = self.declare_parameter('stale_timeout', 0.5).value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value
        self.path = None
        self.command = None
        self.path_time = self.command_time = 0.0
        self.route = self.global_plan = self.local_goal = self.subgoal = None
        self.local_goal_time = self.subgoal_time = 0.0
        self.pub = self.create_publisher(MarkerArray, '/cabot/controller_visualization', 10)
        self.goals_pub = self.create_publisher(MarkerArray, '/cabot/controller_goals', 10)
        mode_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/cabot/controller_mode', self.mode_cb, mode_qos)
        self.create_subscription(Path, '/local_plan', lambda msg: self.path_cb(msg, ('follow',)), 1)
        self.create_subscription(Path, '/control_output_vis',
                                 lambda msg: self.path_cb(msg, ('mpc', 'hybrid', 'sm')), 1)
        self.create_subscription(Twist, '/cmd_vel', self.command_cb, 1)
        self.create_subscription(Path, '/path', self.route_cb, mode_qos)
        self.create_subscription(Path, '/plan', self.global_plan_cb, 1)
        self.create_subscription(Marker, '/local_goal_vis', self.local_goal_cb, 10)
        self.create_subscription(Marker, '/rl_subgoal_vis', self.subgoal_cb, 1)
        self.create_timer(0.1, self.publish_visualization)
        self.create_timer(0.1, self.publish_goals)

    def mode_cb(self, msg):
        if msg.data != self.mode:
            self.path = self.command = None
            self.global_plan = self.local_goal = self.subgoal = None
            self.mode = msg.data
            self.clear()

    def path_cb(self, msg, modes):
        if self.mode in modes:
            self.path = msg
            self.path_time = time.monotonic()

    def command_cb(self, msg):
        self.command = msg
        self.command_time = time.monotonic()

    def route_cb(self, msg):
        self.route = msg
        self.global_plan = None

    def global_plan_cb(self, msg):
        self.global_plan = msg

    def local_goal_cb(self, msg):
        expected = CONTROLLERS.get(self.mode, '') + '/local_goal'
        if msg.ns == expected and msg.action == Marker.ADD:
            self.local_goal = msg
            self.local_goal_time = time.monotonic()

    def subgoal_cb(self, msg):
        if self.mode in ('crowdattn', 'rl', 'hybrid', 'sm') and msg.action == Marker.ADD:
            self.subgoal = msg
            self.subgoal_time = time.monotonic()

    def goal_markers(self, marker_id, frame, point, text, kind, color):
        if not frame or not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
            return []
        marker = self.marker(marker_id, kind, frame)
        marker.pose.position = Point(x=point.x, y=point.y, z=point.z + 0.25)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.28
        marker.color.r, marker.color.g, marker.color.b = color
        label = self.marker(marker_id + 1, Marker.TEXT_VIEW_FACING, frame)
        label.pose.position = Point(x=point.x, y=point.y, z=point.z + 0.6)
        label.scale.z = 0.18
        label.color = marker.color
        label.text = text
        return [marker, label]

    def publish_goals(self):
        if self.mode not in COLORS:
            return
        now = time.monotonic()
        markers = [Marker(action=Marker.DELETEALL)]
        path = self.global_plan if self.global_plan and self.global_plan.poses else self.route
        if path and path.poses:
            label = 'Global goal' if path is self.global_plan else 'Route goal'
            markers += self.goal_markers(10, path.header.frame_id, path.poses[-1].pose.position,
                                         label, Marker.CYLINDER, (1.0, 0.85, 0.0))
        if self.local_goal and now - self.local_goal_time < self.timeout:
            markers += self.goal_markers(20, self.local_goal.header.frame_id, self.local_goal.pose.position,
                                         f'{self.mode} target', Marker.SPHERE, COLORS[self.mode])
        if self.subgoal and now - self.subgoal_time < self.timeout:
            markers += self.goal_markers(30, self.subgoal.header.frame_id, self.subgoal.pose.position,
                                         'RL subgoal', Marker.CUBE, (1.0, 0.25, 1.0))
        self.goals_pub.publish(MarkerArray(markers=markers))

    def marker(self, marker_id, kind, frame):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.mode
        marker.id, marker.type = marker_id, kind
        marker.pose.orientation.w = 1.0
        marker.lifetime.nanosec = 400000000
        marker.color.r, marker.color.g, marker.color.b = COLORS[self.mode]
        marker.color.a = 1.0
        return marker

    def clear(self):
        self.pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def publish_visualization(self):
        now = time.monotonic()
        if self.mode not in COLORS:
            self.clear()
            return
        path_fresh = self.path is not None and now - self.path_time < self.timeout
        command_fresh = self.command is not None and now - self.command_time < self.timeout
        points = []
        frame = self.robot_frame
        description = f'command projection ({self.horizon:g} s)'
        if path_fresh and self.path.header.frame_id and len(self.path.poses) >= 2:
            frame = self.path.header.frame_id
            points = [Point(x=p.pose.position.x, y=p.pose.position.y,
                            z=p.pose.position.z + 0.12) for p in self.path.poses]
            description = 'local trajectory'
        elif command_fresh:
            points = command_points(self.command.linear.x, self.command.angular.z, self.horizon)
        if not points or not all(math.isfinite(v) for p in points for v in (p.x, p.y, p.z)):
            self.clear()
            return

        # Dark outline keeps the colored line visible over costmaps.
        outline = self.marker(0, Marker.LINE_STRIP, frame)
        outline.color.r = outline.color.g = outline.color.b = 0.05
        outline.scale.x = self.width + 0.045
        outline.points = [Point(x=p.x, y=p.y, z=p.z - 0.015) for p in points]
        line = self.marker(1, Marker.LINE_STRIP, frame)
        line.scale.x = self.width
        line.points = points
        endpoint = self.marker(2, Marker.SPHERE, frame)
        endpoint.pose.position = points[-1]
        endpoint.scale.x = endpoint.scale.y = endpoint.scale.z = self.width * 2.0
        label = self.marker(3, Marker.TEXT_VIEW_FACING, self.robot_frame)
        label.pose.position.y = 0.65
        label.pose.position.z = 0.6
        label.scale.z = 0.16
        label.text = f'{self.mode} | {description}'
        markers = [outline, line, endpoint, label]
        if command_fresh and math.isfinite(self.command.angular.z) and abs(self.command.angular.z) > 0.05:
            # Show turning even when forward velocity is zero.
            turn = self.marker(4, Marker.LINE_STRIP, self.robot_frame)
            turn.scale.x = self.width * 0.6
            angle = max(-math.pi, min(math.pi, self.command.angular.z * self.horizon))
            turn.points = [Point(x=0.35 * math.cos(angle * i / 20),
                                 y=0.35 * math.sin(angle * i / 20), z=0.16) for i in range(21)]
            markers.append(turn)
        self.pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = ControllerVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
