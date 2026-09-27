#!/usr/bin/env python3
"""Repeat CaBot destinations until canceled; never publish velocity commands.

Run explicitly, with two or more MapService node IDs as positional arguments.
Stop with navigation_cancel, /cabot/stop_destination_loop, or SIGINT/SIGTERM.
An external destination or pause also ends the loop without overriding that command.
Use --mode summons for CaBot's existing touch-to-stop summons mode.
"""

import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatusArray
from cabot_msgs.msg import Log


class DestinationLoop(Node):
    def __init__(self, destinations, dwell=3.0, mode='destination'):
        super().__init__('destination_loop')
        self.destinations = destinations
        self.dwell = dwell
        self.mode = mode
        self.index = 0
        self.completed = 0
        self.phase = 'starting'
        self.due = time.monotonic() + 3.0
        self.discovery_deadline = time.monotonic() + 30.0
        self.sent_at = None
        self.accepted = False
        self.echo = None
        self.running = True
        self.active = {}
        self.events = self.create_publisher(String, '/cabot/event', 10)
        self.status = self.create_publisher(
            String, '/cabot/destination_loop/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/cabot/event', self.on_event, 20)
        self.create_subscription(Log, '/cabot/activity_log', self.on_activity, 20)
        for topic in ['/navigate_to_pose/_action/status', '/local/navigate_to_pose/_action/status']:
            self.create_subscription(
                GoalStatusArray, topic,
                lambda msg, key=topic: self.active.update(
                    {key: any(s.status in (1, 2, 3) for s in msg.status_list)}),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/cabot/stop_destination_loop', self.on_stop)
        self.create_timer(0.2, self.tick)
        self.report('Waiting for the current navigation to finish')

    def report(self, message):
        text = f'{self.phase}: {message}; completed legs={self.completed}'
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def stop(self, reason, cancel=False):
        if not self.running:
            return
        self.running = False
        self.phase = 'stopped'
        if cancel and self.sent_at is not None:
            self.events.publish(String(data='navigation_cancel'))
        self.report(reason)

    def on_stop(self, request, response):
        self.stop('Stop service requested', cancel=True)
        response.success = True
        response.message = 'Destination loop stopped; navigation cancel requested'
        return response

    def on_event(self, msg):
        if not self.running:
            return
        value = msg.data
        # CaBot serializes these as navigation_destination;<node>, navigation_arrived, etc.
        if value == self.echo:
            self.echo = None
            return
        kind = value.split(';', 1)[0]
        if kind in ('navigation_cancel', 'navigation_pause', 'navigation_idle'):
            self.stop(f'User command: {kind}', cancel=(kind == 'navigation_idle'))
        elif kind in ('navigation_destination', 'navigation_summons'):
            self.stop('Another destination was selected')
        elif kind == 'navigation_arrived' and self.phase == 'navigating' and self.accepted:
            self.completed += 1
            self.phase = 'dwell'
            self.report(f'Arrived at {self.destinations[self.index]}')
            self.index = (self.index + 1) % len(self.destinations)
            self.due = time.monotonic() + self.dwell

    def on_activity(self, msg):
        if (self.running and self.phase == 'navigating'
                and msg.category == 'cabot/navigation' and msg.text == 'to'
                and msg.memo == self.destinations[self.index]):
            self.accepted = True
            self.report(f'Accepted destination {msg.memo}')

    def tick(self):
        if not self.running:
            return
        now = time.monotonic()
        if self.phase == 'navigating':
            if not self.accepted and now - self.sent_at > 30.0:
                self.stop('Destination was not acknowledged within 30 seconds', cancel=True)
            return
        if now < self.due or any(self.active.values()):
            return
        subscribers = self.get_subscriptions_info_by_topic('/cabot/event')
        if not any(s.node_name == 'cabot_ui_manager' for s in subscribers):
            if self.phase == 'starting' and now < self.discovery_deadline:
                return
            self.stop('CaBot UI manager is unavailable', cancel=True)
            return
        self.phase = 'navigating'
        self.accepted = False
        self.sent_at = now
        self.echo = f'navigation_{self.mode};' + self.destinations[self.index]
        self.events.publish(String(data=self.echo))
        self.report(f'Sent destination {self.destinations[self.index]}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destinations', nargs='+')
    parser.add_argument('--dwell', type=float, default=3.0)
    parser.add_argument('--mode', choices=['destination', 'summons'], default='destination')
    args = parser.parse_args()
    if len(args.destinations) < 2 or args.dwell < 1.0:
        parser.error('Provide at least two destinations and a dwell time of at least 1 second')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DestinationLoop(args.destinations, args.dwell, args.mode)
    signal.signal(signal.SIGINT, lambda *_: node.stop('SIGINT', cancel=True))
    signal.signal(signal.SIGTERM, lambda *_: node.stop('SIGTERM', cancel=True))
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.stop('Process exiting', cancel=True)
        # Allow the cancel/status and service response to leave before DDS teardown.
        end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
