#!/usr/bin/env python3

from rclpy.node import Node
import rclpy

from cabot_ui.explore.test_loop import main as main_loop
from cabot_common import vibration
import std_msgs.msg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ExplorationMainLoop(Node):
    def __init__(self):
        super().__init__('exploration_main_loop')
        self.logger = self.get_logger()
        self.logger.info("ExplorationMainLoop node started")

        log_dir = self.declare_parameter('log_dir').value
        self.logger.info(f"log_dir: {log_dir}")
        self.note_pub = self.create_publisher(std_msgs.msg.Int8, "/cabot/notification", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.event_pub = self.create_publisher(std_msgs.msg.String, "/cabot/event", 10)
        self.camera_ready_sub = self.create_subscription(std_msgs.msg.Bool, "/cabot/camera_ready", self.camera_ready_callback, 10)

        dist_filter = self.declare_parameter('dist_filter').value
        is_sim = self.declare_parameter('is_sim').value

        self.event_pub.publish(std_msgs.msg.String(data="explore_main_loop_start")) # TODO: move this inside main_loop
        self.camera_ready = False

        main_loop(
            dist_filter=dist_filter,
            forbidden_area_filter=False,
            trajectory_filter=False,
            auto=True,
            use_image=False,
            log_dir=log_dir,
            sim=is_sim,
            keyboard=False,
            debug=False
        )

    def vibrate(self, pattern=vibration.UNKNOWN): # TODO: move this inside main_loop
        """
        if pattern == vibration.FRONT:
        elif pattern == vibration.RIGHT_TURN:
        elif pattern == vibration.LEFT_TURN:
        """
        self.logger.info(f"Vibrating with pattern: {pattern}")
        msg = std_msgs.msg.Int8()
        msg.data = pattern
        self.note_pub.publish(msg)

    def camera_ready_callback(self, msg):
        self.camera_ready = msg.data
        self.logger.info(f"Camera ready at exploration main loop: {self.camera_ready}")

def main(args=None):
    rclpy.init(args=args)

    node = ExplorationMainLoop()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print(f"Hello from {__file__}")
    main()