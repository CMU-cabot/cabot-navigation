#!/usr/bin/env python3

from mf_localization_msgs.msg import MFLocalizeStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class CaBotExplorationNode(Node):
    def __init__(self):
        super().__init__("cabot_exploration_node")
        self.get_logger().info("cabot exploration can be implemented here")

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.localize_status_pub = self.create_publisher(MFLocalizeStatus, "/localize_status", transient_local_qos)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.timer.cancel()
        msg = MFLocalizeStatus()
        msg.status = MFLocalizeStatus.TRACKING
        self.localize_status_pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()

    node = CaBotExplorationNode()
    rclpy.spin(node)
