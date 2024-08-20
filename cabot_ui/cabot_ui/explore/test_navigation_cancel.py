import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String
import argparse
from mf_localization_msgs.msg import MFLocalizeStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
import sys
import numpy as np
import tty, termios


class CancelNode:
    def __init__(self, node):
        self.node = node
        self.cancel_pub = self.node.create_publisher(String, "/cabot/event", 10)
        self.event_sub = self.node.create_subscription(String, "/cabot/event", self.event_callback, 10)
        self.logger = self.node.get_logger()
        self.state = ""

        self.timer = self.node.create_timer(1.0, self.publish_state)
    
    def event_callback(self, msg):
        if msg.data == "navigation_startchat":
            self.logger.info("Received startchat event")
            self.state = "navigation;cancel"
    
    def run(self):
        self.logger.info("Cancel node started")
        rclpy.spin(self.node)
    
    def publish_state(self):
        if self.state == "navigation;cancel":
            msg = String()
            msg.data = "navigation;cancel"
            self.cancel_pub.publish(msg)
            self.logger.info(f"State published: {msg.data}")
            self.state = ""
        else:
            pass



def main():
    # set log level to debug
    # rclpy.logging._root_logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
    if not rclpy.ok():
        rclpy.init()

    print("Hello from cancel node")
    node = Node("cancel_node", start_parameter_services=False)
    rcl_publisher = CancelNode(node=node)
    try:
        rcl_publisher.run()
    except SystemExit as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
