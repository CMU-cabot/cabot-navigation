import psutil
import argparse
import datetime
import os, sys
import random
import numpy as np
import time
from typing import List, Dict, Union, Optional, Set, Any

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String
from std_srvs.srv import Trigger

from .test_map import main as get_next_point
from .test_explore import main as explore
from .test_image import main as generate_from_images
from .test_speak import speak_text
from .test_state_input import StateInput


class CabotQueryNode(Node):
    """
    subscribe /cabot/user_query topic
    """
    def __init__(self, candidates: List[str]):
        super().__init__("cabot_query_node")
        self.logger = self.get_logger()
        self.logger.info("CabotQueryNode initialized; waiting for /cabot/user_query topic with 'data: direction;front' format")
        self.query_sub = self.create_subscription(String, "/cabot/user_query", self.query_callback, 10)

        # self.event_sub = self.create_subscription(String, "/cabot/event", self.event_callback, 10)
        self.query_type = None
        self.query_string = None
        self.candidates = set(candidates)

        self.cancel_pub = self.create_publisher(String, "/cabot/event", 10)

        self.dir_to_jp = {
            "front": "前",
            "front_left": "左前",
            "front_right": "右前",
            "left": "左",
            "right": "右",
            "back": "後ろ",
            "back_left": "左後ろ",
            "back_right": "右後ろ"
        }

        # create service client
        self.client = self.create_client(Trigger, 'trigger_navigation_cancel')
        
        # wait until service client is ready
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Service not available, waiting...')

        # activate timer for checking cancel state
        self.timer = self.create_timer(5.0, self.check_cancel_state)

    def query_callback(self, msg):
        self.logger.info(f"(test_loop) Received: {msg.data}")
        if msg.data == "navigation;cancel":
            self.logger.info("(test_loop) Canceling the exploration...")
            sys.exit(0)
        elif ";" not in msg.data:
            self.logger.info("Invalid query format; please use 'data: direction;front' format")
            return
        elif len(msg.data.split(";")) != 2:
            self.logger.info("Invalid query format; please use 'data: direction;front' format")
            return
        else:
            self.logger.info(f"valid query format; {msg.data}")
        
        self.query_type, self.query_string = msg.data.split(";")
        
        self.logger.info(f"Query received: {self.query_type}, {self.query_string}")
        
        if self.query_type == "direction":
            if self.query_string not in self.candidates:
                self.get_logger().info(f"Invalid direction: {self.query_string}; please select from {self.candidates}")
                speak_text(f"指定された{self.dir_to_jp[self.query_string]}方向には進めないようです。")
            else:
                # finish the node
                sys.exit(0)
        elif self.query_type == "search":
            self.logger.info("get search query")
            # finish the node
            sys.exit(0)
        else:
            self.logger.info(f"query type '{self.query_type}' is not supported in this script currently; please use 'data: direction;front' format instead")
    
    def check_cancel_state(self):
        self.logger.info("CabotQueryNode; check_cancel_state; Checking cancel state...")
        req = Trigger.Request()
        future = self.client.call_async(req)

        def callback(future):
            try:
                response = future.result()
                if response.success:
                    self.logger.info(f"CabotQueryNode; check_cancel_state; Service call succeeded: {response.message}")
                    if response.message == "running_state":
                        # change the query type to "auto" and exit the node
                        self.query_type = "auto"
                        self.logger.info("CabotQueryNode; check_cancel_state; Query type set to 'auto'. Exiting node.")
                        sys.exit(0)
                    elif response.message == "cancelled_state":
                        # if message is "cancelled_state", do nothing
                        self.logger.info("CabotQueryNode; check_cancel_state; Exploration is cancelled. No action taken.")
                else:
                    self.logger.warn(f"CabotQueryNode; check_cancel_state; Service call failed: {response.message}")
            except Exception as e:
                self.logger.error(f"CabotQueryNode; check_cancel_state; Service call failed: {e}")

        future.add_done_callback(callback)


class PersistentCancelClient(Node):
    def __init__(self):
        super().__init__('persistent_cancel_client')
        self.cli = self.create_client(Trigger, 'trigger_navigation_cancel')
        self.logger = self.get_logger()

        self.auto_mode_state = True
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('PersistentCancelClient; Service not available, waiting...')
    
    def send_request_and_wait(self):
        self.logger.info("PersistentCancelClient; Sending request to get current state (blocking)")
        req = Trigger.Request()
        future = self.cli.call_async(req)
        
        # 非同期リクエストを処理
        while rclpy.ok():
            self.logger.info("PersistentCancelClient; trying to get the result...")
            rclpy.spin_once(self)
            if future.done():
                self.logger.info("PersistentCancelClient; future is done")
                try:
                    self.logger.info(f"PersistentCancelClient; returning the result; {future.result()}")
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f"PersistentCancelClient; Service call failed: {str(e)}")
                    return None
        self.logger.error(f"PersistentCancelClient; Service call failed: No response received or rclpy is not ok (rclpy.ok()={rclpy.ok()})")



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dist_filter", "-d", action="store_true", help="Apply distance filter")
    parser.add_argument("--forbidden_area_filter", "-f", action="store_true", help="Apply forbidden area filter")
    parser.add_argument("--trajectory_filter", "-t", action="store_true", help="Apply trajectory filter")
    parser.add_argument("--auto", "-a", action="store_true", help="Automatically select the next point")
    parser.add_argument("--use_image", "-i", action="store_true", help="Generate explanation from images")
    parser.add_argument("--log_dir", "-l", type=str, help="Log directory; e.g., logs/logs_0123-123456")
    parser.add_argument("--sim", "-s", action="store_true", help="Simulator mode")
    parser.add_argument("--keyboard", "-k", action="store_true", help="Keyboard mode")
    parser.add_argument("--debug", "-db", action="store_true", help="Debug mode")
    args = parser.parse_args()

    # main(
    #     dist_filter=args.dist_filter,
    #     forbidden_area_filter=args.forbidden_area_filter,
    #     trajectory_filter=args.trajectory_filter,
    #     auto=args.auto,
    #     use_image=args.use_image,
    #     log_dir=args.log_dir,
    #     sim=args.sim,
    #     keyboard=args.keyboard,
    #     debug=args.debug
    # )
