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
import time
from typing import Optional
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_ros import TransformListener, Buffer


"""
0. put this script in cabot/cabot-navigation/docker/home/ros2_ws/src

1. launch simulator
```
$ cd cabot/cabot-navigation
$ ./launch.sh -s -e
```

3. Run this script
```
$ cd cabot/cabot-navigation
$ docker compose exec navigation bash
--- in container ---
$ source install/setup.bash
$ cd /home/developer/ros2_ws/src
$ python3 test_explore.py
```

This script will output the coordinates of the robot in the map frame to `coordinates.npy` file.
The coordinates can be used to draw the path of the robot in the map.
"""


def getch():
    """Function to capture keyboard input without pressing Enter, works in Unix."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class CaBotExplorationNode(Node):
    def __init__(self, x: int = 0, y: int = 0, query_type: str = "goal", pose: Optional[float] = None):
        super().__init__("cabot_explore_node")
        self.logger = self.get_logger()
        self.should_stop = False

        self.x = x
        self.y = y
        self.pose = pose
        self.query_type = query_type

        self.coordinates = []

        self.logger.info("cabot exploration can be implemented here")

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.localize_status_pub = self.create_publisher(MFLocalizeStatus, "/localize_status", transient_local_qos)
        self.goal_coordinate_pub = self.create_publisher(String, "/cabot/event", 10)
        self.sub = self.create_subscription(String, "/cabot/event", self.stop_timer_callback, 10)

        # Subscriber for odom and map; use ApproximateTimeSynchronizer to sync the messages
        # self.map_sub = Subscriber(self, OccupancyGrid, "/map")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.ts = ApproximateTimeSynchronizer([self.odom_sub, self.map_sub], queue_size=10, slop=0.1)
        # self.ts = ApproximateTimeSynchronizer([self.map_sub], queue_size=10, slop=0.1)
        # self.ts.registerCallback(self.sync_callback)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.sync_callback, 10)

        self.map_x = 0
        self.map_y = 0
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0

        self.timer = self.create_timer(0.01, self.timer_callback)
    
    def sync_callback(self, map_msg):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            position = transform.transform.translation
            self.coordinates.append((position.x, position.y))

            self.map_x = map_msg.info.origin.position.x
            self.map_y = map_msg.info.origin.position.y
            self.map_width = map_msg.info.width
            self.map_height = map_msg.info.height
            self.map_resolution = map_msg.info.resolution

            self.logger.info(f"Position: x={position.x}, y={position.y}")
        except Exception as e:
            self.get_logger().warn(f"Could not transform base_link to map: {e}")

    def timer_callback(self):
        self.logger.info("[CabotExplorationNode] Exploration node started")
        self.timer.cancel()
        
        msg = MFLocalizeStatus()
        msg.status = MFLocalizeStatus.TRACKING
        self.localize_status_pub.publish(msg)

        if self.query_type == "search":
            self.goal_coordinate_pub.publish(String(data="navigation_search"))
        # time.sleep(0.5)
        
        coords_msg = String()
        coords_msg.data = f"navigation;destination;goal:{self.x}:{self.y}"
        self.logger.info(f"[CabotExplorationNode] Publishing: {coords_msg}")
        self.goal_coordinate_pub.publish(coords_msg)
        self.logger.info(f"[CabotExplorationNode] Published: {coords_msg}")

    def stop_timer_callback(self, msg):
        self.logger.info(f"(test_explore) Received: {msg.data}")
        if msg.data == "navigation_arrived" or msg.data == "navigation;cancel":
            self.logger.info("[CabotExplorationNode] Goal is reached (or canceled)")

            if self.pose is not None:
                self.logger.info("[CabotExplorationNode] Pose is provided, so rotate the robot to the given pose")
            sys.exit(0)


class CancelNode(Node):
    def __init__(self):
        super().__init__("cancel_node")
        self.logger = self.get_logger()
        self.cancel_pub = self.create_publisher(String, "/cabot/event", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.timer.cancel()
        self.logger.info("Canceling navigation is called")
        cancel_msg = String()
        cancel_msg.data = "navigation;cancel"
        self.cancel_pub.publish(cancel_msg)
        self.logger.info(f"Canceling navigation by publishing: {cancel_msg}")


def main(x: int, y: int, query_type: str, cancel_mode: bool = False):
    # set log level to debug
    # rclpy.logging._root_logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
    if not cancel_mode:
        if x is None and y is None:
            raise ValueError("Please provide x and y coordinates")

    re_init = False
    if not rclpy.ok():
        rclpy.init()
        re_init = True
    rcl_publisher = CaBotExplorationNode(x, y, query_type)
    if cancel_mode:
        cancel_pub = rcl_publisher.create_publisher(String, "/cabot/event", 10)
        rcl_publisher.logger.info("Press 'c' to cancel navigation")
        try:
            while True:
                key = getch()
                if key == 'c':
                    rcl_publisher.logger.info("Cancelling navigation")
                    cancel_pub.publish(String(data="navigation;cancel"))
                    raise KeyboardInterrupt
                elif key == '\x03':  # Handle Ctrl+C
                    raise KeyboardInterrupt
        except KeyboardInterrupt:
            rcl_publisher.logger.info("ctrl+c is pressed")
        finally:
            rcl_publisher.destroy_node()
            rcl_publisher.logger.info("destroyed nodes")
            # if re_init:
            #     rclpy.shutdown()
            #     rcl_publisher.logger.info("shutdown rclpy")
    else:
        try:
            rcl_publisher.logger.info("[test_explore; main] start spin")
            rclpy.spin(rcl_publisher)
        except SystemExit as e:
            print(e)
        rcl_publisher.destroy_node()
        # if re_init:
        #     rcl_publisher.logger.info("test_explore; shutdown rclpy")
        #     rclpy.try_shutdown()

        print("spin done")
        # convert coordinates to map coordinates
        try:
            coordinates = np.asarray(rcl_publisher.coordinates) / rcl_publisher.map_resolution
            coordinates[:, 1] = -coordinates[:, 1]
            map_x = -rcl_publisher.map_x / rcl_publisher.map_resolution
            map_y = -rcl_publisher.map_y / rcl_publisher.map_resolution
            map_coordinates = np.asarray([map_x, rcl_publisher.map_height - map_y])
            coordinates += map_coordinates
            with open("coordinates.npy", "wb") as f:
                np.save(f, coordinates)
        except Exception as e:
            print(e)
            print("Error saving coordinates")
        


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-x", type=float, default=None, help="x coordinate of the goal")
    parser.add_argument("-y", type=float, default=None, help="y coordinate of the goal")
    parser.add_argument("-c", action="store_true", help="activate in cancel mode")
    args = parser.parse_args()

    main(x=args.x, y=args.y, cancel_mode=args.c)
