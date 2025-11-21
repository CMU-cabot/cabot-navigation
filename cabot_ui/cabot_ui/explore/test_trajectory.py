import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String
from visualization_msgs.msg import Marker, MarkerArray
import argparse
from mf_localization_msgs.msg import MFLocalizeStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
import sys
import numpy as np
import tf_transformations
from scipy.ndimage import gaussian_filter
from skimage.morphology import skeletonize
import cv2
import random
from typing import Tuple, Dict, List, Any, Union, Optional
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from sklearn.cluster import DBSCAN
from tqdm import tqdm


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
$ python3 test_map.py
```

This script will output the current local costmap to `local_costmap.npy` file.
"""



class CaBotTrajectoryNode(Node):
    def __init__(self):
        super().__init__("cabot_trajectory_node")

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.localize_status_pub = self.create_publisher(MFLocalizeStatus, "/localize_status", transient_local_qos)
        
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.map_x = 0
        self.map_y = 0
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_orientation = 0
        self.coordinates = []
        self.orientation = []
        self.count = 0

    def map_callback(self, msg):
        # print("map callback")
        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

        map_quaternion = (msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(map_quaternion)
        self.map_orientation = yaw

        self.map_data = np.asarray(msg.data).reshape((msg.info.height, msg.info.width))
        # print(f"map data; x: {self.map_x:.5f}, y: {self.map_y:.5f}, width: {self.map_width}, height: {self.map_height}, resolution: {self.map_resolution:.5f}, orientation: {self.map_orientation:.5f}")
        
        # np.save("local_costmap.npy", self.map_data)
        # print("local costmap saved")
    
    def odom_callback(self, msg):
        # self.get_logger().info(f"Received: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        self.coordinates.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.orientation.append(yaw)  # yaw is the rotation around the z-axis; unit: radian
        
        # print(f"odom callback; x: {msg.pose.pose.position.x:.6f}, y: {msg.pose.pose.position.y:.6f}, yaw: {yaw:.6f}")

        # save
        np.save("trajectory.npy", np.asarray(self.coordinates))
        print(f"{self.count} trajectory saved")
        self.count += 1

        # save map data and draw trajectory
        if self.map_resolution != 0:
            map_data = np.copy(self.map_data)
            map_data = np.flip(map_data, axis=0)
            max_value = np.max(map_data)
            highlighted_map = max_value - map_data
            highlighted_map[highlighted_map == max_value + 1] = 0
            plt.imshow(highlighted_map, cmap=cm.gray)
            for i in range(len(self.coordinates)):
                x, y = self.coordinates[i]
                x, y = convert_odom_to_map(x, y, self.map_x, self.map_y, self.map_resolution, self.map_height)
                plt.scatter(x, y, s=20, cmap=cm.jet, c=[i / len(self.coordinates)], vmin=0, vmax=1)
                if i == len(self.coordinates) - 1:
                    print(f"  current odom; x: {self.coordinates[i][0]:.5f}, y: {self.coordinates[i][1]:.5f}", end="")
                    print(f" -> current position; x: {x:.5f}, y: {y:.5f}")
            plt.savefig("trajectory.png")
            plt.clf()
            plt.close()
            print(f"  {self.count} trajectory and map saved")



def convert_map_to_odom(x, y, map_x, map_y, map_resolution, map_height) -> Tuple[float, float]:
    converted_x = (x - map_x) * map_resolution
    converted_y = -(y - map_height + map_y) * map_resolution
    return converted_x, converted_y


def convert_odom_to_map(x, y, map_x, map_y, map_resolution, map_height) -> Tuple[float, float]:
    converted_x = x / map_resolution - map_x / map_resolution
    converted_y = -y / map_resolution + map_height + map_y / map_resolution
    return converted_x, converted_y



def main():
    # set log level to debug
    # rclpy.logging._root_logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

    rclpy.init()
    rcl_publisher = CaBotTrajectoryNode()
    
    try:
        rclpy.spin(rcl_publisher)
    except SystemExit as e:
        print(e)
    
    rcl_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":    
    main()