import argparse
import datetime
import json
import os
import random
import sys
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter
from shapely.geometry import Point, Polygon
from skimage.morphology import skeletonize
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf_transformations
from geometry_msgs.msg import Point
from mf_localization_msgs.msg import MFLocalizeStatus
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
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
$ python3 test_map.py
```

This script will output the current local costmap to `local_costmap.npy` file.
"""

dir_map = {
        0: "front",
        1: "front_left",
        2: "left",
        3: "back_left",
        4: "back",
        5: "back_right",
        6: "right",
        7: "front_right"
    }
inv_dir_map = {v: k for k, v in dir_map.items()}



class CaBotMapNode(Node):
    def __init__(self):
        super().__init__("cabot_map_node")
        self.logger = self.get_logger()
        self.logger.info("[CaBotMapNode] CaBotMapNode is initialized")

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.localize_status_pub = self.create_publisher(MFLocalizeStatus, "/localize_status", transient_local_qos)
        
        self.map_sub = self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.map_callback, transient_local_qos)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_x = 0
        self.map_y = 0
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_orientation = 0
        self.coordinates = []
        self.orientation = []

    def map_callback(self, msg):
        """
        Receive map data and save it to local variables
        """
        if self.map_width != 0:
            # map already received
            return

        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

        # calculate map orientation in radian
        map_quaternion = (msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(map_quaternion)
        self.map_orientation = yaw

        if(self.map_width == 0):
            self.logger.info("[CaBotMapNode] map callback received but map width is 0, waiting for valid map...")
            return

        # get occupancy grid data
        self.map_data = np.asarray(msg.data).reshape((msg.info.height, msg.info.width))
        self.logger.info(f"[CaBotMapNode] map data; x: {self.map_x:.5f}, y: {self.map_y:.5f}, width: {self.map_width}, height: {self.map_height}, resolution: {self.map_resolution:.5f}, orientation: {self.map_orientation:.5f}")

    def odom_callback(self, msg):
        """
        Receive odometry data and save it to local variables;
        If the map data is received, save the odometry data and exit the program
        """
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = transform.transform.translation
            quaternion = transform.transform.rotation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

            # compare with odom-based position
            # odom_x = msg.pose.pose.position.x
            # odom_y = msg.pose.pose.position.y
            # odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            # odom_roll, odom_pitch, odom_yaw = tf_transformations.euler_from_quaternion(odom_quaternion)
            # self.get_logger().info(f"[CaBotMapNode] odom callback; tf_x: {position.x:.6f}, tf_y: {position.y:.6f}, tf_yaw: {yaw:.6f}, odom_x: {odom_x:.6f}, odom_y: {odom_y:.6f}, odom_yaw: {odom_yaw:.6f}")

            self.coordinates.append((position.x, position.y))
            self.orientation.append(yaw)  # yaw is the rotation around the z-axis; unit: radian
            
            if self.map_resolution != 0:
                print(f"odom callback; x: {position.x:.6f}, y: {position.y:.6f}, yaw: {yaw:.6f}")
                # save local costmap and exit
                sys.exit(0)
        except Exception as e:
            self.get_logger().warn(f"Could not transform base_link to map: {e}")


class CabotRvizPointDrawer(Node):
    def __init__(self, coordinates: List[Tuple[float, float]], colors: List[Tuple[float, float, float]] = None):
        super().__init__("cabot_rviz_point_drawer")
        self.logger = self.get_logger()
        self.logger.info("CabotRvizPointDrawer is initialized")
        self.publisher_ = self.create_publisher(MarkerArray, "/marker", 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.color = colors
        self.coordinates = coordinates
    
    def publish_clear_points(self):
        """
        Clear points on rviz
        """
        self.logger.info("Clearing points on rviz")
        marker = MarkerArray()
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "points"
        marker_msg.action = Marker.DELETEALL
        marker.markers.append(marker_msg)
        self.publisher_.publish(marker)
    
    def publish_points(self):
        """
        Draw points on rviz for visualization
        """
        self.logger.info("Publishing points on rviz")
        self.publish_clear_points()
        marker = MarkerArray()
        for i, coord in enumerate(self.coordinates):
            marker_msg = Marker()
            marker_msg.header.frame_id = "map"
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            marker_msg.ns = "points"
            marker_msg.id = i
            marker_msg.type = Marker.SPHERE
            marker_msg.action = Marker.ADD
            marker_msg.pose.position.x = coord[0]
            marker_msg.pose.position.y = coord[1]
            marker_msg.pose.position.z = 0.0
            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 1.0
            marker_msg.scale.x = 1.0
            marker_msg.scale.y = 1.0
            marker_msg.scale.z = 1.0
            marker_msg.color.a = 1.0
            if self.color is None:
                marker_msg.color.r = 1.0 if i != len(self.coordinates) - 1 else 0.0
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0 if i != len(self.coordinates) - 1 else 1.0
            else:
                marker_msg.color.r = self.color[i][0]
                marker_msg.color.g = self.color[i][1]
                marker_msg.color.b = self.color[i][2]
            marker_msg.lifetime = rclpy.duration.Duration(seconds=100).to_msg()
            marker.markers.append(marker_msg)
        self.publisher_.publish(marker)

    def timer_callback(self):
        self.publish_points()


class MapData:
    def __init__(self, map_x: int, map_y: int, map_resolution: float, map_height: int, map_data: np.ndarray):
        self.map_x = map_x
        self.map_y = map_y
        self.map_resolution = map_resolution
        self.map_height = map_height
        self.map_data = np.flip(map_data, axis=0)
        self.costmap = None

        self.highlighted_map = self.get_gaussian_map()
        self.skeleton_map = self.skeletonize(self.highlighted_map)

    def skeletonize(self, highlighted_map):
        return skeletonize(highlighted_map > np.quantile(highlighted_map, 0.8))
    
    def get_gaussian_map(self):
        highlighted_map = np.copy(self.map_data)
        max_value = np.max(self.map_data)
        highlighted_map = max_value - highlighted_map
        highlighted_map[highlighted_map == max_value + 1] = 0
        self.costmap = highlighted_map

        # apply gaussian filter to highlighted_map to make the highlighted area smoother
        highlighted_map = gaussian_filter(highlighted_map, sigma=5)
        return highlighted_map


def convert_map_to_odom(x, y, map_x, map_y, map_resolution, map_height) -> Tuple[float, float]:
    converted_x = (x - map_x) * map_resolution
    converted_y = -(y - map_height + map_y) * map_resolution
    return converted_x, converted_y

def convert_odom_to_map_batch(coords: np.ndarray, map_x: int, map_y: int, map_resolution: float, map_height: int) -> np.ndarray:
    # coords: (n, 2); n: number of coordinates
    converted_x = coords[:, 0] / map_resolution + map_x
    converted_y = -coords[:, 1] / map_resolution + map_height - map_y
    return np.stack([converted_x, converted_y], axis=1)


def draw_points_on_rviz(coordinates: List[Tuple[float, float]]):
    if not rclpy.ok():
        rclpy.init()
    point_drawer = CabotRvizPointDrawer(coordinates)
    rclpy.spin_once(point_drawer)
    point_drawer.destroy_node()


# y
def get_direction_from_orientation(current_location: np.ndarray, orientation: np.ndarray, coordinates: np.ndarray) -> List[str]:
    # classify into 8 directions
    # 0: 337.5 - 22.5, 1: 22.5 - 67.5, 2: 67.5 - 112.5, 3: 112.5 - 157.5, 4: 157.5 - 202.5, 5: 202.5 - 247.5, 6: 247.5 - 292.5, 7: 292.5 - 337.5
    directions: List[str] = []
    coordinates = coordinates[:, ::-1]  # flip x and y
    
    for i in range(len(coordinates)):
        angle = np.arctan2(coordinates[i, 0] - current_location[0], coordinates[i, 1] - current_location[1]) - orientation
        angle_in_degree = np.degrees(angle)
        each_direction = (int(angle_in_degree + 382.5) // 45) % 8
        directions.append(dir_map[each_direction])
    return directions

def set_next_point_based_on_skeleton(
        map_array: np.ndarray, 
        floor: int,
        orientation: np.ndarray, 
        coords: np.ndarray, 
        map_resolution: float, 
        map_x: int, 
        map_y: int, 
        map_height: int,
        log_dir: str = ".",
        previous_destination: Optional[np.ndarray] = None,
        logger: Optional[Any] = None
    ):
    orientation = np.pi / 2 + orientation  # flip x axis
    
    # initialize map data
    map_data = MapData(map_x, map_y, map_resolution, map_height, map_array)
    cv2.imwrite(f"{log_dir}/highlighted_map.png", map_data.costmap)
    np.save(f"{log_dir}/highlighted_map.npy", map_data.highlighted_map)

    # add one point for each direction to make sure that the robot can move to the direction
    # 3m away from the current point
    additional_dist = 3 / map_resolution
    additional_points = [
        # front
        coords[-1] + additional_dist * np.array([np.sin(orientation[-1]), np.cos(orientation[-1])]),
        # back
        coords[-1] - additional_dist * np.array([np.sin(orientation[-1]), np.cos(orientation[-1])]),
        # left
        coords[-1] + additional_dist * np.array([np.sin(orientation[-1] + np.pi / 2), np.cos(orientation[-1] + np.pi / 2)]),
        # right
        coords[-1] + additional_dist * np.array([np.sin(orientation[-1] - np.pi / 2), np.cos(orientation[-1] - np.pi / 2)])
    ]
    additional_points = np.asarray([[int(p[1]), int(p[0])] for p in additional_points])
    logger.info(f"candidates with additional points: {additional_points}")
    

    # save current coords as txt
    np.savetxt(f"{log_dir}/current_coords.txt", coords[-1])

    directions = get_direction_from_orientation(coords[-1], orientation[-1], additional_points)

    # randomly sample a point from each direction
    # note; the point should be in 25%-75% of the distance from the current point to the intersection point
    direction_to_points = {}  # {direction: [points], ...}
    for i, direction in enumerate(directions):
        if direction not in direction_to_points:
            direction_to_points[direction] = []
        direction_to_points[direction].append(additional_points[i])
    
    sampled_points = []  # [[point, direction], ...]
    for direction, points in direction_to_points.items():
        sampled_points.append([points[0], direction, 0])

    cand_coords_odom = []
    for i, cand in enumerate(sampled_points):
        converted_cand = convert_map_to_odom(cand[0][1], cand[0][0], map_x, map_y, map_resolution, map_height)
        cand_coords_odom.append(converted_cand)
        print(f"sampled point {i}: {cand[0][::-1]} ({cand[1]}) (x: {converted_cand[0]:.2f}, y: {converted_cand[1]:.2f}) (score: {cand[2]})")
    
    # draw points on rviz
    # add current point
    current_coords_odom = convert_map_to_odom(coords[-1][0], coords[-1][1], map_x, map_y, map_resolution, map_height)
    cand_coords_odom.append(current_coords_odom)
    draw_points_on_rviz(cand_coords_odom)

    # copy local_map.png to upper directory
    os.system(f"cp {log_dir}/local_map.png {log_dir}/../local_map.png")
    
    sampled_point_in_odom = [convert_map_to_odom(point[0][1], point[0][0], map_x, map_y, map_resolution, map_height) for point in sampled_points]
    sampled_directions = [point[1] for point in sampled_points]
    sampled_points_and_directions = [[[point[0], point[1]], direction] for point, direction in zip(sampled_point_in_odom, sampled_directions)]
    return sampled_points_and_directions, current_coords_odom, orientation[-1]


def main(
        floor: int,
        do_dist_filter: bool = True,
        do_forbidden_area_filter: bool = True,
        do_trajectory_filter: bool = True,
        auto_mode: bool = False,
        log_dir: str = ".",
        availability_from_image: Optional[Dict[str, bool]] = None,
        initial_coords: Optional[Tuple[float, float]] = None,
        initial_orientation: Optional[float] = None,
        follow_initial_orientation: bool = True,
        previous_destination: Optional[np.ndarray] = None,
        logger: Optional[Any] = None
    ) -> Optional[Tuple[float, float]]:
    """
    main function to get the next point based on the local map
    Args:
        do_dist_filter (bool, optional): flag to apply distance filter. Defaults to True.
        do_forbidden_area_filter (bool, optional): flag to apply forbidden area filter. Defaults to True.
        do_trajectory_filter (bool, optional): flag to apply trajectory filter. Defaults to True.
        auto_mode (bool, optional): flag to automatically select the next point. Defaults to False.
        log_dir (str, optional): directory to save logs. Defaults to ".".
        availability_from_image (Optional[Dict[str, bool]], optional): availability of the front, left, and right directions, obtained from marker and GPT. Defaults to None.
        forbidden_centers (Optional[List[Tuple[float, float]]], optional): center coordinates (odom) of the forbidden areas, calculated from `availability_from_image`. Defaults to None.

    Returns:
        Optional[Tuple[float, float]]: _description_
    """
    timestamp = datetime.datetime.now().strftime("%m%d_%H%M%S")
    log_dir = f"{log_dir}/{timestamp}"
    os.makedirs(log_dir, exist_ok=True)

    re_init = False
    if not rclpy.ok():
        rclpy.init()
        re_init = True
    rcl_publisher = CaBotMapNode()
    
    while(rcl_publisher.map_width == 0): #We need to wait until we get the map info
        try:
            rclpy.spin(rcl_publisher)
        except SystemExit as e:
            print(e)
    
    rcl_publisher.destroy_node()
    # if re_init:
    #     rclpy.shutdown()

    # save map data
    map_data = np.copy(rcl_publisher.map_data)
    map_resolution = rcl_publisher.map_resolution
    map_x = rcl_publisher.map_x
    map_y = rcl_publisher.map_y
    map_height = rcl_publisher.map_height
    map_width = rcl_publisher.map_width    
    coordinates = np.asarray(rcl_publisher.coordinates)
    orientation = np.asarray(rcl_publisher.orientation)
    
    coordinates = np.asarray(coordinates) / map_resolution
    coordinates[:, 1] = -coordinates[:, 1]
    map_x = -map_x / map_resolution
    map_y = -map_y / map_resolution
    map_coordinates = np.asarray([map_x, map_height - map_y])
    coordinates += map_coordinates
    with open(f"{log_dir}/coordinates_local_map.npy", "wb") as f:
        np.save(f, coordinates)
    with open(f"{log_dir}/orientation_local_map.npy", "wb") as f:
        np.save(f, orientation)
    
    print("local map coordinates saved")

    output_point, current_coords, current_orientation = set_next_point_based_on_skeleton(
        map_data, floor, orientation, coordinates, 
        map_resolution, map_x, map_y, map_height,  
        log_dir=log_dir, 
        previous_destination=previous_destination,
        logger=logger
    )
    return output_point, current_coords, current_orientation


if __name__ == "__main__":    
    parser = argparse.ArgumentParser()
    parser.add_argument("--dist_filter", "-d", action="store_true", help="Apply distance filter")
    parser.add_argument("--forbidden_area_filter", "-f", action="store_true", help="Apply forbidden area filter")
    parser.add_argument("--trajectory_filter", "-t", action="store_true", help="Apply trajectory filter")
    parser.add_argument("--auto", "-a", action="store_true", help="Automatically select the next point")
    args = parser.parse_args()

    _ = main(do_dist_filter=args.dist_filter, do_forbidden_area_filter=args.forbidden_area_filter, do_trajectory_filter=args.trajectory_filter, auto_mode=args.auto)
    if rclpy.ok():
        rclpy.shutdown()