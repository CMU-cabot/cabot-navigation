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
        self.global_map_pub = self.create_publisher(OccupancyGrid, "/global_costmap/costmap", transient_local_qos)
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

        # self.global_map_pub.publish(msg)

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
        self.timer_ = self.create_timer(0.001, self.timer_callback)
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
        self.logger.info(f"Colors: {self.color}")
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
            marker_msg.scale.x = 0.4
            marker_msg.scale.y = 0.4
            marker_msg.scale.z = 0.4
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
    def __init__(self, map_x: int, map_y: int, map_resolution: float, map_width: int, map_height: int, map_data: np.ndarray):
        self.map_x = map_x
        self.map_y = map_y
        self.map_resolution = map_resolution
        self.map_width = map_width
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


def draw_points_on_rviz(coordinates: List[Tuple[float, float]], colors: List[Tuple[float, float, float]] = None):
    if not rclpy.ok():
        rclpy.init()
    point_drawer = CabotRvizPointDrawer(coordinates, colors)
    rclpy.spin_once(point_drawer)
    point_drawer.destroy_node()

def get_unknown_area_mask(map_array: np.ndarray) -> np.ndarray:
    unknown_area_mask = np.zeros_like(map_array, dtype=bool)
    unknown_area_mask[map_array == -1] = True
    return unknown_area_mask

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

def get_submap_for_direction(current_location: np.ndarray, orientation: np.ndarray, map_x, map_y, map_resolution, map_height, map_array, direction: int, submap_size_meters: int, logger) -> np.ndarray:
    # We first get the center of the submap based on the direction
    direction_angle = direction * (np.pi / 8)  # convert to radian
    distance_from_current = submap_size_meters / 2  # meters
    center_x = current_location[0] + distance_from_current * np.sin(orientation + direction_angle) / map_resolution
    center_y = current_location[1] + distance_from_current * np.cos(orientation + direction_angle) / map_resolution

    logger.info(f"Getting submap for direction {direction} at center ({center_x:.2f}, {center_y:.2f})")

    logger.info(f"Map parameters: map_x: {map_x}, map_y: {map_y}, map_resolution: {map_resolution}, map_height: {map_height}")

    # Convert center_x and center_y to map coordinates
    center_map_x = center_x
    center_map_y = center_y

    center_map_x = int(center_map_x)
    center_map_y = int(center_map_y)

    logger.info(f"Center in map coordinates: ({center_map_x}, {center_map_y})")

    half_size = int((submap_size_meters / map_resolution) / 2)
    submap = map_array[
        max(0, center_map_y - half_size):min(map_height, center_map_y + half_size),
        max(0, center_map_x - half_size):min(map_height, center_map_x + half_size)
    ]

    logger.info(f"Submap shape: {submap.shape}")

    # Also get the robot position in the submap as index coordinates (todo checking orientation here)
    robot_map_x = current_location[0]
    robot_map_y = current_location[1]
    robot_map_x = int(robot_map_x)
    robot_map_y = int(robot_map_y)
    robot_submap_x = robot_map_x - (center_map_x - half_size)
    robot_submap_y = robot_map_y - (center_map_y - half_size)

    # Floor / Ceil the robot position in the submap to make sure it's within the submap
    robot_submap_x = max(0, min(submap.shape[1] - 1, robot_submap_x))
    robot_submap_y = max(0, min(submap.shape[0] - 1, robot_submap_y))

    return submap, [robot_submap_x, robot_submap_y], [center_map_x, center_map_y] 

def compute_accessibility_score(submap: np.ndarray, robot_submap_coords: List[int], logger) -> np.ndarray:
    # Optimize using skimage.graph.MCP which uses C++ backend for Dijkstra
    from skimage import graph

    # Initialize costs array with 0.0 (free space)
    costs = np.zeros(submap.shape, dtype=np.float32)
    
    # Vectorized cost assignment based on submap values
    # Low cost
    costs[(submap > 0) & (submap < 20)] = 5.0
    # Medium cost
    costs[(submap >= 20) & (submap < 50)] = 20.0
    # High cost
    costs[(submap >= 50) & (submap < 98)] = 50.0
    # Obstacles / Unknown (Infinite cost)
    costs[(submap >= 98) | (submap == -1)] = np.inf

    # Set start point cost to 0 to avoid adding penalty for the starting pixel itself
    start_x, start_y = robot_submap_coords
    costs[start_y, start_x] = 0.0

    # Define 8-connected neighborhood offsets
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1),
               (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    # Compute minimum costs using MCP (Minimum Cost Path)
    mcp = graph.MCP(costs, offsets=offsets)
    cumulative_costs, _ = mcp.find_costs([(start_y, start_x)])
    
    # Normalize scores to [0, 1]
    # Max score reference is 100.0 as per original code
    normalized_scores = np.zeros_like(cumulative_costs, dtype=np.float32)
    
    reachable = np.isfinite(cumulative_costs)
    normalized_scores[reachable] = cumulative_costs[reachable] / 100.0
    normalized_scores[~reachable] = 1.0  # Unreachable areas get max score

    return normalized_scores

def set_next_point_based_on_skeleton(
        map_array: np.ndarray, 
        floor: int,
        orientation: np.ndarray, 
        coords: np.ndarray, 
        map_resolution: float, 
        map_x: int, 
        map_y: int, 
        map_width: int,
        map_height: int,
        log_dir: str = ".",
        previous_destination: Optional[np.ndarray] = None,
        logger: Optional[Any] = None
    ):
    orientation = np.pi / 2 + orientation  # flip x axis
    
    # initialize map data
    #cv2.imwrite(f"{log_dir}/highlighted_map.png", map_data.costmap)
    #np.save(f"{log_dir}/highlighted_map.npy", map_data.highlighted_map)

    # log unknown area mask
    #unknown_area_mask = get_unknown_area_mask(map_array)
    #np.save(f"{log_dir}/unknown_area_mask.npy", unknown_area_mask.astype(np.uint8))
    #cv2.imwrite(f"{log_dir}/unknown_area_mask.png", unknown_area_mask.astype(np.uint8) * 255)

    # add one point for each direction to make sure that the robot can move to the direction
    # 3m away from the current point

    # Todo : djikstra in 5x5m area to find points in each direction
    # Also multiply by angle and distance score later and costmap score
    # If no point found in the direction, do not add the point
    # If no point found in any direction, make the robot turn around
    # Maybe add a trace of previous path to avoid going back (v2}


    colors = []
    centers = []
    additional_points = []
    distance_score = None
    for i in range(16):
        submap, robot_submap_coords, center_submap = get_submap_for_direction(
            coords[-1], orientation[-1], map_x, map_y, map_resolution, map_height, map_array, i, submap_size_meters=10, logger=logger
        )

        if center_submap[0] <= 0 or center_submap[0] >= map_width or center_submap[1] <= 0 or center_submap[1] >= map_height:
            logger.info(f"Direction {i}: Center of submap is out of map bounds, skipping this direction.")
            continue

        meteric_center_submap = convert_map_to_odom(center_submap[0], center_submap[1], map_x, map_y, map_resolution, map_height)

        centers.append(meteric_center_submap)

        # cv2.imwrite(f"{log_dir}/submap_direction_{i}.png", submap)
        logger.info(f"submap_direction_{i} center: {center_submap}, robot in submap: {robot_submap_coords}")

        # djikstra from robot_submap_coord to score each pixel (cost) in the submap between 0 and 1
        accessibility_score = compute_accessibility_score(submap, robot_submap_coords, logger)
        accessibility_score = np.clip(accessibility_score, 0.0, 1.0)
        logger.info(f"accessibility_score_direction_{i} min: {np.min(accessibility_score)}, max: {np.max(accessibility_score)}")
        # cv2.imwrite(f"{log_dir}/accessibility_score_direction_{i}.png", (accessibility_score * 255).astype(np.uint8))

        # get a cost of distance from submap center to each pixel (0 to 1)
        if i == 0:
            height, width = submap.shape
            y_indices, x_indices = np.indices((height, width))
            distance_from_center = np.sqrt((x_indices - width / 2.0) ** 2 + (y_indices - height / 2.0) ** 2)
            max_distance = 5 / map_resolution  # 5 meters in pixels
            distance_score = (distance_from_center / max_distance)
            distance_score = np.clip(distance_score, 0.0, 1.0)
            # cv2.imwrite(f"{log_dir}/distance_score_direction_{i}.png", (distance_score * 255).astype(np.uint8))

        if accessibility_score.shape != distance_score.shape:
            logger.warning(f"Direction {i}: Accessibility score shape {accessibility_score.shape} does not match distance score shape {distance_score.shape}, skipping this direction.")
            continue

        # combine accessibility score and distance score and normalize to [0, 1]
        combined_score = (1.0 - accessibility_score) * (1.0 - distance_score)

        # cv2.imwrite(f"{log_dir}/combined_score_direction_{i}.png", (combined_score * 255).astype(np.uint8))
        logger.info(f"combined_score_direction_{i} min: {np.min(combined_score)}, max: {np.max(combined_score)}")

        # Apply a gaussian filter to smooth the combined score and favor larger areas
        smoothed_combined_score = gaussian_filter(combined_score, sigma=3)
        # Add robot position on the smoothed combined score for visualization
        # smoothed_combined_score_visu[robot_submap_coords[1], robot_submap_coords[0]] = 1.0
        # cv2.imwrite(f"{log_dir}/smoothed_combined_score_direction_{i}.png", (smoothed_combined_score * 255).astype(np.uint8))

        # Find the pixel with the highest score
        max_index = np.unravel_index(np.argmax(smoothed_combined_score), smoothed_combined_score.shape)
        logger.info(f"max_index_direction_{i}: {max_index}, score: {smoothed_combined_score[max_index]:.4f}")

        score_accepted_threshold = 0.1
        if smoothed_combined_score[max_index] < score_accepted_threshold:
            logger.info(f"Direction {i}: No suitable point found (max score {smoothed_combined_score[max_index]:.4f} < {score_accepted_threshold})")
            continue

        # Convert max_index to map coordinates
        point_map_x = center_submap[0] - (submap.shape[1] // 2) + max_index[1]
        point_map_y = center_submap[1] - (submap.shape[0] // 2) + max_index[0]
        point_map_y = map_height-point_map_y  # flip y axis

        additional_points.append([point_map_x, point_map_y])  # flip x and y for map coordinates are later
        colors.append((1.0, distance_score[max_index], 0.0))
        logger.info(f"Direction {i}: Selected point at map coordinates ({point_map_x}, {point_map_y}) with score {smoothed_combined_score[max_index]:.4f}")

    #draw_points_on_rviz(centers)

    if len(colors) == 0:
        colors = None

    #additional_dist = 3 / map_resolution
    #additional_points = [
    #    # front
    #    coords[-1] + additional_dist * np.array([np.sin(orientation[-1]), np.cos(orientation[-1])]),
    #    # back
    #    coords[-1] - additional_dist * np.array([np.sin(orientation[-1]), np.cos(orientation[-1])]),
    #    # left
    #    coords[-1] + additional_dist * np.array([np.sin(orientation[-1] + np.pi / 2), np.cos(orientation[-1] + np.pi / 2)]),
    #    # right
    #    coords[-1] + additional_dist * np.array([np.sin(orientation[-1] - np.pi / 2), np.cos(orientation[-1] - np.pi / 2)])
    #]
    additional_points = np.asarray([[int(p[1]), int(p[0])] for p in additional_points])
    logger.info(f"candidates with additional points: {additional_points}")
    
    if len(additional_points) == 0:
        logger.info("No additional points found in any direction. Robot should consider turning around.")
        return [], convert_map_to_odom(coords[-1][0], coords[-1][1], map_x, map_y, map_resolution, map_height), orientation[-1]

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
    # cand_coords_odom.append(current_coords_odom)

    draw_points_on_rviz(cand_coords_odom, colors)


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

    starting_time = datetime.datetime.now()


    # logger.info(f"TME taken begin: {(datetime.datetime.now() - starting_time).total_seconds():.2f} seconds")

    re_init = False
    if not rclpy.ok():
        rclpy.init()
        re_init = True
    rcl_publisher = CaBotMapNode()

    # logger.info(f"TME taken after CaBotMapNode: {(datetime.datetime.now() - starting_time).total_seconds():.2f} seconds")

    
    while(rcl_publisher.map_width == 0): #We need to wait until we get the map info
        try:
            rclpy.spin(rcl_publisher)
        except SystemExit as e:
            print(e)

    # logger.info(f"TME taken after get map info: {(datetime.datetime.now() - starting_time).total_seconds():.2f} seconds")

    
    rcl_publisher.destroy_node()
    # if re_init:
    #     rclpy.shutdown()

    # logger.info(f"TME taken after get map info: {(datetime.datetime.now() - starting_time).total_seconds():.2f} seconds")

    # save map data
    map_data = np.copy(rcl_publisher.map_data)
    map_resolution = rcl_publisher.map_resolution
    map_x = rcl_publisher.map_x
    map_y = rcl_publisher.map_y
    map_height = rcl_publisher.map_height
    map_width = rcl_publisher.map_width    
    coordinates = np.asarray(rcl_publisher.coordinates)
    orientation = np.asarray(rcl_publisher.orientation)

    # logger.info(f"TME taken after copy info: {(datetime.datetime.now() - starting_time).total_seconds():.2f} seconds")
    
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

    # logger.info(f"TME taken after save info: {(datetime.datetime.now() - starting_time).total_seconds():.2f} seconds")

    
    print("local map coordinates saved")

    output_point, current_coords, current_orientation = set_next_point_based_on_skeleton(
        map_data, floor, orientation, coordinates, 
        map_resolution, map_x, map_y, map_width, map_height,  
        log_dir=log_dir, 
        previous_destination=previous_destination,
        logger=logger
    )

    logger.info(f"TME taken TOTAL: {(datetime.datetime.now() - starting_time).total_seconds():.2f} seconds")

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