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
        
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
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
        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

        # calculate map orientation in radian
        map_quaternion = (msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(map_quaternion)
        self.map_orientation = yaw

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
        self.intersection_points = self.find_intersection_points(self.skeleton_map)
        self.intersection_points_num = len(self.intersection_points)
    
    def skeletonize(self, highlighted_map):
        return skeletonize(highlighted_map > np.quantile(highlighted_map, 0.8))
    
    def find_intersection_points(self, skeleton):
        skeleton = skeleton.astype(np.uint8)
        kernel = np.array([[1, 1, 1], [1, 10, 1], [1, 1, 1]])
        filtered = cv2.filter2D(skeleton, -1, kernel)
        intersection_points = np.argwhere(filtered >= 13)

        # remove points that are too close
        if len(intersection_points) > 1:
            clustering = DBSCAN(eps=25, min_samples=1).fit(intersection_points)
            labels = clustering.labels_

            # calculate center of each cluster
            unique_labels = set(labels)
            cluster_centers = []
            for label in unique_labels:
                if label == -1:
                    continue
                cluster = intersection_points[labels == label]
                center = np.mean(cluster, axis=0).astype(int)
                cluster_centers.append(center)
            return np.array(cluster_centers)
        return intersection_points
    
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


def get_direction_from_orientation(current_location: np.ndarray, orientation: np.ndarray, coordinates: np.ndarray) -> List[str]:
    # classify into 8 directions
    # 0: 337.5 - 22.5, 1: 22.5 - 67.5, 2: 67.5 - 112.5, 3: 112.5 - 157.5, 4: 157.5 - 202.5, 5: 202.5 - 247.5, 6: 247.5 - 292.5, 7: 292.5 - 337.5
    directions: List[str] = []
    coordinates = coordinates[:, ::-1]  # flip x and y
    
    for i in range(len(coordinates)):
        angle = np.arctan2(coordinates[i, 0] - current_location[0], coordinates[i, 1] - current_location[1]) - orientation
        if angle < 0:
            angle += 2 * np.pi
        angle_in_degree = np.degrees(angle)
        if angle_in_degree < 22.5 or angle_in_degree >= 337.5:
            each_direction = 0
        elif angle_in_degree < 67.5:
            each_direction = 1
        elif angle_in_degree < 112.5:
            each_direction = 2
        elif angle_in_degree < 157.5:
            each_direction = 3
        elif angle_in_degree < 202.5:
            each_direction = 4
        elif angle_in_degree < 247.5:
            each_direction = 5
        elif angle_in_degree < 292.5:
            each_direction = 6
        else:
            each_direction = 7
        # print(f"current location: {current_location}, coordinate: {coordinates[i]}, angle: {angle_in_degree}, direction: {dir_map[each_direction]}")
        directions.append(dir_map[each_direction])
    return directions


class CheckForbiddenArea:
    def __init__(self):
        self.vertices = [(-9.366968154907227, 1.053526520729065),
            (-4.28364896774292, 9.702329635620117),
            (-11.109362602233887, 14.134446144104004),
            (-12.125733375549316, 14.788288116455078),
            (-16.91551971435547, 20.455156326293945),
            (-18.444501876831055, 23.94382667541504),
            (-23.88684844970703, 26.557083129882812),
            (-24.6151123046875, 28.59307289123535),
            (-21.491485595703125, 32.22541427612305),
            (-12.632528305053711, 24.962932586669922),
            (-9.219273567199707, 24.091917037963867),
            (-10.890478134155273, 21.55048942565918),
            (-5.587409019470215, 14.577529907226562),
            (0.6534930467605591, 13.562857627868652),
            (2.98064923286438, 9.78571605682373),
            (8.78853702545166, 7.46344518661499),
            (12.202397346496582, 9.498923301696777),
            (17.793643951416016, 8.191060066223145),
            (19.756338119506836, 6.085613250732422),
            (30.430456161499023, -0.6697861552238464),
            (26.582834243774414, -13.451976776123047),
            (15.253827095031738, -9.601555824279785),
            (9.734109878540039, -3.643073320388794),
            (4.436126708984375, -0.9560613036155701),
            (-1.0094401836395264, -5.9678497314453125)]
        self.polygon = Polygon(self.vertices)
    
    def is_point_in_forbidden_area(self, odom_x, odom_y) -> bool:
        point = Point(odom_x, odom_y)
        # if the point is in the area, return False (area is allowed)
        return not self.polygon.contains(point)


class FilterCandidates:
    def __init__(
            self, 
            map_data: MapData, 
            floor: int,
            do_dist_filter: bool = True, 
            do_forbidden_area_filter: bool = True, 
            do_trajectory_filter: bool = True,
            availability_from_image: Optional[Dict[str, bool]] = None,
            forbidden_area_centers: Optional[List[Tuple[float, float]]] = None,
            initial_pose_filter: bool = True,
            log_dir: str = ".",
            marker_a: Optional[float] = None,
            marker_b: Optional[float] = None,
            logger: Optional[Any] = None
        ):
        self.logger = logger

        # forbidden_area_centers: [(x, y), ...] in odom coordinates
        self.map_x = map_data.map_x
        self.map_y = map_data.map_y
        self.map_resolution = map_data.map_resolution
        self.map_height = map_data.map_height
        self.map_data = map_data

        self.initial_coords = (0, 0)
        self.initial_orientation = 0

        self.availability_from_image = availability_from_image
        self.forbidden_area_centers = [] if forbidden_area_centers is None else forbidden_area_centers

        self.log_dir = log_dir

        self.do_dist_filter = do_dist_filter
        self.do_forbidden_area_filter = do_forbidden_area_filter
        self.do_trajectory_filter = do_trajectory_filter
        self.do_initial_pose_filter = initial_pose_filter
        print(f"filter: dist: {do_dist_filter}, "
              f"forbidden: {do_forbidden_area_filter}, "
              f"trajectory: {do_trajectory_filter} "
              f"initial_pose: {initial_pose_filter}"
        )

        if self.do_forbidden_area_filter:
            self.forbidden_area_checker = CheckForbiddenArea()
        
        # check trajectory from trajectory.npy 
        if do_trajectory_filter:
            trajectory = np.load("trajectory.npy")  # odom coordinates
            trajectories_in_map = convert_odom_to_map_batch(trajectory, self.map_x, self.map_y, self.map_resolution, self.map_height)
            # set area around the trajectory as forbidden area
            traj_forbidden_map = np.zeros(map_data.map_data.shape)
            traj_forbidden_size = 10
            traj_forbidden_map[trajectories_in_map[:, 1].astype(int), trajectories_in_map[:, 0].astype(int)] = 100
            traj_forbidden_map = cv2.dilate(traj_forbidden_map, np.ones((traj_forbidden_size, traj_forbidden_size), np.uint8))
            # apply gaussian filter to make the forbidden area smoother
            traj_forbidden_map = gaussian_filter(traj_forbidden_map, sigma=10)
            traj_forbidden_map = traj_forbidden_map > 0
            # debug; save forbidden map
            # plt.imshow(traj_forbidden_map, cmap='gray')
            # plt.savefig("traj_forbidden_area.png")
            # plt.clf()
            # plt.close()
            self.traj_forbidden_map = traj_forbidden_map
        else:
            self.traj_forbidden_map = np.zeros(map_data.map_data.shape)
        
        if self.availability_from_image is not None:
            self.avail_map = np.zeros(map_data.map_data.shape)
            self.all_avail_points = np.stack(
                np.meshgrid(
                    np.arange(map_data.map_data.shape[1]), 
                    np.arange(map_data.map_data.shape[0]), 
                    indexing='ij'
                ), axis=-1
            ).reshape(-1, 2)

        self.max_dist = 500
        self.min_dist = 50
        if floor == 5:
            corridor_width_meter = 7.5  # corridor width in meter (5th floor): 7.5m
        elif floor == 3:
            corridor_width_meter = 3
        elif floor == 7:
            corridor_width_meter = 2
        else:
            corridor_width_meter = 3  # default value
        self.corridor_width = (corridor_width_meter / 2) / self.map_resolution  # convert meter to pixel

        # for marker filter
        self.marker_a = marker_a
        self.marker_b = marker_b
    
    def save_traj_map(self, log_dir: str = "."):
        plt.imshow(self.traj_forbidden_map, cmap='gray')
        plt.savefig(f"{log_dir}/traj_forbidden_area.png")
        plt.clf()
        plt.close()
    
    def dist_filter(self, current_point: np.ndarray, candidates: np.ndarray, do_max_filter = False, do_min_filter = False) -> np.ndarray:
        # coords: (n, 2); n: number of coordinates
        # score 1 for the points that are in the forbidden area
        score_map = np.zeros(candidates.shape[0])
        distances = np.linalg.norm(candidates - current_point[::-1], axis=1)
        if do_max_filter:
            score_map[distances > self.max_dist] = distances[distances > self.max_dist] / self.max_dist
        if do_min_filter:
            score_map[distances < self.min_dist] = 10
        score_map[distances > self.max_dist * 2] = 10
        return score_map
    
    def marker_filter(
            self, 
            current_point: np.ndarray, 
            orientation, 
            candidates: np.ndarray, 
            initial_orientation: float
        ) -> np.ndarray:
        # if marker is detected, set the area defined by the following conditions as forbidden area
        # the area behind the marker
        # i.e., the area that is in front of the current point and behind the marker
        # behind the marker: parallel to the initial pose of the robot and 3m away from the marker

        # debug; 900 < x < 1200 -> consider marker is detected
        # if 900 < current_point[0] < 1200:
        #     self.availability_from_image = {
        #         "front_marker": True,
        #         "left_marker": False,
        #         "right_marker": False,
        #         "front_available": False,
        #         "left_available": False,
        #         "right_available": False,
        #     }

        if self.availability_from_image is None:
            return np.zeros(candidates.shape[0])
        
        if self.availability_from_image["front_marker"]:
            if self.marker_a is None:
                # get the point 3m away from the current point in the direction of the initial orientation
                dist_to_marker = 100
                marker_point = np.array([current_point[0] + dist_to_marker * np.sin(initial_orientation), current_point[1] + dist_to_marker * np.cos(initial_orientation)])
                # get the direction of the marker
                marker_direction = initial_orientation + np.pi
                
                # calculate the line equation of the boundary of the forbidden area
                # y = ax + b
                self.marker_a = np.tan(marker_direction)
                self.marker_b = marker_point[1] - self.marker_a * marker_point[0]
        if self.marker_a is not None and self.marker_b is not None:
            # set the area behind the marker as forbidden area
            # i.e., set 100 for the points that are in the forbidden area in the self.avail_map
            scores = np.zeros(candidates.shape[0])
            for i, candidate in enumerate(candidates):
                if candidate[1] > self.marker_a * candidate[0] + self.marker_b:
                    scores[i] = 10
            return scores
        else:
            return np.zeros(candidates.shape[0])
    
    def replanning_filter(self, previous_destination: np.ndarray, candidates: np.ndarray) -> np.ndarray:
        # set the area around the previous destination as forbidden area
        # calculate the distance from the previous destination
        thres_dist = 3 / self.map_resolution
        previous_destination = convert_odom_to_map_batch(np.array([previous_destination]), self.map_x, self.map_y, self.map_resolution, self.map_height)[0]
        dists = np.linalg.norm(candidates - previous_destination, axis=1)
        self.logger.info(f"replanning filter; candidates: {candidates}, previous destination: {previous_destination}, dists: {dists}")
        score_map = np.zeros(candidates.shape[0])
        score_map[dists < thres_dist] = 10
        return score_map
    
    def availability_filter(
            self, current_point: np.ndarray, orientation, candidates: np.ndarray
        ) -> np.ndarray:
        # if availability is False, set 3m-radius circular area which center is 6m away from the current point to that direction as forbidden area
        front_availability = (not self.availability_from_image["front_marker"]) and self.availability_from_image["front_available"]
        left_availability = (not self.availability_from_image["left_marker"]) and self.availability_from_image["left_available"]
        right_availability = (not self.availability_from_image["right_marker"]) and self.availability_from_image["right_available"]

        radius = 4 / self.map_resolution
        center_dist = 8 / self.map_resolution

        if not front_availability:
            front_center = current_point + np.array([center_dist * np.sin(orientation), center_dist * np.cos(orientation)])
            # set 3m-radius circular area as forbidden area in the self.avail_map
            # i.e., set 100 for the points that are in the forbidden area in the self.avail_map
            front_center_in_odom = convert_map_to_odom(front_center[1], front_center[0], self.map_x, self.map_y, self.map_resolution, self.map_height)
            self.forbidden_area_centers.append(front_center_in_odom)
            # front_forbidden_area = np.linalg.norm(self.all_avail_points - front_center, axis=1) < radius
            # self.avail_map[self.all_avail_points[front_forbidden_area][:, 1].astype(int), self.all_avail_points[front_forbidden_area][:, 0].astype(int)] = 100
        if not left_availability:
            left_center = current_point + np.array([center_dist * np.sin(orientation + np.pi / 2), center_dist * np.cos(orientation + np.pi / 2)])
            left_center_in_odom = convert_map_to_odom(left_center[1], left_center[0], self.map_x, self.map_y, self.map_resolution, self.map_height)
            self.forbidden_area_centers.append(left_center_in_odom)
            # left_forbidden_area = np.linalg.norm(self.all_avail_points - left_center, axis=1) < radius
            # self.avail_map[self.all_avail_points[left_forbidden_area][:, 1].astype(int), self.all_avail_points[left_forbidden_area][:, 0].astype(int)] = 100
        if not right_availability:
            right_center = current_point + np.array([center_dist * np.sin(orientation - np.pi / 2), center_dist * np.cos(orientation - np.pi / 2)])
            right_center_in_odom = convert_map_to_odom(right_center[1], right_center[0], self.map_x, self.map_y, self.map_resolution, self.map_height)
            self.forbidden_area_centers.append(right_center_in_odom)
            # right_forbidden_area = np.linalg.norm(self.all_avail_points - right_center, axis=1) < radius
            # self.avail_map[self.all_avail_points[right_forbidden_area][:, 1].astype(int), self.all_avail_points[right_forbidden_area][:, 0].astype(int)] = 100
        
        if len(self.forbidden_area_centers) > 0:
            center_in_map = convert_odom_to_map_batch(np.array(self.forbidden_area_centers), self.map_x, self.map_y, self.map_resolution, self.map_height)[:, ::-1]
            for center in center_in_map:
                forbidden_area = np.linalg.norm(self.all_avail_points - center, axis=1) < radius
                self.avail_map[self.all_avail_points[forbidden_area][:, 1].astype(int), self.all_avail_points[forbidden_area][:, 0].astype(int)] = 100
        
            # check if the candidates are in the forbidden area
            score_map = np.zeros(candidates.shape[0])
            score_map[self.avail_map[candidates[:, 0].astype(int), candidates[:, 1].astype(int)] == 100] = 10

            # for debug; save avail_map
            print("saving avail_map...")
            plt.imshow(self.avail_map, cmap='gray')
            plt.scatter(current_point[0], current_point[1], c='r', s=10)
            plt.arrow(current_point[0], current_point[1], 20 * np.sin(orientation), 20 * np.cos(orientation), head_width=20, head_length=20, fc='r', ec='r')
            plt.savefig(f"{self.log_dir}/avail_map.png")
            plt.clf()
            plt.close()
            print(f"avail_map saved to {self.log_dir}/avail_map.png")
        else:
            score_map = np.zeros(candidates.shape[0])

        return score_map
    
    def forbidden_area_filter(self, candidates: np.ndarray) -> np.ndarray:
        # coords: (n, 2); n: number of coordinates
        # score 100 for the points that are in the forbidden area
        score_map = np.zeros(candidates.shape[0])
        # candidates_in_odom = convert_map_to_odom_batch(candidates, self.map_x, self.map_y, self.map_resolution, self.map_height)
        is_forbidden = []
        # print("checking forbidden area...")
        for cand in candidates:
            cand_in_odom = convert_map_to_odom(cand[1], cand[0], self.map_x, self.map_y, self.map_resolution, self.map_height)
            is_forbidden.append(self.forbidden_area_checker.is_point_in_forbidden_area(cand_in_odom[0], cand_in_odom[1]))
        # is_forbidden = self.forbidden_area_checker.points_check(candidates_in_odom)
        score_map[is_forbidden] = 10
        return score_map
    
    def trajectory_filter(self, candidates: np.ndarray) -> np.ndarray:
        # coords: (n, 2); n: number of coordinates
        # score 1 for the points that are in the forbidden area
        score_map = np.zeros(candidates.shape[0])
        is_forbidden = self.traj_forbidden_map[candidates[:, 0].astype(int), candidates[:, 1].astype(int)]
        score_map[is_forbidden] = 1
        return score_map

    def initial_pose_filter(self, candidates: np.ndarray, initial_coords: Tuple[float, float], initial_orientation: float) -> np.ndarray:
        # coords: (n, 2); n: number of coordinates
        # score 1 for the points that are in the forbidden area
        self.initial_coords = initial_coords
        self.initial_orientation = initial_orientation

        score_map = np.zeros(candidates.shape[0])
        dists = self.dist_from_initial_pose(candidates)
        score_map[dists > self.corridor_width] = 10
        return score_map

    def dist_from_initial_pose(self, candidates: np.ndarray) -> np.ndarray:
        initial_coords_in_map = convert_odom_to_map_batch(np.array([self.initial_coords]), self.map_x, self.map_y, self.map_resolution, self.map_height)[0]
        initial_coords_in_map = initial_coords_in_map[::-1].astype(int)
        initial_orientation = np.pi / 2 + self.initial_orientation
        # filter out the points that are too far to the line from the initial point and the orientation
        # first, calculate the line equation from the initial point and the orientation
        # i.e., y = ax + b
        a = np.tan(initial_orientation)
        b = initial_coords_in_map[1] - a * initial_coords_in_map[0]
        dists = np.abs(a * candidates[:, 0] - candidates[:, 1] + b) / np.sqrt(a ** 2 + 1)
        return dists

    def costmap_filter(self, candidates: np.ndarray) -> np.ndarray:
        score_map = np.zeros(candidates.shape[0])
        
        candidates_without_additional = candidates[:self.map_data.intersection_points_num, :]
        score_candidates_without_additional = self.map_data.highlighted_map[candidates_without_additional[:, 0].astype(int), candidates_without_additional[:, 1].astype(int)]
        map_max_score = np.max(score_candidates_without_additional)
        map_min_score = max(np.min(score_candidates_without_additional), 1)
        
        cand_on_map = self.map_data.highlighted_map[candidates[:, 0].astype(int), candidates[:, 1].astype(int)]
        is_forbidden = cand_on_map < map_min_score
        score_map[is_forbidden] = 20
        return score_map

    def filter(self, current_point, candidates, orientation, previous_destination) -> np.ndarray:
        # candidates: (n, 2); n: number of coordinates
        # orientation: (1,); current orientation in radian
        # return: (n, 1); n: number of coordinates
        if self.do_dist_filter:
            dist_filter = self.dist_filter(current_point, candidates, do_max_filter=True, do_min_filter=True)
        else:
            dist_filter = np.zeros(candidates.shape[0])
        
        if self.do_forbidden_area_filter:
            forbidden_area_filter = self.forbidden_area_filter(candidates)
        else:
            forbidden_area_filter = np.zeros(candidates.shape[0])
        
        if self.do_trajectory_filter:
            trajectory_filter = self.trajectory_filter(candidates)
        else:
            trajectory_filter = np.zeros(candidates.shape[0])
        
        if self.availability_from_image is not None:
            availability_filter = self.availability_filter(current_point, orientation, candidates)
        else:
            availability_filter = np.zeros(candidates.shape[0])
        
        if self.do_initial_pose_filter:
            initial_pose_filter = self.initial_pose_filter(candidates, (0, 0), 0)
        else:
            initial_pose_filter = np.zeros(candidates.shape[0])
        
        marker_filter = self.marker_filter(current_point, orientation, candidates, self.initial_orientation)
        replanning_filter = self.replanning_filter(previous_destination, candidates)
        costmap_filter = self.costmap_filter(candidates)

        # Log all the filter scores
        self.logger.info(f"dist_filter: {dist_filter}")
        self.logger.info(f"forbidden_area_filter: {forbidden_area_filter}")
        self.logger.info(f"trajectory_filter: {trajectory_filter}")
        self.logger.info(f"availability_filter: {availability_filter}")
        self.logger.info(f"initial_pose_filter: {initial_pose_filter}")
        self.logger.info(f"marker_filter: {marker_filter}")
        self.logger.info(f"replanning_filter: {replanning_filter}")
        self.logger.info(f"costmap_filter: {costmap_filter}")
        
        #score_map = dist_filter + forbidden_area_filter + trajectory_filter + availability_filter \
        #      + initial_pose_filter + marker_filter + replanning_filter + costmap_filter
        
        score_map = forbidden_area_filter + trajectory_filter + availability_filter \
              + initial_pose_filter + marker_filter + replanning_filter + costmap_filter
        #return score_map

        return np.zeros(score_map.shape) - score_map
        

def set_next_point_based_on_skeleton(
        map_array: np.ndarray, 
        floor: int,
        orientation: np.ndarray, 
        coords: np.ndarray, 
        map_resolution: float, 
        map_x: int, 
        map_y: int, 
        map_height: int,
        do_dist_filter: bool = True,
        do_forbidden_area_filter: bool = True,
        do_trajectory_filter: bool = True,
        auto_mode: bool = False,
        log_dir: str = ".",
        availability_from_image: Optional[Dict[str, bool]] = None,
        forbidden_centers: Optional[List[Tuple[float, float]]] = None,
        initial_coords: Optional[Tuple[float, float]] = None,
        initial_orientation: Optional[float] = None,
        follow_initial_orientation: bool = True,
        marker_a: Optional[float] = None,
        marker_b: Optional[float] = None,
        previous_destination: Optional[np.ndarray] = None,
        logger: Optional[Any] = None
    ):
    orientation = np.pi / 2 + orientation  # flip x axis
    
    # initialize map data
    map_data = MapData(map_x, map_y, map_resolution, map_height, map_array)
    cv2.imwrite(f"{log_dir}/highlighted_map.png", map_data.costmap)
    np.save(f"{log_dir}/highlighted_map.npy", map_data.highlighted_map)

    # create candidate filter
    cand_filter = FilterCandidates(
        map_data, 
        floor=floor,
        do_dist_filter=do_dist_filter,
        do_forbidden_area_filter=do_forbidden_area_filter,
        do_trajectory_filter=do_trajectory_filter,
        availability_from_image=availability_from_image,
        forbidden_area_centers=forbidden_centers,
        initial_pose_filter=follow_initial_orientation,
        log_dir=log_dir,
        marker_a=marker_a,
        marker_b=marker_b,
        logger=logger
    )
    if do_trajectory_filter:
        cand_filter.save_traj_map(log_dir)
    
        # calculate cover rate (ratio of trajectory points that are covered by the highlighted map)
        traj_map = cand_filter.traj_forbidden_map
        base_map = np.zeros(traj_map.shape)
        base_map[map_data.costmap > 0] = 1
        covered_map = np.zeros(traj_map.shape)
        covered_map[traj_map & (base_map > 0)] = 1
        cover_rate = np.sum(covered_map) / np.sum(base_map)
        print(f"cover rate: {cover_rate:.4f}")

    # for i, cand in enumerate(map_data.intersection_points):
    #     print(f"candidate {i}: {cand[::-1]} (x: {cand_coords_odom[i][0]:.2f}, y: {cand_coords_odom[i][1]:.2f})")
    
    # # tmp debug; use all points on the map as candidates
    # intersection_points = np.argwhere(map_data.highlighted_map > -1)
    # # randomly sample 1000 points
    # if len(intersection_points) > 1000:
    #     intersection_points = intersection_points[np.random.choice(len(intersection_points), 1000)]
    # map_data.intersection_points = intersection_points

    logger.info(f"candidates: {map_data.intersection_points}")
    # add one point for each direction to make sure that the robot can move to the direction
    # 3m away from the current point
    additional_dist = 1 / map_resolution
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
    map_data.intersection_points = np.concatenate([map_data.intersection_points, additional_points], axis=0)
    logger.info(f"candidates with additional points: {map_data.intersection_points}")
    
    cand_score = cand_filter.filter(
        current_point=coords[-1], candidates=map_data.intersection_points,
        orientation=orientation[-1], previous_destination=previous_destination
    )
    # save current coords as txt
    np.savetxt(f"{log_dir}/current_coords.txt", coords[-1])
    
    cand_score = (cand_score + 1) * 100
    filtered_map = np.zeros(map_data.highlighted_map.shape)
    filtered_map[map_data.intersection_points[:, 0], map_data.intersection_points[:, 1]] = cand_score
    np.save(f"{log_dir}/filtered_map.npy", filtered_map)
    plt.imshow(filtered_map, cmap='gray')
    plt.savefig(f"{log_dir}/filtered_map.png")
    print("filtered map saved")
    plt.clf()
    plt.close()

    # plot non-zero values
    nonzero = np.nonzero(filtered_map)

    plt.imshow(map_data.highlighted_map, cmap='gray', alpha=0.5)
    plt.scatter(nonzero[1], nonzero[0], s=10, cmap='rainbow', c=filtered_map[nonzero])
    plt.colorbar()
    plt.scatter(coords[-1, 0], coords[-1, 1], s=80, c='black', marker='*')
    plt.savefig(f"{log_dir}/filtered_map_with_score.png")
    print(f"filtered map with score saved at {log_dir}/filtered_map_with_score.png")
    plt.clf()
    plt.close()

    # copy filtered_map_with_score.png to upper directory
    os.system(f"cp {log_dir}/filtered_map_with_score.png {log_dir}/../filtered_map_with_score.png")

    directions = get_direction_from_orientation(coords[-1], orientation[-1], map_data.intersection_points)

    # randomly sample a point from each direction
    # note; the point should be in 25%-75% of the distance from the current point to the intersection point
    direction_to_points = {}  # {direction: [points], ...}
    for i, direction in enumerate(directions):
        if direction not in direction_to_points:
            direction_to_points[direction] = []
        direction_to_points[direction].append(map_data.intersection_points[i])
    
    sampled_points = []  # [[point, direction], ...]
    for direction, points in direction_to_points.items():
        if len(points) == 0:
            logger.info(f"no candidate points in {direction}")
            continue
        logger.info(f"{direction}: {len(points)}")
        points_array = np.array(points)
        points_scores = filtered_map[points_array[:, 0], points_array[:, 1]]
        min_score = np.min(points_scores)
        if min_score > 500:
            logger.info(f"no candidate points with score (cost) < 500 in {direction}")
            continue
        min_score_points = points_array[points_scores == min_score]
        min_score_points_scores = points_scores[points_scores == min_score]

        # add distance from the initial pose to the score
        dist_from_initial_pose = cand_filter.dist_from_initial_pose(min_score_points)
        logger.info(f"[set_next_point_based_on_skeleton] min score: {min_score_points_scores}, dist from initial pose: {dist_from_initial_pose}")
        min_score_points_scores += dist_from_initial_pose
        logger.info(f"[set_next_point_based_on_skeleton] min score after adding dist from initial pose: {min_score_points_scores}")

        min_score_points = min_score_points[min_score_points_scores == np.min(min_score_points_scores)]
        sampled_point = min_score_points[np.random.choice(len(min_score_points))]
        sampled_points.append([sampled_point, direction, min_score])

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
    
    # draw map and points
    plt.imshow(map_data.highlighted_map, cmap='gray')
    plt.imshow(map_data.skeleton_map, cmap='gray', alpha=0.5)
    # plot intersection points (color is defined by the direction)
    
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w']
    for i, cand in enumerate(map_data.intersection_points):
        plt.scatter(cand[1], cand[0], color=colors[int(inv_dir_map[directions[i]])], s=50)
    plt.plot(coords[-1, 0], coords[-1, 1], 'wo')
    dx = np.sin(orientation[-1]) * 50
    dy = np.cos(orientation[-1]) * 50
    plt.arrow(coords[-1, 0], coords[-1, 1], dx, dy, head_width=20, head_length=20, fc='w', ec='w', zorder=10)
    # show legends at outside of the plot
    for i, direction in enumerate(dir_map.values()):
        plt.scatter(0, 0, color=colors[i], label=direction)
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))

    # draw sampled points
    for i, cand_and_dir in enumerate(sampled_points):
        cand = cand_and_dir[0]
        color = colors[int(inv_dir_map[cand_and_dir[1]])]
        plt.scatter(cand[1], cand[0], color=color, s=170, marker='*', edgecolors='white', linewidths=1.5)
    plt.savefig(f"{log_dir}/local_map.png", bbox_inches='tight')
    plt.clf()
    plt.close()

    # copy local_map.png to upper directory
    os.system(f"cp {log_dir}/local_map.png {log_dir}/../local_map.png")
    
    sampled_point_in_odom = [convert_map_to_odom(point[0][1], point[0][0], map_x, map_y, map_resolution, map_height) for point in sampled_points]
    sampled_directions = [point[1] for point in sampled_points]
    sampled_points_and_directions = [[[point[0], point[1]], direction] for point, direction in zip(sampled_point_in_odom, sampled_directions)]
    return sampled_points_and_directions, cand_filter.forbidden_area_centers, current_coords_odom, orientation[-1], map_data.costmap, cand_filter.marker_a, cand_filter.marker_b


def main(
        floor: int,
        do_dist_filter: bool = True,
        do_forbidden_area_filter: bool = True,
        do_trajectory_filter: bool = True,
        auto_mode: bool = False,
        log_dir: str = ".",
        availability_from_image: Optional[Dict[str, bool]] = None,
        forbidden_centers: Optional[List[Tuple[float, float]]] = None,
        initial_coords: Optional[Tuple[float, float]] = None,
        initial_orientation: Optional[float] = None,
        follow_initial_orientation: bool = True,
        marker_a: Optional[float] = None,
        marker_b: Optional[float] = None,
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
    np.save(f"{log_dir}/local_costmap.npy", map_data)
    # save map metadata
    with open(f"{log_dir}/map_metadata.json", "w") as f:
        json.dump({
            "map_resolution": map_resolution,
            "map_x": map_x,
            "map_y": map_y,
            "map_height": map_height,
            "map_width": map_width
        }, f)

    # convert coordinates to map coordinates
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

    # set next point
    map_data = np.load(f"{log_dir}/local_costmap.npy")
    # next_point = set_next_point(map_data, orientation, coordinates, map_resolution, map_x, map_y, map_height)
    # print(f"Next point: {next_point}")
    output_point, forbidden_centers, current_coords, current_orientation, costmap, marker_a, marker_b = set_next_point_based_on_skeleton(
        map_data, floor, orientation, coordinates, 
        map_resolution, map_x, map_y, map_height, 
        do_dist_filter=do_dist_filter, 
        do_forbidden_area_filter=do_forbidden_area_filter, 
        do_trajectory_filter=do_trajectory_filter, 
        auto_mode=auto_mode, 
        log_dir=log_dir, 
        availability_from_image=availability_from_image,
        forbidden_centers=forbidden_centers,
        initial_coords=initial_coords,
        initial_orientation=initial_orientation,
        follow_initial_orientation=follow_initial_orientation,
        marker_a=marker_a,
        marker_b=marker_b,
        previous_destination=previous_destination,
        logger=logger
    )
    return output_point, forbidden_centers, current_coords, current_orientation, costmap, marker_a, marker_b


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