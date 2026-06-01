#!/usr/bin/env python3

import datetime
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import std_msgs
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
import math
import tf_transformations
from tf2_ros import TransformListener, Buffer
import matplotlib.pyplot as plt

from numba import njit, prange

class RadarVisualizer:
    def __init__(self, initial_mag_map, initial_x, initial_y):
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        # --- AXE 1 : Carte dynamique ---
        self.ax1.set_title("Navigation - Live Map Update")
        # On stocke l'objet image pour pouvoir faire set_data plus tard
        self.img_plot = self.ax1.imshow(initial_mag_map, cmap='viridis', origin='lower')
        self.robot_dot, = self.ax1.plot(initial_x, initial_y, 'ro', markersize=10)
        
        # --- AXE 2 : Histogramme Polaire ---
        self.ax2.remove()
        self.ax2 = self.fig.add_subplot(122, projection='polar')
        self.ax2.set_title("Orientation Distribution")
        
        self.theta = np.linspace(0, 2*np.pi, 16, endpoint=False)
        self.bars = self.ax2.bar(self.theta, np.zeros(16), width=0.3, color='orange', alpha=0.8)
        
        self.ax2.set_theta_zero_location('E')
        self.ax2.set_theta_direction(1)
        self.ax2.set_ylim(0, 2.5)

    def update_ui(self, x, y, local_bins, new_mag_map=None, rotation_angle=0):
        """
        Met à jour l'interface. Si new_mag_map est fourni, la carte est rafraîchie.
        """
        # 1. Update de la carte si nécessaire
        if new_mag_map is not None:
            # log min and max
            print(f"Updating mag map with new data. Min: {new_mag_map.min()}, Max: {new_mag_map.max()}")

            # Inverse X Axis for better visualization (optional, depends on how you want to display the map)
            self.img_plot.set_data(new_mag_map)
            # Optionnel : si la plage de valeurs change (ex: mag monte subitement)
            self.img_plot.set_clim(vmin=new_mag_map.min(), vmax=new_mag_map.max())

        # 2. Update Position du Robot
        self.robot_dot.set_data([x], [y])
        
        # 3. Update Histogramme
        for bar, val in zip(self.bars, local_bins):
            bar.set_height(val)

        #self.ax2.set_theta_offset(np.deg2rad(90 + rotation_angle))
        
        # 4. Refresh
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def show(self):
        plt.ioff()
        plt.show()


@njit
def get_ray_pixels(x1, y1, vx, vy, max_dist):
    # vx, vy est le vecteur unitaire du gradient
    pixels = []
    for d in range(0, max_dist):
        curr_x = int(x1 + vx * d)
        curr_y = int(y1 + vy * d)
        
        # On évite les doublons si le rayon avance lentement
        if not pixels or (curr_x, curr_y) != pixels[-1]:
            pixels.append((curr_x, curr_y))
    return pixels

def smooth_transfer(image):
    img_filtered = np.clip(image - 0.5, 0, None)

    A = 8.5 
    B = 5.5 
    
    result = (A * img_filtered) / (B + img_filtered)
    
    return result

#include the voting algorithm in the numba function to speed up the process
@njit(parallel=True)
def voting_algorithm(X, Y, U_UNIT, V_UNIT, score_dir, accessible_mask, cw, ch, max_distance=30, orientation_bins=16):

    optimal_opposition = 0.2
    max_distance = 50
    opposition_variation = 0.4

    optimal_angle = np.arccos(optimal_opposition)  # Angle where the opposition is optimal
    delta_angle_check = optimal_angle/2  # Angle range to check around the optimal angle

    left_rotation_matrix = np.array([[np.cos(delta_angle_check), -np.sin(delta_angle_check)],
                                      [np.sin(delta_angle_check),  np.cos(delta_angle_check)]])
    
    right_rotation_matrix = np.array([[np.cos(-delta_angle_check), -np.sin(-delta_angle_check)],
                                      [np.sin(-delta_angle_check),  np.cos(-delta_angle_check)]])
    
    U_LEFT = left_rotation_matrix[0, 0] * U_UNIT + left_rotation_matrix[0, 1] * V_UNIT
    V_LEFT = left_rotation_matrix[1, 0] * U_UNIT + left_rotation_matrix[1, 1] * V_UNIT
    U_RIGHT = right_rotation_matrix[0, 0] * U_UNIT + right_rotation_matrix[0, 1] * V_UNIT
    V_RIGHT = right_rotation_matrix[1, 0] * U_UNIT + right_rotation_matrix[1, 1] * V_UNIT

    for i in prange(X.shape[0]):
        for j in range(X.shape[1]):
            if accessible_mask[i, j]:  # Only consider points with significant gradient
                ray_pixels = get_ray_pixels(X[i, j], Y[i, j], U_UNIT[i, j], V_UNIT[i, j], max_distance)
                ray_pixels += get_ray_pixels(X[i, j], Y[i, j], U_LEFT[i, j], V_LEFT[i, j], max_distance)
                ray_pixels += get_ray_pixels(X[i, j], Y[i, j], U_RIGHT[i, j], V_RIGHT[i, j], max_distance)
                for (rx, ry) in ray_pixels:
                    if 0 <= rx < cw and 0 <= ry < ch:  # Ensure we are within bounds
                        # get the gradient of target pixel and origin pixel
                        target_grad_x = U_UNIT[ry, rx]
                        target_grad_y = V_UNIT[ry, rx]
                        origin_grad_x = U_UNIT[i, j]
                        origin_grad_y = V_UNIT[i, j]

                        # calculate dot product of the gradients
                        dot_product = target_grad_x * origin_grad_x + target_grad_y * origin_grad_y

                        #optimal_opposition = 0
                        #opposition_variation = 0.5 
                        
                        opposition_score = -dot_product  # Higher when gradients are opposite
                        
                        distance_to_optimal = abs(opposition_score - optimal_opposition)
                        orientation_score = max(0, 1 - (distance_to_optimal / opposition_variation))  # Linear decay from 1 to 0 as we move away from optimal opposition

                        # orientation_score = max(-dot_product-0.3, 0)
                        # orientation_score = min(orientation_score, 0.3)
                        # orientation_score = 1 - (abs(orientation_score - 0.15)) / 0.15
                        #0 => 0, 0.5 => 0, 0.25 => 1
                        orientation_score = 1 - (abs(orientation_score - 0.15)) / 0.15

                        if orientation_score == 0:
                            continue

                        distance_score = max(0, 1 - (np.sqrt((rx - X[i, j])**2 + (ry - Y[i, j])**2) / max_distance))
                        #distance_score = 1

                        local_score = orientation_score * distance_score

                        # Calculate the dot product of the normalized gradients
                        target_mag = np.sqrt(target_grad_x**2 + target_grad_y**2) + 1e-5
                        origin_mag = np.sqrt(origin_grad_x**2 + origin_grad_y**2) + 1e-5
                        target_unit_x = target_grad_x / target_mag
                        target_unit_y = target_grad_y / target_mag

                        # target_center_x = (j + rx) // 2
                        # target_center_y = (i + ry) // 2

                        t = ((j - rx) * target_unit_x + (i - ry) * target_unit_y) / (target_unit_x**2 + target_unit_y**2 + 1e-5)
                        u = ((rx - j) * target_unit_x + (ry - i) * target_unit_y) / (target_unit_x**2 + target_unit_y**2 + 1e-5)
                        if t == 0 or u == 0:
                            continue

                        target_center_x = int(rx + t * target_unit_x)
                        target_center_y = int(ry + t * target_unit_y)

                        # make sure the target center is within bounds
                        if target_center_x < 0 or target_center_x >= cw or target_center_y < 0 or target_center_y >= ch:
                            continue

                        average_grad_x = (origin_grad_x + target_grad_x) / 2
                        average_grad_y = (origin_grad_y + target_grad_y) / 2

                        grad_angle = np.arctan2(-average_grad_x, average_grad_y) - np.pi/2  # Angle in radians
                        # Convert angle to [0, 2π]
                        if grad_angle < 0:
                            grad_angle += 2 * np.pi

                        # Determine the orientation bin
                        lower_index = int(grad_angle // (2 * np.pi / orientation_bins))
                        upper_index = (lower_index + 1) % orientation_bins
                        interpolation_factor = (grad_angle - lower_index * (2 * np.pi / orientation_bins)) / (2 * np.pi / orientation_bins)

                        # Distribute the score between the two bins
                        score_dir[target_center_y, target_center_x, lower_index] += local_score * (1 - interpolation_factor)
                        score_dir[target_center_y, target_center_x, upper_index] += local_score * interpolation_factor
                        

                        #score[target_center_y, target_center_x] += local_score  # Increment the score for each valid ray pixel


class ManualModeVib(Node):
    def __init__(self):
        super().__init__('manual_mode_vib')

        self.log_dir = os.environ.get('ROS_LOG_DIR') + "/../"

        # --- 1. ROS 2 Parameters ---
        # Allows you to tune the node via launch files without changing the code
        self.declare_parameter('anchor_file', '')
        self.declare_parameter('min_corridor_width', 1.0) # Min width of a valid door/corridor (m)
        self.declare_parameter('max_corridor_width', 3.0) # Max width to connect two corners (m)
        self.declare_parameter('corner_quality', 0.1)     # OpenCV Harris corner quality
        
        self.anchor_file = self.get_parameter('anchor_file').value
        self.min_width = self.get_parameter('min_corridor_width').value
        self.max_width = self.get_parameter('max_corridor_width').value
        self.quality = self.get_parameter('corner_quality').value

        # --- 2. State Variables ---
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.map_frame = "cabot/local_costmap_link" 

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.last_right_vib_pos_x = 0.0
        self.last_right_vib_pos_y = 0.0
        self.last_left_vib_pos_x = 0.0
        self.last_left_vib_pos_y = 0.0

        self.is_vibrating = False

        # --- 3. Publishers & Subscribers ---
        # Listens to the local costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishes raw vectors for the C++ Haptic driver: [ox, oy, dx, dy, ox2, oy2, ...]
        self.vector_pub = self.create_publisher(Float32MultiArray, '/cabot/intersection_vectors', 10)
        
        # Publishes green arrows for RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/cabot/intersection_markers', 10)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.pause_control_sub = self.create_subscription(std_msgs.msg.Bool, "/cabot/pause_control", self.pause_callback, 10)

        #leftVibPub = create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator4", 10);
        #rightVibPub = create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator3", 10);
        self.left_vib_pub = self.create_publisher(std_msgs.msg.UInt8, "/cabot/vibrator3", 10)
        self.right_vib_pub = self.create_publisher(std_msgs.msg.UInt8, "/cabot/vibrator4", 10)

        self.radar_visualizer = RadarVisualizer(np.ones((50, 50)), 25, 25) # Initial mag map and robot position

        self.get_logger().info("ManualModeVib node initialized. Waiting for costmap...")

    def pause_callback(self, msg):
        self.is_vibrating = msg.data

    def odom_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = transform.transform.translation
            quaternion = transform.transform.rotation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            self.odom_x = position.x
            self.odom_y = position.y
            self.odom_yaw = yaw
        except Exception as e:
            self.get_logger().warn(f"Could not transform base_link to map: {e}")


    def costmap_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.map_frame = msg.header.frame_id
        map_quaternion = (msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(map_quaternion)
        self.map_orientation = yaw
        
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0: 
            return

        grid = np.array(msg.data).reshape((h, w))
        
        # Exemple si robot au centre de la costmap
        robot_px = self.world_to_px(self.odom_x, self.odom_y)
        # Son angle par rapport à la map
        robot_yaw = self.odom_yaw - self.map_orientation # On soustrait l'orientation de la map pour avoir un angle relatif à la map 

        self.get_logger().info(f"Robot World Pos: ({self.odom_x:.2f}, {self.odom_y:.2f}), Yaw: {math.degrees(self.odom_yaw):.1f}° | Robot Px: ({robot_px[0]}, {robot_px[1]}), Yaw Rel: {math.degrees(robot_yaw):.1f}°")

        data_f = grid.astype(np.float32)

        # take 50x50 subgrid around the robot
        cw = 50
        ch = 50
        x_min = max(0, robot_px[0] - cw//2)
        x_max = min(w, robot_px[0] + cw//2)
        y_min = max(0, robot_px[1] - ch//2)
        y_max = min(h, robot_px[1] + ch//2)
        data_f = data_f[y_min:y_max, x_min:x_max]


        gx = cv2.Sobel(data_f, cv2.CV_32F, 1, 0, ksize=5)
        gy = cv2.Sobel(data_f, cv2.CV_32F, 0, 1, ksize=5)
        mag = cv2.magnitude(gx, gy)

        # we have the gradient, now, let's get the motion field. Use the robot position (500, 480) as the origin for calculating the motion field
        #robot_pos = (100, 75)

        # get Accessible pixels (where the cost is less than 200)
        accessible_mask = data_f < 90

        # Calculate the motion field vectors (normalized)
        # motion_field_x = gx / (mag + 1e-5)  # Avoid division by zero
        # motion_field_y = gy / (mag + 1e-5) # Avoid division by zero


        vector_field_x = -gx
        vector_field_y = -gy

        # get the motion field vectors every 5 pixels for visualization
        step = 1
        x_coords = np.arange(0, cw, step)
        y_coords = np.arange(0, ch, step)
        X, Y = np.meshgrid(x_coords, y_coords)
        U = vector_field_x[::step, ::step]
        V = vector_field_y[::step, ::step]

        # Remove vectors with very small magnitude for better visualization
        threshold = 100  # Adjust this threshold as needed
        mask = mag[::step, ::step] > threshold
        U = U * mask
        V = V * mask
        X = X * mask
        Y = Y * mask

        U_UNIT = U / (np.sqrt(U**2 + V**2) + 1e-5)  # Avoid division by zero
        V_UNIT = V / (np.sqrt(U**2 + V**2) + 1e-5)  # Avoid division by zero

        orientation_bins = 16

        max_distance = 30 # 50 pixels = 5m
        score_dir = np.zeros((ch, cw, orientation_bins), dtype=np.float32)  # Initialize score array

        voting_algorithm(X, Y, U_UNIT, V_UNIT, score_dir=score_dir, accessible_mask=accessible_mask[::step, ::step], cw=cw, ch=ch)

        # Now, we have a score for each pixel
        smooth_transfer_score = smooth_transfer(score_dir)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))

        inflated_img = cv2.dilate(smooth_transfer_score, kernel, iterations=1)
        inflated_img = inflated_img * accessible_mask[::step, ::step, np.newaxis]  # Apply the accessible mask to the inflated image
        inflated_img = cv2.blur(inflated_img, (10, 10))

        local_bins = inflated_img[25, 25, :]
        
        self.radar_visualizer.update_ui(25, 25, local_bins, new_mag_map=data_f, rotation_angle=(math.degrees(robot_yaw)))

        #angles = np.linspace(0, 2*np.pi, orientation_bins, endpoint=False)
        # angles = np.linspace(0, -2 * np.pi, orientation_bins, endpoint=False)
        # relative_angles = angles + robot_yaw

        # left_weights = np.maximum(0, np.sin(relative_angles)**4)
        # right_weights = np.maximum(0, -np.sin(relative_angles)**4)

        # left_score = np.sum(inflated_img[25, 25, :] * left_weights)
        # right_score = np.sum(inflated_img[25, 25, :] * right_weights)

        # 1. On remet la grille dans le même sens que le voting_algorithm (0=Nord, Horaire)
        angles = np.linspace(0, 2 * np.pi, orientation_bins, endpoint=False)

        target_left = (np.pi/2 + robot_yaw) % (2 * np.pi)
        target_right = (3*np.pi/2 + robot_yaw) % (2 * np.pi)

        # 3. Fonction magique pour calculer la distance angulaire la plus courte (gère le passage de 359° à 0°)
        def get_angular_dist(a1, a2):
            return np.abs(np.arctan2(np.sin(a1 - a2), np.cos(a1 - a2)))

        dist_to_left = get_angular_dist(angles, target_left)
        dist_to_right = get_angular_dist(angles, target_right)

        # 4. FILTRE SERRE : On définit une fenêtre d'ouverture (ex: 25 degrés max)
        # L'intensité décroît proprement de 1 à 0 si on s'éloigne de l'angle droit
        tolerance = np.deg2rad(25) 
        left_weights = np.maximum(0, 1 - (dist_to_left / tolerance))
        right_weights = np.maximum(0, 1 - (dist_to_right / tolerance))

        left_score = np.sum(inflated_img[int(25), int(25), :] * left_weights)
        right_score = np.sum(inflated_img[int(25), int(25), :] * right_weights)



        # # Get score on the left and on the right of the robot, ponderate close to center more than far from the center, use robot_yaw to determine which side is left and which side is right
        # left_score = np.sum(inflated_img[25, 25, :] * np.maximum(np.zeros_like(orientation_bins), np.sin(np.linspace(0, 2*np.pi, orientation_bins, endpoint=False) + robot_yaw)))
        # right_score = np.sum(inflated_img[25, 25, :] * np.maximum(np.zeros_like(orientation_bins), np.sin(np.linspace(0, 2*np.pi, orientation_bins, endpoint=False) + robot_yaw + np.pi)))

        # log left and right scores
        self.get_logger().info(f"Left Score: {left_score:.2f}, Right Score: {right_score:.2f}")

        # Publish the vibration intensity based on the scores
        left_vib_msg = std_msgs.msg.UInt8()
        right_vib_msg = std_msgs.msg.UInt8()
        max_vib_intensity = 255
        total_score = 3
        left_vib_msg.data = max(0, min(255, int((left_score / total_score) * max_vib_intensity)) - 3)
        right_vib_msg.data = max(0, min(255, int((right_score / total_score) * max_vib_intensity)) - 3)

        # Threshold of 10
        if not self.is_vibrating:
            left_vib_msg.data = 0
            right_vib_msg.data = 0
            return

        # Cap the vibration intensity to avoid overwhelming the user
        max_allowed_vib_intensity = 20
        left_vib_msg.data = min(left_vib_msg.data, max_allowed_vib_intensity)
        right_vib_msg.data = min(right_vib_msg.data, max_allowed_vib_intensity)

        distance_to_previous_right_vib = np.sqrt((self.last_right_vib_pos_x - self.odom_x)**2 + (self.last_right_vib_pos_y - self.odom_y)**2)
        distance_to_previous_left_vib = np.sqrt((self.last_left_vib_pos_x - self.odom_x)**2 + (self.last_left_vib_pos_y - self.odom_y)**2)

        # Only publish if the robot has moved at least 0.2m from the last vibration position to avoid spamming vibrations when the robot is stuck
        if distance_to_previous_right_vib > 0.5 and right_vib_msg.data > 0:
            self.right_vib_pub.publish(right_vib_msg)
            self.last_right_vib_pos_x = self.odom_x
            self.last_right_vib_pos_y = self.odom_y

        if distance_to_previous_left_vib > 0.5 and left_vib_msg.data > 0:
            self.left_vib_pub.publish(left_vib_msg)
            self.last_left_vib_pos_x = self.odom_x
            self.last_left_vib_pos_y = self.odom_y



        
    def world_to_px(self, world_x, world_y):
        # 1. Translation
        dx = world_x - self.origin_x
        dy = world_y - self.origin_y
        
        # 2. Dé-rotation (on tourne dans le sens inverse du yaw de la map)
        cos_theta = math.cos(-self.map_orientation)
        sin_theta = math.sin(-self.map_orientation)
        
        x_rel = dx * cos_theta - dy * sin_theta
        y_rel = dx * sin_theta + dy * cos_theta
        
        # 3. Scaling
        px_x = int(x_rel / self.resolution)
        px_y = int(y_rel / self.resolution)
        
        return np.array([px_x, px_y])

    def px_to_world(self, pt_px):
        # 1. Mise à l'échelle (pixels -> mètres)
        x_rel = pt_px[0] * self.resolution
        y_rel = pt_px[1] * self.resolution

        # 2. Application de la rotation (Matrice de rotation 2D)
        cos_theta = math.cos(self.map_orientation)
        sin_theta = math.sin(self.map_orientation)
        
        rotated_x = x_rel * cos_theta - y_rel * sin_theta
        rotated_y = x_rel * sin_theta + y_rel * cos_theta

        # 3. Translation (Ajout de l'origine)
        world_x = self.origin_x + rotated_x
        world_y = self.origin_y + rotated_y
        
        return [world_x, world_y]

def main(args=None):
    rclpy.init(args=args)
    node = ManualModeVib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    pass

if __name__ == '__main__':
    main()

