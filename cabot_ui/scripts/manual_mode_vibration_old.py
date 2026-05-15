#!/usr/bin/env python3

import datetime
import os

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

        #leftVibPub = create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator4", 10);
        #rightVibPub = create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator3", 10);
        self.left_vib_pub = self.create_publisher(std_msgs.msg.UInt8, "/cabot/vibrator3", 10)
        self.right_vib_pub = self.create_publisher(std_msgs.msg.UInt8, "/cabot/vibrator4", 10)

        self.get_logger().info("ManualModeVib node initialized. Waiting for costmap...")

    def detect_side_intersection(self, grid, robot_world_pos, robot_world_yaw, side="right"):
        self.get_logger().info(f"--- BEAM CAST {side.upper()} START ---")
        
        # 1. Paramètres
        num_rays = 50
        beam_width = 0.8
        max_range = 2.5
        vote_threshold = 0.8
        
        angle_offset = -np.pi/2 if side == "right" else np.pi/2
        probe_angle = robot_world_yaw + angle_offset
        grid_angle = probe_angle - self.map_orientation
        
        self.get_logger().info(f"[1] RobotYaw: {math.degrees(robot_world_yaw):.1f}°, ProbeAngle: {math.degrees(probe_angle):.1f}°, GridAngle: {math.degrees(grid_angle):.1f}°")

        # 2. Points de départ des rayons (Monde -> Pixels)
        offsets = np.linspace(-beam_width/2, beam_width/2, num_rays)
        start_x = robot_world_pos[0] + offsets * np.cos(robot_world_yaw)
        start_y = robot_world_pos[1] + offsets * np.sin(robot_world_yaw)
        
        dx = start_x - self.origin_x
        dy = start_y - self.origin_y
        cos_m, sin_m = np.cos(-self.map_orientation), np.sin(-self.map_orientation)
        
        start_px_x = ((dx * cos_m - dy * sin_m) / self.resolution).astype(int) #todo reprendre ici
        start_px_y = ((dx * sin_m + dy * cos_m) / self.resolution).astype(int)
        
        self.get_logger().info(f"[2] Beam Start Px: de ({start_px_x[0]}, {start_px_y[0]}) à ({start_px_x[-1]}, {start_px_y[-1]})")

        # 3. Génération de la tranche
        steps = np.arange(0, max_range, self.resolution)
        cos_g, sin_g = np.cos(grid_angle), np.sin(grid_angle)
        step_grid, _ = np.meshgrid(steps / self.resolution, range(num_rays))
        
        all_px_x = (start_px_x[:, np.newaxis] + step_grid * cos_g).astype(int)
        all_px_y = (start_px_y[:, np.newaxis] + step_grid * sin_g).astype(int)
        
        self.get_logger().info(f"[3] Grid Shape: {all_px_x.shape} (Rayons x Steps), CostMap Shape: {grid.shape}")

        # 4. Sampling et Vote
        h, w = grid.shape
        valid_mask = (all_px_x >= 0) & (all_px_x < w) & (all_px_y >= 0) & (all_px_y < h)
        
        # Log the grid in the logs
        for i in range(num_rays):
            row_str = ""
            for j in range(len(steps)):
                if valid_mask[i, j]:
                    cell_value = grid[all_px_y[i, j], all_px_x[i, j]]
                    row_str += f"{cell_value:3d} "
                else:
                    row_str += "XXX "
            self.get_logger().info(f"Ray {i+1:02d}: {row_str}")
        
        num_out = np.sum(~valid_mask)
        if num_out > 0:
            self.get_logger().info(f"[4] {num_out} points sont hors des limites de la costmap")

        beam_results = np.zeros((num_rays, len(steps)))
        beam_results[valid_mask] = grid[all_px_y[valid_mask], all_px_x[valid_mask]] < 20
        
        votes = np.sum(beam_results, axis=0)
        max_vote = np.max(votes)
        is_corridor_voted = votes >= (num_rays * vote_threshold)
        
        self.get_logger().info(f"[4] Max Votes: {max_vote}/{num_rays} (Seuil requis: {int(num_rays * vote_threshold)})")

        # 5. Analyse des Gaps
        diff = np.diff(is_corridor_voted.astype(int))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        
        self.get_logger().info(f"[5] Candidats trouvés: {len(starts)} zones libres détectées après vote.")

        intersections = []
        for i, s_idx in enumerate(starts):
            possible_ends = ends[ends > s_idx]
            if len(possible_ends) > 0:
                e_idx = possible_ends[0]
                width_m = (e_idx - s_idx) * self.resolution
                dist_from_robot = s_idx * self.resolution
                
                log_prefix = f"    - Zone {i+1} [dist: {dist_from_robot:.2f}m, width: {width_m:.2f}m]"
                
                if 0.5 < width_m < 2.0:
                    mid_idx = (s_idx + e_idx) // 2
                    mid_x_px = all_px_x[num_rays // 2, mid_idx]
                    mid_y_px = all_px_y[num_rays // 2, mid_idx]
                    
                    origin_m = self.px_to_world([mid_x_px, mid_y_px])
                    normal = np.array([np.cos(probe_angle), np.sin(probe_angle)])
                    intersections.append((origin_m, normal))
                    
                    self.get_logger().info(f"{log_prefix} : ✅ VALIDÉ")
                else:
                    reason = "TROP ÉTROIT" if width_m <= 0.5 else "TROP LARGE"
                    self.get_logger().info(f"{log_prefix} : ❌ REJETÉ ({reason})")

        # 6. Vibration Logic
        if len(intersections) > 0:
            # On ne vibre que si on a trouvé quelque chose dans CE scan spécifique
            self.get_logger().info(f"!!! VIBRATION {side.upper()} ACTIVE (20) !!!")
            if side == "left":
                self.left_vib_pub.publish(UInt8(data=20))
            else:
                self.right_vib_pub.publish(UInt8(data=20))
        else:
            # Optionnel: On pourrait couper la vibration ici, mais attention si l'autre côté détecte
            pass

        self.get_logger().info(f"--- BEAM CAST {side.upper()} END ({len(intersections)} found) ---")
        return intersections

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
        robot_yaw = self.odom_yaw - self.map_orientation  # On soustrait l'orientation de la map pour avoir un angle relatif à la map 

        self.get_logger().info(f"Robot World Pos: ({self.odom_x:.2f}, {self.odom_y:.2f}), Yaw: {math.degrees(self.odom_yaw):.1f}° | Robot Px: ({robot_px[0]}, {robot_px[1]}), Yaw Rel: {math.degrees(robot_yaw):.1f}°")

        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        folder_name = os.path.join(self.log_dir,"costmap_debug")
        self.get_logger().info(f"Saving debug costmaps to: {folder_name}")
        os.makedirs(folder_name, exist_ok=True)
        grid.tofile(os.path.join(folder_name, f"costmap_{current_time}.raw"))

        # Scan
        all_intersections = []
        all_intersections.extend(self.detect_side_intersection(grid, robot_px, robot_yaw, side="right"))
        all_intersections.extend(self.detect_side_intersection(grid, robot_px, robot_yaw, side="left"))
        
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

    def publish_data(self, data_list, is_intersection=True):
        """Publie les données pour le driver et les marqueurs RViz."""
        if not data_list:
            return

        # 1. Publish Raw Array (seulement pour les intersections)
        if is_intersection:
            vector_msg = Float32MultiArray()
            for origin, direct in data_list:
                vector_msg.data.extend([float(origin[0]), float(origin[1]), float(direct[0]), float(direct[1])])
            self.vector_pub.publish(vector_msg)

        # 2. Publish RViz Markers
        marker_array = MarkerArray()
        
        # On définit le namespace et le type selon la donnée
        ns = "intersections" if is_intersection else "corners"
        m_type = Marker.ARROW if is_intersection else Marker.SPHERE
        
        # On ne clean que le namespace concerné
        clear_marker = Marker()
        clear_marker.header.frame_id = self.map_frame
        clear_marker.ns = ns
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, (origin, direct) in enumerate(data_list):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = i
            marker.type = m_type
            marker.action = Marker.ADD
            
            # Position : On met Z à 0.2 pour éviter que la map ne cache le marqueur
            marker.pose.position.x = float(origin[0])
            marker.pose.position.y = float(origin[1])
            marker.pose.position.z = 0.2 
            
            if is_intersection:
                # Orientation de la flèche
                angle = math.atan2(direct[1], direct[0])
                marker.pose.orientation.z = math.sin(angle / 2.0)
                marker.pose.orientation.w = math.cos(angle / 2.0)
                marker.scale.x, marker.scale.y, marker.scale.z = 0.6, 0.1, 0.1
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.1, 0.9 # Vert
            else:
                # Taille de la sphère pour les coins
                marker.scale.x, marker.scale.y, marker.scale.z = 0.15, 0.15, 0.15
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.1, 0.1, 1.0 # Rouge
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

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



    # --- A. Image Processing ---
        # grid = np.array(msg.data).reshape((h, w))

        # # Seuil à 100 pour ignorer l'inflation et ne garder que le mur réel
        # _, binary = cv2.threshold(grid.astype(np.uint8), 90, 255, cv2.THRESH_BINARY)

        # # Inverse pour que les murs soient blancs (255) et le reste noir (0)
        # binary = cv2.bitwise_not(binary)

        # # Save for debugging
        # current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        # folder_name = os.path.join(self.log_dir,"binary_debug")
        # self.folder_name = folder_name
        # self.get_logger().info(f"Saving debug images to: {folder_name}")
        # os.makedirs(folder_name, exist_ok=True)
        # cv2.imwrite(
        #     os.path.join(folder_name, f"binary_{current_time}.png"),
        #     binary
        # )

        # # --- B. Corner Detection via Approximation ---
        # contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # # Draw contours
        # debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        # cv2.drawContours(debug_img, contours, -1, (0, 255, 0), 2)
        # cv2.imwrite(
        #     os.path.join(folder_name, f"contours_{current_time}.png"),
        #     debug_img
        # )

        # all_corners = []

        # for cnt in contours:
        #     # epsilon : ajuster entre 0.01 et 0.05 selon la précision voulue.
        #     # Plus epsilon est grand, plus on ignore les arrondis de l'inflation.
        #     epsilon = 0.02 * cv2.arcLength(cnt, True)
        #     approx = cv2.approxPolyDP(cnt, epsilon, True)
            
        #     # On extrait les points [x, y]
        #     for point in approx:
        #         all_corners.append(point[0])

        # # Conversion finale en tableau NumPy (N, 2)
        # corners = np.array(all_corners)


        # intersections = []
        # corner_displays = []

        # # --- C. Vector Extraction ---
        # if corners is not None:
        #     pts = corners.reshape(-1, 2)
        #     robot_px = np.array([w // 2, h // 2]) # Robot is typically at the center

        #     # Try to pair every corner with every other corner
        #     for i in range(len(pts)):
        #         corner_displays.append((self.px_to_world(pts[i]), [0,1])) # For RViz display only

        #         for j in range(i + 1, len(pts)):
        #             p1, p2 = pts[i], pts[j]
                    
        #             dist_px = np.linalg.norm(p1 - p2)
        #             dist_m = dist_px * self.resolution
                    
        #             # If the distance matches a standard doorway or corridor
        #             if self.min_width < dist_m < self.max_width:
        #                 # 1. Origin is the midpoint of the gap
        #                 mid_px = (p1 + p2) / 2.0
                        
        #                 # 2. Normal vector of the segment
        #                 seg = p2 - p1
        #                 normal = np.array([-seg[1], seg[0]]).astype(float)
        #                 norm_length = np.linalg.norm(normal)
                        
        #                 if norm_length == 0: continue
        #                 normal /= norm_length
                        
        #                 # 3. Ensure the vector points AWAY from the robot (into the corridor)
        #                 vec_to_mid = mid_px - robot_px
        #                 if np.dot(normal, vec_to_mid) < 0:
        #                     normal = -normal
                        
        #                 # 4. Convert origin back to Map/World coordinates
        #                 origin_m = self.px_to_world(mid_px)
        #                 intersections.append((origin_m, normal))

        # # --- D. Publish Data ---
        # self.get_logger().info(f"Found {len(intersections)} int. and {len(corner_displays)} corners.")

        # # On publie les coins en rouge (sphères)
        # self.publish_data(corner_displays, is_intersection=False)
        
        # # On publie les intersections en vert (flèches)
        # self.publish_data(intersections, is_intersection=True)