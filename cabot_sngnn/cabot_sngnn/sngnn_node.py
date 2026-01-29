#!/usr/bin/env python3

import sys
import os
import json
import copy
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from people_msgs.msg import People
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point, do_transform_pose
import numpy as np
import math
import time
import cv2

# Add SNGNN2D-v2 to path
SNGNN_PATH = os.path.join(os.environ.get('HOME', '/home/developer'), 'ros2_ws/src/SNGNN2D-v2')
sys.path.append(SNGNN_PATH)
sys.path.append(os.path.join(SNGNN_PATH, 'utils'))
sys.path.append(os.path.join(SNGNN_PATH, 'dataset'))
sys.path.append(os.path.join(SNGNN_PATH, 'nets'))

try:
    from utils.socnav2d_V2_API import SocNavAPI
    from dataset.socnav2d_dataset import SocNavDataset
except ImportError as e:
    print(f"Error importing SNGNN: {e}")
    # Fallback or exit?
    # sys.exit(1)

class SNGNNNode(Node):
    def __init__(self):
        super().__init__('sngnn_node')

        self.declare_parameter('model_path', os.path.join(SNGNN_PATH, 'model'))
        self.model_path = self.get_parameter('model_path').value

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').value

        self.declare_parameter('global_frame', 'map')
        self.global_frame = self.get_parameter('global_frame').value
        self.declare_parameter('goal_topic', '/plan')
        self.goal_topic = self.get_parameter('goal_topic').value
        self.declare_parameter('map_topic', '/map')
        self.map_topic = self.get_parameter('map_topic').value
        self.declare_parameter('occupied_threshold', 65)
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.declare_parameter('wall_stride', 1)
        self.wall_stride = self.get_parameter('wall_stride').value
        self.declare_parameter('people_track_timeout', 1.5)
        self.people_track_timeout = self.get_parameter('people_track_timeout').value
        self.declare_parameter('save_image', True)
        self.save_image = self.get_parameter('save_image').value
        self.declare_parameter('save_interval_sec', 1.0)
        self.save_interval_sec = self.get_parameter('save_interval_sec').value
        self.declare_parameter('output_dir', os.path.join(os.environ.get('HOME', '/home/developer'), '.ros', 'log', 'sngnn_costmap'))
        self.output_dir = self.get_parameter('output_dir').value
        self.declare_parameter('output_resolution', 0.05)
        self.output_resolution = self.get_parameter('output_resolution').value
        self.declare_parameter('output_width_m', 13.0)
        self.output_width_m = self.get_parameter('output_width_m').value
        self.declare_parameter('output_height_m', 13.0)
        self.output_height_m = self.get_parameter('output_height_m').value
        self.declare_parameter('crop_with_map', True)
        self.crop_with_map = self.get_parameter('crop_with_map').value

        self.callback_group = ReentrantCallbackGroup()

        # Initialize SNGNN
        self.device = 'cpu'
        try:
            import torch
            if torch.cuda.is_available():
                self.device = 'cuda'
                self.get_logger().info("CUDA is available. Using GPU.")
            else:
                self.get_logger().info("CUDA is not available. Using CPU.")
        except ImportError:
            self.get_logger().warn("Could not import torch. Using CPU.")
        
        try:
            self.get_logger().info(f"Loading model from {self.model_path}")
            self.api = SocNavAPI(base=self.model_path, device=self.device)
            self.net = self.api.net
            self.get_logger().info("Model loaded")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.api = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_people = None
        self.latest_odom = None
        self.latest_cmd_vel = None # Need this? Or use odom.twist
        self.latest_goal = None
        self.latest_map = None
        self.people_tracks = {}
        self.last_save_time = 0.0

        self.create_subscription(People, '/people', self.people_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Path, self.goal_topic, self.goal_callback, 10, callback_group=self.callback_group)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10, callback_group=self.callback_group)
        
        # Publisher for costmap
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/sngnn_costmap', 10)

        # History buffer: list of (timestamp, robot_pose, people_list)
        self.history = []
        self.N_INTERVALS = 3
        self.FRAMES_INTERVAL = 1.0 # seconds

        self.is_processing = False

        self.timer = self.create_timer(0.2, self.loop, callback_group=self.callback_group) # 5Hz

    def people_callback(self, msg):
        self.latest_people = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def map_callback(self, msg):
        self.latest_map = msg

    def goal_callback(self, msg):
        if not msg.poses:
            return
        self.latest_goal = msg.poses[-1]

    def get_transform(self, target, source, time):
        try:
            # Use 0 timestamp for latest transform if time is Time(nanoseconds=0)
            return self.tf_buffer.lookup_transform(target, source, time, rclpy.time.Duration(seconds=0.1))
        except Exception as e:
            # self.get_logger().warn(f"TF lookup failed {source} -> {target}: {e}")
            return None

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def build_walls_from_map(self, now, robot_x, robot_y):
        """
        Build walls from the map, cropping to robot vicinity and simplifying using contour approximation.
        This reduces the number of wall segments by merging collinear points.
        """
        if self.latest_map is None:
            return []

        res = self.latest_map.info.resolution
        width = self.latest_map.info.width
        height = self.latest_map.info.height
        origin = self.latest_map.info.origin
        
        # Determine scan range based on robot position and output size
        # Add margin to ensure we cover the features
        margin = 2.0
        min_x_m = robot_x - self.output_width_m / 2.0 - margin
        max_x_m = robot_x + self.output_width_m / 2.0 + margin
        min_y_m = robot_y - self.output_height_m / 2.0 - margin
        max_y_m = robot_y + self.output_height_m / 2.0 + margin
        
        min_ix = int((min_x_m - origin.position.x) / res)
        max_ix = int((max_x_m - origin.position.x) / res)
        min_iy = int((min_y_m - origin.position.y) / res)
        max_iy = int((max_y_m - origin.position.y) / res)
        
        min_ix = max(0, min_ix)
        max_ix = min(width, max_ix)
        min_iy = max(0, min_iy)
        max_iy = min(height, max_iy)

        if min_ix >= max_ix or min_iy >= max_iy:
            return []

        # Extract ROI efficiently from 1D map data
        # Creating a numpy array from the full list is slow, so we construct from sliced rows.
        raw_data = self.latest_map.data
        rows = []
        roi_width = max_ix - min_ix
        
        for y in range(min_iy, max_iy):
            start = y * width + min_ix
            end = start + roi_width
            rows.append(raw_data[start:end])
        
        # Convert to numpy array for OpenCV
        roi = np.array(rows, dtype=np.int8)
        
        # Create binary image
        # occupied_threshold (e.g., 65) vs map data (0-100, -1)
        binary_img = np.zeros_like(roi, dtype=np.uint8)
        binary_img[roi >= self.occupied_threshold] = 255
        
        # Find contours
        # RETR_EXTERNAL: retrieves only the extreme outer contours. 
        # CHAIN_APPROX_SIMPLE: encodes vertical, horizontal, and diagonal segments
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        walls = []
        
        for cnt in contours:
            # Approximate contour with accuracy proportional to perimeter
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            # Ensure minimal simplification to avoid over-smoothing useful details if needed, 
            # but usually dynamic epsilon is best.
            
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            pts = approx.reshape(-1, 2)
            n_pts = len(pts)
            
            if n_pts < 2:
                continue
                
            for i in range(n_pts):
                # Get start and end points of the current segment
                p1 = pts[i]
                p2 = pts[(i + 1) % n_pts]
                
                # Transform pixel coordinates (in ROI) back to Global Map Coordinates (meters)
                # Pixel center is at integer + 0.5
                wx1 = origin.position.x + (min_ix + p1[0]) * res
                wy1 = origin.position.y + (min_iy + p1[1]) * res
                wx2 = origin.position.x + (min_ix + p2[0]) * res
                wy2 = origin.position.y + (min_iy + p2[1]) * res
                
                # Add wall segment
                walls.append({
                    'x1': wx1,
                    'y1': wy1,
                    'x2': wx2,
                    'y2': wy2,
                })
        return walls

    def get_robot_in_map(self):
        if self.latest_map is None:
            return None
        trans = self.get_transform(self.latest_map.header.frame_id, self.target_frame, rclpy.time.Time())
        if trans is None:
            return None
        return (trans.transform.translation.x, trans.transform.translation.y)

    def crop_output_to_local(self, ret):
        # With local costmap input, we might not need cropping based on global map position.
        # But we might want to mask areas outside the costmap if SNGNN predicts there?
        # For now, if we assume input map covers the ROI, we can just pass through or 
        # implement simple masking if useful.
        # Disabling legacy global map crop logic.
        return ret

        # Legacy code for reference/restoration if needed:
        """
        if not self.crop_with_map or self.latest_map is None:
            return ret
        robot_xy = self.get_robot_in_map()
        ...
        """

    def loop(self):
        if self.api is None:
            return
        
        # Best effort: Skip if processing
        if self.is_processing:
            return

        self.is_processing = True
        try:
            self._process()
        except Exception as e:
            self.get_logger().error(f"Error in process loop: {e}")
        finally:
            self.is_processing = False

    def _process(self):
        # 1. Capture current state
        now = self.get_clock().now()
        
        if self.latest_odom is None:
            return

        # We need robot command (velocity)
        # Assuming current odom twist is the command result (or close enough)
        v = self.latest_odom.twist.twist.linear.x
        w = self.latest_odom.twist.twist.angular.z

        # Get robot pose in map frame
        robot_x = 0.0
        robot_y = 0.0
        robot_yaw = 0.0
        # Use Time() (zero) to get latest available transform
        # Creating explicit Time object to ensure it is interpreted as latest
        trans = self.get_transform(self.global_frame, self.target_frame, rclpy.time.Time())
        if trans:
             robot_x = trans.transform.translation.x
             robot_y = trans.transform.translation.y
             robot_yaw = self.quat_to_yaw(trans.transform.rotation)
        
        current_people = []
        
        # Use simple people tracking directly in global frame (or whatever frame people are in)
        # Assuming people message is in 'map' or 'odom' which is consistent.
        
        if self.latest_people:
            for idx, p in enumerate(self.latest_people.people):
                # No transform to base_link
                # Just track in the original frame
                
                person_key = p.name if p.name else f"idx_{idx}"
                now_sec = now.nanoseconds / 1e9
                prev = self.people_tracks.get(person_key)
                
                if prev:
                    dt = max(0.001, now_sec - prev['t'])
                    vx = (p.position.x - prev['x']) / dt
                    vy = (p.position.y - prev['y']) / dt
                    last_orientation = prev.get('a', 0.0)
                else:
                    vx = 0.0
                    vy = 0.0
                    last_orientation = 0.0
                
                self.people_tracks[person_key] = {
                    't': now_sec,
                    'x': p.position.x,
                    'y': p.position.y,
                    'vx': vx,
                    'vy': vy,
                    'a': last_orientation
                }
                
                speed = math.hypot(vx, vy)
                orientation = math.atan2(vy, vx) if speed > 1e-3 else last_orientation
                self.people_tracks[person_key]['a'] = orientation
                self.people_tracks[person_key]['vx'] = vx
                self.people_tracks[person_key]['vy'] = vy

        from geometry_msgs.msg import PointStamped
        
        now_sec = now.nanoseconds / 1e9
        for key, track in list(self.people_tracks.items()):
            if now_sec - track['t'] > self.people_track_timeout:
                continue
            
            # Use data directly
            x = track['x']
            y = track['y']
            vx = track['vx']
            vy = track['vy']
            a = track['a']

            current_people.append({
                'id': 0,
                'key': key,
                'x': x,
                'y': y,
                'vx': vx,
                'vy': vy,
                'va': 0,
                'a': a,
            })
        # SNGNN expects compact IDs for people nodes.
        for idx, person in enumerate(current_people, start=1):
            person['id'] = idx
        
        # Use the current cabot goal if available; otherwise keep a zero goal.
        goal = [{'x': 0.0, 'y': 0.0}]
        if self.latest_goal is not None:
             goal = [{'x': self.latest_goal.pose.position.x, 'y': self.latest_goal.pose.position.y}]

        walls = self.build_walls_from_map(now, robot_x, robot_y)

        frame_data = {
            'ID': int(now.nanoseconds / 1e9),
            'timestamp': now.nanoseconds / 1e9,
            'command': [v, 0, w], # m/s ?
            'people': current_people,
            'objects': [],
            'walls': walls,
            'interaction': [],
            'goal': goal,
            'robot_pose': {
                'x': robot_x,
                'y': robot_y,
                'a': robot_yaw
            }
        }

        self.history.append(frame_data)
        
        # Maintain history
        # We need samples at t, t-1, t-2
        # Prune old
        while len(self.history) > 0 and self.history[0]['timestamp'] < (now.nanoseconds/1e9 - self.N_INTERVALS * self.FRAMES_INTERVAL - 1.0):
             self.history.pop(0)

        # Select frames
        # Find frame closest to t, t-1, t-2
        selected_frames = []
        current_time = now.nanoseconds / 1e9
        
        for i in range(self.N_INTERVALS):
            target_time = current_time - i * self.FRAMES_INTERVAL
            # Find closest in history
            best_frame = None
            min_dt = 1000.0
            for f in self.history:
                dt = abs(f['timestamp'] - target_time)
                if dt < min_dt:
                    min_dt = dt
                    best_frame = f
            
            if best_frame and min_dt < 0.5: # Tolerance
                selected_frames.append(best_frame)
            else:
                # Missing history
                pass

        if len(selected_frames) < self.N_INTERVALS:
            # duplicate oldest or fill?
            if len(selected_frames) > 0:
                 while len(selected_frames) < self.N_INTERVALS:
                     # Copy last frame and shift timestamp to satisfy dataset requirements
                     prev_frame = selected_frames[-1]
                     new_frame = copy.deepcopy(prev_frame)
                     new_frame['timestamp'] = prev_frame['timestamp'] - self.FRAMES_INTERVAL
                     selected_frames.append(new_frame)
            else:
                return # Not enough data

        # Prepare output frames by deep copying selected frames
        # This preserves the original history with map coordinates
        output_frames = [copy.deepcopy(f) for f in selected_frames]
        
        # The first frame is the latest (current) frame
        # We use its robot pose as the origin for the local coordinate system
        ref_frame = output_frames[0]
        ref_x = ref_frame['robot_pose']['x']
        ref_y = ref_frame['robot_pose']['y']
        ref_a = ref_frame['robot_pose']['a']
        
        # Rotation matrix for global-to-local transformation
        # Rotate by -ref_a
        c = math.cos(ref_a)
        s = math.sin(ref_a)

        def transform_frame(frame):
            # 1. Transform Robot Pose
            dx = frame['robot_pose']['x'] - ref_x
            dy = frame['robot_pose']['y'] - ref_y
            frame['robot_pose']['x'] = dx * c + dy * s
            frame['robot_pose']['y'] = -dx * s + dy * c
            
            da = frame['robot_pose']['a'] - ref_a
            frame['robot_pose']['a'] = math.atan2(math.sin(da), math.cos(da))

            # 2. Transform People
            for p in frame['people']:
                dx = p['x'] - ref_x
                dy = p['y'] - ref_y
                p['x'] = dx * c + dy * s
                p['y'] = -dx * s + dy * c
                
                dvx = p['vx']
                dvy = p['vy']
                p['vx'] = dvx * c + dvy * s
                p['vy'] = -dvx * s + dvy * c
                
                da = p['a'] - ref_a
                p['a'] = math.atan2(math.sin(da), math.cos(da))

            # 3. Transform Goal
            for g in frame['goal']:
                dx = g['x'] - ref_x
                dy = g['y'] - ref_y
                g['x'] = dx * c + dy * s
                g['y'] = -dx * s + dy * c

        # Transform Frame 0 first to establish reference nodes
        transform_frame(output_frames[0])

        # 4. Transform Walls for Frame 0 only
        # We will share these transformed walls with all history frames to ensure consistent graph nodes
        for w in output_frames[0]['walls']:
            dx1 = w['x1'] - ref_x
            dy1 = w['y1'] - ref_y
            w['x1'] = dx1 * c + dy1 * s
            w['y1'] = -dx1 * s + dy1 * c
            
            dx2 = w['x2'] - ref_x
            dy2 = w['y2'] - ref_y
            w['x2'] = dx2 * c + dy2 * s
            w['y2'] = -dx2 * s + dy2 * c
            
        ref_walls = output_frames[0]['walls']
        
        # Map people in Frame 0 to consistent IDs.
        # Key: track identifier (name), Value: Node ID (1..N)
        # Also map key to the person object in Frame 0 for fallback
        people_key_to_id = {}
        people_key_to_obj = {}
        for idx, person in enumerate(output_frames[0]['people'], start=1):
            person['id'] = idx
            if 'key' in person:
                people_key_to_id[person['key']] = idx
                people_key_to_obj[person['key']] = person

        # Process remaining history frames
        for i in range(1, len(output_frames)):
            frame = output_frames[i]
            transform_frame(frame)
            
            # Use Reference Walls
            frame['walls'] = copy.deepcopy(ref_walls)
            
            # Filter People: Only keep those present in Frame 0 (Graph Nodes)
            # And ensure IDs match. If missing, use Frame 0 as dummy (occlusion handling)
            new_people = []
            
            # Create a map of people present in THIS history frame
            existing_people_map = {p.get('key'): p for p in frame['people']}
            
            # Iterate over valid keys determined by Frame 0
            # This ensures 'new_people' has EXACTLY the same length and order as output_frames[0]['people']
            # which guarantees 'max_used_id' increments consistently in initializeAlt8, preventing Goal/Wall ID shifts.
            for key_0, id_0 in people_key_to_id.items():
                if key_0 in existing_people_map:
                    # Person exists in this history frame
                    p = existing_people_map[key_0]
                    p['id'] = id_0
                    new_people.append(p)
                else:
                    # Person missing in this history frame (e.g. appeared recently)
                    # Use Frame 0 data as dummy to preserve node count/indices
                    # Deep copy to permit modification if needed, though we just assume they stay at last seen pos
                    dummy = copy.deepcopy(people_key_to_obj[key_0])
                    # dummy['id'] is already id_0
                    new_people.append(dummy)

            frame['people'] = new_people

        # Prepare for SNGNN
        # input is a list of dicts (frames)
        
        # Create Graph
        try:
            # SocNavDataset expects raw_data_path or list of dicts?
            # Constructor: __init__(self, path, alt, net, mode='train', raw_dir='data/', prev_graph = None, init_line=-1, end_line=-1, loc_limit=limit, force_reload=False, verbose=False, debug=False, device = 'cpu')
            # If path is NOT str, it assumes it is data?
            # In test.py: graph = SocNavDataset(data, ...) where data is loaded json (list of frames)
            
            graph = SocNavDataset(output_frames, alt='8', net=self.net, mode='test', raw_dir='', device=self.device)
            ret = self.api.predictOneGraph(graph)[0]
            
            # Process output
            # ret is tensor reshaped to 256x256
            image_width = 256
            ret = ret.reshape(image_width, image_width)
            ret = ret.cpu().detach().numpy()
            
            raw_ret = ret.copy()

            ret = self.crop_output_to_local(ret)
            output_width_px = max(1, int(round(self.output_width_m / self.output_resolution)))
            output_height_px = max(1, int(round(self.output_height_m / self.output_resolution)))
            ret = cv2.resize(ret, (output_width_px, output_height_px), interpolation=cv2.INTER_CUBIC)
            
            # Convert to nav_msgs/OccupancyGrid
            # ret is float32 numpy array with range [-1, 1] (SNGNN output)
            
            grid_msg = OccupancyGrid()
            grid_msg.header.frame_id = self.target_frame
            grid_msg.header.stamp = now.to_msg()
            grid_msg.info.resolution = self.output_resolution
            grid_msg.info.width = ret.shape[1]
            grid_msg.info.height = ret.shape[0]
            
            # Origin is bottom-left. The costmap is centered on the robot in base_link.
            # So origin is (-width_m / 2, -height_m / 2)
            grid_msg.info.origin.position.x = - (grid_msg.info.width * grid_msg.info.resolution) / 2.0
            grid_msg.info.origin.position.y = - (grid_msg.info.height * grid_msg.info.resolution) / 2.0
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0

            # Scale [-1, 1] to [0, 100]
            grid_data = (1.0 - ret) / 2.0 * 100.0
            grid_data = np.clip(grid_data, 0, 100).astype(np.int8)
            grid_msg.data = grid_data.flatten().tolist()
            
            self.costmap_pub.publish(grid_msg)
            
            if self.save_image:
                now_sec = now.nanoseconds / 1e9
                if now_sec - self.last_save_time >= self.save_interval_sec:
                    os.makedirs(self.output_dir, exist_ok=True)
                    # Convert raw (-1 to 1) to 0-255
                    img = np.clip((raw_ret + 1.0) / 2.0 * 255.0, 0, 255).astype(np.uint8)
                    timestamp = int(now_sec * 1000)
                    out_path = os.path.join(self.output_dir, f"sngnn_costmap_{timestamp}.png")
                    json_path = os.path.join(self.output_dir, f"sngnn_input_{timestamp}.json")
                    json_map_path = os.path.join(self.output_dir, f"sngnn_input_map_{timestamp}.json")
                    try:
                        cv2.imwrite(out_path, img)
                        with open(json_path, 'w') as f:
                            json.dump(output_frames, f, indent=2)
                        with open(json_map_path, 'w') as f:
                            json.dump(selected_frames, f, indent=2)
                        self.last_save_time = now_sec
                    except Exception as e:
                        self.get_logger().error(f"Failed to save image: {e}")

        except Exception as e:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            filename = exception_traceback.tb_frame.f_code.co_filename
            line_no = exception_traceback.tb_lineno
            self.get_logger().error(f"{filename}の{line_no}行目でエラーが発生しました。詳細：{e}")
            self.get_logger().error(f"Prediction failed: {e}")

def input_id_hash(s):
    # simple hash for string id
    return abs(hash(s))

def main(args=None):
    rclpy.init(args=args)
    node = SNGNNNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
