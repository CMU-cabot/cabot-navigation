#!/usr/bin/env python3

import datetime
import random
import time
from typing import List, Optional

import numpy as np
import psutil
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import std_msgs.msg
from cabot_ui.explore.test_state_control import StateControl

from cabot_common import vibration
from cabot_ui.explore.test_explore import main as explore
from cabot_ui.explore.test_image import main as generate_from_images
from cabot_ui.explore.test_loop import PersistentCancelClient, CabotQueryNode
from cabot_ui.explore.test_map import main as get_next_point
from cabot_ui.explore.test_speak import speak_text
import threading
import nav_msgs



class ExplorationMainLoop(Node):
    def __init__(self):
        super().__init__('exploration_main_loop')
        self.logger = self.get_logger()
        self.logger.info("ExplorationMainLoop node started")

        self.log_dir = self.declare_parameter('log_dir').value
        self.logger.info(f"log_dir: {self.log_dir}")
        self.note_pub = self.create_publisher(std_msgs.msg.Int8, "/cabot/notification", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.event_pub = self.create_publisher(std_msgs.msg.String, "/cabot/event", 10)
        self.camera_ready_sub = self.create_subscription(std_msgs.msg.Bool, "/cabot/camera_ready", self.camera_ready_callback, 10)
        self.event_sub = self.create_subscription(std_msgs.msg.String, "/cabot/event", self.event_callback, 10)
        self.state_sub = self.create_subscription(std_msgs.msg.String, "/cabot/state", self.state_callback, 10)
        self.plan_sub = self.create_subscription(nav_msgs.msg.Path, "/plan", self.plan_callback, 10)
        self.state_control = StateControl(self)
        self.dist_filter = self.declare_parameter('dist_filter').value
        self.is_sim = self.declare_parameter('is_sim').value
        self.in_button_control = False
        self.in_conversation = False

        self.camera_ready = False
        self.iter = 0

        self.timer = self.create_timer(1.0, self.check_camera_ready_and_start)

        self.marker_a, self.marker_b = None, None
        self.planning_events = []
        self.planning_time_threshold = 10
        self.planning_count_threshold = 10

        self.plan_events = []
        self.ndtws = []
        self.plan_time_threshold = 10
        self.plan_count_threshold = 3
        self.plan_ave_ndtw_threshold = 30

        self.poses = None
        self.nDTW_threshold = 10
        self.nDTW_max_threshold = 150

        self.previous_destination = np.asarray([0, 0])

        self.logger.info("ExplorationMainLoop initialized")

    def event_callback(self, msg):
        self.logger.info(f"[Main Loop] Received event: {msg.data}")

        # TODO: Maybe reset the planning events when the robot is stopped
        if msg.data == "navigation;exploration;compute_path_to_pose":
            current_time = time.time()
            self.planning_events = [t for t in self.planning_events if current_time - t < self.planning_time_threshold]
            self.logger.info(f"[Main Loop] Planning detected at {current_time}")
            if len(self.planning_events) > 0:
                self.logger.info(f"[Main Loop] Time from the last planning event: {current_time - self.planning_events[-1]} seconds")
            self.logger.info(f"[Main Loop] Planning events in last {self.planning_time_threshold} seconds: {len(self.planning_events)}")
            self.planning_events.append(current_time)
            if len(self.planning_events) >= self.planning_count_threshold:
                # Robot may be stuck in planning
                self.logger.warning(f"[Main Loop] WARNING: More than {self.planning_count_threshold} planning events detected within {self.planning_time_threshold} seconds!")
        elif msg.data == "navigation;exploration;compute_path_to_pose_done":
            self.logger.info("[Main Loop] Planning done")
        # if msg.data == "navigation;event;navigation_arrived" or msg.data == "navigation;cancel":
        #     self.logger.info("[Main Loop] Clear planning_events and ndtws")
        #     self.ndtws = []
        #     self.plan_events = []

    def state_callback(self, msg):
        self.logger.info(f"[Main Loop] Received state message: {msg.data}")
        if msg.data == "button_control_start":
            self.in_button_control = True
        elif msg.data == "button_control_end":
            self.in_button_control = False
        elif msg.data == "conversation_start":
            self.in_conversation = True
        elif msg.data == "conversation_end":
            self.in_conversation = False

    def plan_callback(self, msg):
        self.logger.info(f"[Main Loop] Received plan message")
        current_time = time.time()
        poses = msg.poses

        if self.poses is not None:
            #calculate nDTW
            nDTW = self.calculate_nDTW(self.poses, poses)
            self.logger.info(f"[Main Loop] nDTW = {nDTW}")

            if nDTW > self.nDTW_threshold:
                self.logger.info("[Main Loop] Plan has changed! (Replanning)")
                
                intime_event_index = [i for i, t in enumerate(self.plan_events) if current_time - t < self.plan_time_threshold]
                self.plan_events = [self.plan_events[i] for i in intime_event_index]
                self.ndtws = [self.ndtws[i] for i in intime_event_index]

                if len(self.plan_events) > 0:
                    self.logger.info(f"[Main Loop] Time from the last plan event: {current_time - self.plan_events[-1]} seconds")
                self.logger.info(f"[Main Loop] plan events in last {self.plan_time_threshold} seconds: {len(self.plan_events)}, nDTWs: {self.ndtws}")
                self.plan_events.append(current_time)
                self.ndtws.append(nDTW)
                if len(self.plan_events) >= self.plan_count_threshold:
                    # calculate average nDTW, but only lower than the max nDTW threshold
                    average_ndtws = np.mean([ndtw for ndtw in self.ndtws if ndtw < self.nDTW_max_threshold])
                    self.logger.warning(f"[Main Loop] WARNING: {len(self.plan_events)} plan events detected within {self.plan_time_threshold} seconds! (Average nDTW: {average_ndtws})")
                    if np.mean(self.ndtws) > self.plan_ave_ndtw_threshold:
                        self.logger.warning(f"[Main Loop] WARNING: Average nDTW is greater than {self.plan_ave_ndtw_threshold}!")
                        self.logger.info(f"nDTWs: {self.ndtws}, average: {average_ndtws}, size: {len(self.ndtws)}")
                        self.logger.info("[Main Loop] Canceling and Replanning...")
                        self.event_pub.publish(std_msgs.msg.String(data="navigation;cancel"))
                        self.event_pub.publish(std_msgs.msg.String(data="navigation;replan"))

        self.poses = poses

    def calculate_path_length(self, poses):
        path_length = 0
        for i in range(1, len(poses)):
            x1, y1 = poses[i-1].pose.position.x, poses[i-1].pose.position.y
            x2, y2 = poses[i].pose.position.x, poses[i].pose.position.y
            path_length += np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
        return path_length

    def dtw_distance(self, path1, path2):
        n, m = len(path1), len(path2)
        dtw_matrix = np.zeros((n+1, m+1))
        dtw_matrix[1:, 0] = np.inf
        dtw_matrix[0, 1:] = np.inf

        for i in range(1, n+1):
            for j in range(1, m+1):
                x1, y1 = path1[i-1].pose.position.x, path1[i-1].pose.position.y
                x2, y2 = path2[j-1].pose.position.x, path2[j-1].pose.position.y
                cost = np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
                dtw_matrix[i, j] = cost + min(dtw_matrix[i-1, j],    # insertion
                                            dtw_matrix[i, j-1],    # deletion
                                            dtw_matrix[i-1, j-1])  # match

        return dtw_matrix[n, m]

    def calculate_nDTW(self, path1, path2):
        dtw_cost = self.dtw_distance(path1, path2)
        path_length_1 = self.calculate_path_length(path1)
        path_length_2 = self.calculate_path_length(path2)
        total_path_length = path_length_1 + path_length_2
        nDTW = dtw_cost / total_path_length
        return nDTW

    def change_timer_interval(self, interval: float):
        self.timer.cancel()
        self.timer = self.create_timer(interval, self.main_loop)

    def check_camera_ready_and_start(self):
        if self.camera_ready:
            self.timer.cancel()
            threading.Thread(target=self.main_callback).start()
        else:
            self.logger.info("Camera is not ready yet; waiting...")

    def main_callback(self):
        self.timer.cancel()
        self.logger.info("Exploration main loop started")

        dist_filter = self.dist_filter
        forbidden_area_filter = False
        trajectory_filter = False
        auto = True
        use_image = False
        log_dir = self.log_dir
        sim = self.is_sim
        keyboard = False
        debug = False

        self.event_pub.publish(std_msgs.msg.String(data="navigation_main_loop_start")) # TODO: move this inside main_loop
        # print all the parameters
        self.logger.info(f"dist_filter: {dist_filter}")
        self.logger.info(f"forbidden_area_filter: {forbidden_area_filter}")
        self.logger.info(f"trajectory_filter: {trajectory_filter}")
        self.logger.info(f"auto: {auto}")
        self.logger.info(f"use_image: {use_image}")
        self.logger.info(f"log_dir: {log_dir}")
        self.logger.info(f"sim: {sim}")
        self.logger.info(f"keyboard: {keyboard}")
        self.logger.info(f"debug: {debug}")
        maps = []

        if not rclpy.ok():
            rclpy.init()
        state_client = PersistentCancelClient()
        executor = MultiThreadedExecutor()
        executor.add_node(state_client)

        timestamp = datetime.datetime.now().strftime("%m%d-%H%M%S")

        # if test_trajectory.py is not running, run it
        is_trajectory_running = False
        is_already_checked = False
        has_left_initial_area = False
        
        forbidden_centers = []

        initial_coords: Optional[List[float]] = None
        initial_orientation: Optional[float] = None

        if not trajectory_filter:
            is_trajectory_running = True
        
        # check if test_trajectory.py is running in trajectory filter mode
        while not is_trajectory_running:
            for proc in psutil.process_iter():
                try:
                    if proc.name() == "python3" and "test_trajectory.py" in proc.cmdline():
                        is_trajectory_running = True
                        break
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass

            if not is_trajectory_running and not is_already_checked:
                print("test_trajectory.py is not running. Please run it first.")
                is_already_checked = True

        if log_dir:
            log_dir = f"{log_dir}"
        else:
            log_dir = f"logs/logs_{timestamp}"
            print(f"log dir is not specified; using {log_dir}")
        print(f"Logs will be saved in {log_dir}")
        
        # 0. run exploration loop!
        self.logger.info("[ExplorationMainLoop] Waiting for 5 seconds before starting exploration loop...")
        time.sleep(5)
        self.logger.info("[ExplorationMainLoop] Starting exploration loop!")

        while True:
            # check rclpy is ok
            if not rclpy.ok():
                state_client.logger.info("rclpy is not ok; initializing...")
                rclpy.init()
            start_all = time.time()
            state_client.logger.info(f"\nIteration: {self.iter}")
            self.state_control.set_state("paused")
            # 1. generate "intersection" explanation from images to check the availability of each direction
            if use_image:
                state_client.logger.info("Generating GPT-4o explanation from images...\n")
                availability_from_image = generate_from_images(
                    log_dir=log_dir, once=True, is_sim=sim,
                    intersection_detection_mode=True,
                    surronding_explain_mode=False,
                    semantic_map_mode=False,
                    debug=debug
                )
                state_client.logger.info(f"Availability from image: {availability_from_image}\n")

                availability = {"front": True, "right": True, "left": True}
                avail_jp = {"front": "前", "right": "右", "left": "左"}
                availability_speech = ""
                for key, value in availability_from_image.items():
                    # key: "front_marker", "right_marker", "left_marker", "front_available", "right_available", "left_available"
                    direction = key.split("_")[0]
                    key_type = key.split("_")[1]
                    if key_type == "marker":
                        value = not value
                    if not value:
                        availability[direction] = False
                avail_dir = [avail_jp[key] for key, value in availability.items() if value]
                no_avail_dir = [avail_jp[key] for key, value in availability.items() if not value]
                if len(avail_dir) > 0:
                    availability_speech = "、".join(avail_dir) + "方向に進めそうです。"
                if len(no_avail_dir) > 0:
                    availability_speech += "、".join(no_avail_dir) + "方向には進めなさそうです。"
                speak_text(availability_speech)
            else:
                availability_from_image = None

            # 2. get the next point to explore
            self.logger.info(f"availability_from_image: {availability_from_image}")
            state_client.logger.info(f"Getting next point...\n")
            start = time.time()
            sampled_points, forbidden_centers, current_coords, current_orientation, costmap, marker_a, marker_b = get_next_point(
                do_dist_filter=dist_filter, 
                do_forbidden_area_filter=forbidden_area_filter, 
                do_trajectory_filter=trajectory_filter, 
                auto_mode=auto,
                log_dir=log_dir, 
                availability_from_image=availability_from_image,
                forbidden_centers=forbidden_centers,
                initial_coords=initial_coords,
                initial_orientation=initial_orientation,
                marker_a=self.marker_a,
                marker_b=self.marker_b,
                previous_destination=self.previous_destination,
                logger=state_client.logger
            )
            state_client.logger.info(f"Took {time.time() - start:.2f} seconds to get the next point\n")

            self.marker_a = marker_a
            self.marker_b = marker_b

            maps.append(costmap)
            if self.iter == 0:
                initial_coords = current_coords
                initial_orientation = current_orientation
            # if current coords is close to the initial coords, stop the exploration
            dist_from_initial = np.linalg.norm(np.array(current_coords) - np.array(initial_coords))
            state_client.logger.info(f"Distance from the initial coords: {dist_from_initial:.2f}")
            if has_left_initial_area is False and dist_from_initial > 3.0:
                has_left_initial_area = True
                state_client.logger.info("Left the initial area")
            if has_left_initial_area is True and dist_from_initial < 3.0:
                state_client.logger.info("Initial coords reached; stopping the exploration")
                speak_text("一周しました。探索を終了します。")
                # break

            # calculate map's highlihgted area's diff 
            if len(maps) > 1:
                current_map_area = np.sum(maps[-1] > 0)
                prev_map_area = np.sum(maps[-2] > 0)
                diff = current_map_area - prev_map_area
                state_client.logger.info(f"Highlighted area diff: {diff} ({diff / prev_map_area * 100:.2f}%)")

            # pick up one direction
            # first, ask user to select the direction
            direction_candidates: List[str] = sorted(list(set([x[1] for x in sampled_points])))
            state_client.logger.info(f"Direction candidates: {direction_candidates}")

            state_checked_result = state_client.send_request_and_wait()
            state_client.logger.info(f"test_loop; Service call result: {state_checked_result}")
            if state_checked_result.success:
                state_client.logger.info(f"test_loop; Service call succeeded: {state_checked_result.message}")
                current_state = state_checked_result.message  # "running_state" or "cancelled_state"
            else:
                state_client.logger.info(f"test_loop; Service call failed: {state_checked_result.message}")
                current_state = "running_state"
            
            state_client.logger.info(f"test_loop; Current state: {current_state}")
            if current_state == "running_state":
                auto = True
            else:
                auto = False

            query_node = None
            if not auto:
                if keyboard:
                    abbrv_dirs = ["".join([x[0] for x in dir.split("_")]) for dir in direction_candidates]
                    state_client.logger.info(f"Select the direction of the next point from the following directions: {abbrv_dirs}, or 'none'")
                    selected_dir = input("Enter the direction: ")
                    while selected_dir not in abbrv_dirs and selected_dir != "none":
                        state_client.logger.info(f"Invalid direction; try again (directions: {abbrv_dirs}, or 'none')")
                        selected_dir = input("Enter the direction: ")
                else:
                    # accept input from ros topic (user_query_node)
                    # if not rclpy.ok():
                    #     rclpy.init()

                    # here, say something when button control mode is on
                    if self.in_button_control:
                        speak_text("行きたい方向のボタンを押してください。")

                    query_node = CabotQueryNode(candidates=direction_candidates)
                    try:
                        rclpy.spin(query_node)
                    except SystemExit as e:
                        state_client.logger.info(str(e))
                    finally:
                        query_node.destroy_node()
                    # rclpy.try_shutdown()
                    state_client.logger.info(f"Query received: {query_node.query_type}, {query_node.query_string}")
                    if query_node.query_type == "direction":
                        selected_dir = query_node.query_string
                        selected_dir = "".join([x[0] for x in selected_dir.split("_")])
                    elif query_node.query_type == "search":
                        selected_dir = "coordinates"
                        y, x = query_node.query_string.split(",")
                    elif query_node.query_type == "auto":
                        selected_dir = "none"
                    else:
                        # do nothing
                        state_client.logger.info(f"query type '{query_node.query_type}' is not supported!")
                    state_client.logger.info(f"Selected direction: {selected_dir} (received: {query_node.query_type}, {query_node.query_string})")
            else:
                selected_dir = "none"

            if selected_dir == "none":
                # priority: front > front_left = front_right > others
                cand_directions = [cand[1] for cand in sampled_points]
                if "front" in cand_directions:
                    output_direction = "front"
                elif "front_left" in cand_directions and "front_right" in cand_directions:
                    output_direction = random.choice(["front_left", "front_right"])
                elif "front_left" in cand_directions:
                    output_direction = "front_left"
                elif "front_right" in cand_directions:
                    output_direction = "front_right"
                elif "left" in cand_directions and "right" in cand_directions:
                    output_direction = random.choice(["left", "right"])
                elif "left" in cand_directions:
                    output_direction = "left"
                elif "right" in cand_directions:
                    output_direction = "right"
                else:
                    if len(cand_directions) == 0:
                        state_client.logger.info("no candidate directions")
                    else:
                        output_direction = random.choice(cand_directions)
                if len(sampled_points) > 0:
                    output_point_in_direction = [cand[0] for cand in sampled_points if cand[1] == output_direction]
                    if len(output_point_in_direction) == 0:
                        state_client.logger.info(f"no candidate points in the selected direction ({output_direction})")
                    else:
                        output_point = output_point_in_direction[0]
                else:
                    state_client.logger.info("no candidate points is found")
            else:
                if selected_dir == "coordinates":
                    output_point = (float(x), float(y))
                    output_direction = "coordinates"
                else:
                    output_point_and_direction = [cand for cand in sampled_points if "".join([x[0] for x in cand[1].split("_")]) == selected_dir][0]
                    output_point = output_point_and_direction[0]
                    output_direction = output_point_and_direction[1]

            state_client.logger.info(f"Next point: {output_point} ({output_direction})")
            dist = np.linalg.norm(np.array(output_point) - np.array(current_coords))

            japanese_directions = {
                "front": "前",
                "front_left": "左前",
                "front_right": "右前",
                "left": "左",
                "right": "右",
                "back": "後ろ",
                "back_left": "左後ろ",
                "back_right": "右後ろ"
            }
            # if output_direction == "coordinates":
            #     speak_text(f"次は、指定された座標、およそ{dist:.2f}メートル先に進みます。")
            if not output_direction == "coordinates":
                # speak_text(f"次は、{japanese_directions[output_direction]}方向、およそ{dist:.2f}メートル先に進みます。")
                if output_direction.startswith("front"):
                    self.vibrate(vibration.FRONT)
                elif output_direction.endswith("left"):
                    self.vibrate(vibration.LEFT_TURN)
                elif output_direction.endswith("right"):
                    self.vibrate(vibration.RIGHT_TURN)
                elif output_direction == "back":
                    self.vibrate(vibration.UNKNOWN)
                else:
                    self.logger.info(f"Unknown direction: {output_direction}")
                    self.vibrate(vibration.UNKNOWN)

            # return output_point, cand_filter.forbidden_area_centers
            
            state_client.logger.info(f"Exploring next point: {output_point}\n")
            self.previous_destination = output_point
            self.logger.info(f"Set previous destination: {self.previous_destination}")
            x, y = output_point
            if query_node is not None:
                query_type = query_node.query_type
            else:
                query_type = "none"
            self.state_control.set_state("running")
            
            start = time.time()
            explore(x, y, query_type=query_type)
            state_client.logger.info(f"Took {time.time() - start:.2f} seconds to explore the next point\n")
            state_client.logger.info(f"Exploration {self.iter} done\n")
            state_client.logger.info(f"Took {time.time() - start_all:.2f} seconds for all\n")
            self.iter += 1

            # finish check
            should_finish = False
            if should_finish:
                break

    def vibrate(self, pattern=vibration.UNKNOWN):
        """
        if pattern == vibration.FRONT:
        elif pattern == vibration.RIGHT_TURN:
        elif pattern == vibration.LEFT_TURN:
        """
        self.logger.info(f"Vibrating with pattern: {pattern}")
        msg = std_msgs.msg.Int8()
        msg.data = pattern
        self.note_pub.publish(msg)

    def camera_ready_callback(self, msg):
        if not self.camera_ready:
            self.logger.info(f"Camera ready at exploration main loop: {self.camera_ready}")
        self.camera_ready = msg.data

def main(args=None):
    rclpy.init(args=args)

    node = ExplorationMainLoop()

    # use MultiThreadedExecutor to run multiple nodes in the same process
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    print(f"Hello from {__file__}")
    main()