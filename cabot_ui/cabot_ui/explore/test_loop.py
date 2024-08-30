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
                speak_text(f"{self.dir_to_jp[self.query_string]}の方向に進みます。")
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


def main(
    dist_filter: bool = False,
    forbidden_area_filter: bool = False,
    trajectory_filter: bool = False,
    auto: bool = False,
    use_image: bool = False,
    log_dir: Optional[str] = None,
    sim: bool = False,
    keyboard: bool = False,
    debug: bool = False
):
    # print all the parameters
    print(f"dist_filter: {dist_filter}")
    print(f"forbidden_area_filter: {forbidden_area_filter}")
    print(f"trajectory_filter: {trajectory_filter}")
    print(f"auto: {auto}")
    print(f"use_image: {use_image}")
    print(f"log_dir: {log_dir}")
    print(f"sim: {sim}")
    print(f"keyboard: {keyboard}")
    print(f"debug: {debug}")
    maps = []

    if not rclpy.ok():
        rclpy.init()
    state_client = PersistentCancelClient()
    executor = MultiThreadedExecutor()
    executor.add_node(state_client)

    iter = 0
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
    time.sleep(20)
    while True:
        # check rclpy is ok
        if not rclpy.ok():
            state_client.logger.info("rclpy is not ok; initializing...")
            rclpy.init()
        state_client.logger.info(f"\nIteration: {iter}")

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
        state_client.logger.info(f"Getting next point...\n")
        sampled_points, forbidden_centers, current_coords, current_orientation, costmap = get_next_point(
            do_dist_filter=dist_filter, 
            do_forbidden_area_filter=forbidden_area_filter, 
            do_trajectory_filter=trajectory_filter, 
            auto_mode=auto, 
            log_dir=log_dir, 
            availability_from_image=availability_from_image,
            forbidden_centers=forbidden_centers,
            initial_coords=initial_coords,
            initial_orientation=initial_orientation
        )
        maps.append(costmap)
        if iter == 0:
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
            output_point_in_direction = [cand[0] for cand in sampled_points if cand[1] == output_direction]
            if len(output_point_in_direction) == 0:
                state_client.logger.info(f"no candidate points in the selected direction ({output_direction})")
            else:
                output_point = output_point_in_direction[0]
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
        if output_direction == "coordinates":
            speak_text(f"次は、指定された座標、およそ{dist:.2f}メートル先に進みます。")
        else:
            speak_text(f"次は、{japanese_directions[output_direction]}方向、およそ{dist:.2f}メートル先に進みます。")
        # return output_point, cand_filter.forbidden_area_centers
        
        state_client.logger.info(f"Exploring next point: {output_point}\n")
        x, y = output_point
        if query_node is not None:
            query_type = query_node.query_type
        else:
            query_type = "none"
        explore(x, y, query_type=query_type)
        state_client.logger.info(f"Exploration {iter} done\n")
        iter += 1

        # finish check
        should_finish = False
        if should_finish:
            break


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

    main(
        dist_filter=args.dist_filter,
        forbidden_area_filter=args.forbidden_area_filter,
        trajectory_filter=args.trajectory_filter,
        auto=args.auto,
        use_image=args.use_image,
        log_dir=args.log_dir,
        sim=args.sim,
        keyboard=args.keyboard,
        debug=args.debug
    )