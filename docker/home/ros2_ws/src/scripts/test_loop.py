from test_map import main as get_next_point
from test_explore import main as explore
from test_image import main as generate_from_images
from test_speak import speak_text
import psutil
import argparse
import datetime
import os, sys
import random
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String


class CabotQueryNode(Node):
    """
    subscribe /cabot/user_query topic
    """
    def __init__(self):
        super().__init__("cabot_query_node")
        print("CabotQueryNode initialized")
        self.query_sub = self.create_subscription(String, "/cabot/user_query", self.query_callback, 10)
        self.query_type = None
        self.query_string = None

    def query_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        self.query_type, self.query_string = msg.data.split(";")
        print(f"Query received: {self.query_type}, {self.query_string}")
        # finish the node
        sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dist_filter", "-d", action="store_true", help="Apply distance filter")
    parser.add_argument("--forbidden_area_filter", "-f", action="store_true", help="Apply forbidden area filter")
    parser.add_argument("--trajectory_filter", "-t", action="store_true", help="Apply trajectory filter")
    parser.add_argument("--auto", "-a", action="store_true", help="Automatically select the next point")
    parser.add_argument("--image", "-i", action="store_true", help="Generate explanation from images")
    parser.add_argument("--log_dir", "-l", type=str, help="Log directory")
    parser.add_argument("--sim", "-s", action="store_true", help="Simulator mode")
    parser.add_argument("--keyboard", "-k", action="store_true", help="Keyboard mode")
    args = parser.parse_args()

    maps = []

    iter = 0
    timestamp = datetime.datetime.now().strftime("%m%d-%H%M%S")

    # if test_trajectory.py is not running, run it
    is_trajectory_running = False
    is_already_checked = False
    forbidden_centers = []

    if not args.trajectory_filter:
        is_trajectory_running = True
    
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

    if args.log_dir:
        log_dir = f"logs/{args.log_dir}"
    else:
        log_dir = f"logs/logs_{timestamp}"
        print(f"log dir is not specified; using {log_dir}")
    print(f"Logs will be saved in {log_dir}")
    while True:
        print(f"\nIteration: {iter}")

        if args.image:
            print("Generating GPT-4o explanation from images...\n")
            availability_from_image = generate_from_images(
                log_dir=log_dir, once=True, is_sim=args.sim,
                intersection_detection_mode=True,
                surronding_explain_mode=False,
                semantic_map_mode=False
            )
            # availability_from_image: {
            #     "front_marker": rcl_publisher.front_marker_detected,
            #     "left_marker": rcl_publisher.left_marker_detected,
            #     "right_marker": rcl_publisher.right_marker_detected,
            #     "front_available": gpt_explainer.front_available,
            #     "left_available": gpt_explainer.left_available,
            #     "right_available": gpt_explainer.right_available
            # }
            print(f"Availability from image: {availability_from_image}\n")
        else:
            # availability_from_image = {
            #     "front_marker": False,
            #     "left_marker": False,
            #     "right_marker": False,
            #     "front_available": False,
            #     "left_available": False,
            #     "right_available": True
            # }
            availability_from_image = None

        print(f"Getting next point...\n")
        sampled_points, forbidden_centers, current_coords, costmap = get_next_point(
            do_dist_filter=args.dist_filter, 
            do_forbidden_area_filter=args.forbidden_area_filter, 
            do_trajectory_filter=args.trajectory_filter, 
            auto_mode=args.auto, 
            log_dir=log_dir, 
            availability_from_image=availability_from_image,
            forbidden_centers=forbidden_centers
        )
        maps.append(costmap)

        # calculate map's highlihgted area's diff 
        if len(maps) > 1:
            current_map_area = np.sum(maps[-1] > 0)
            prev_map_area = np.sum(maps[-2] > 0)
            diff = current_map_area - prev_map_area
            print("=====================")
            print(f"Highlighted area diff: {diff} ({diff / prev_map_area * 100:.2f}%)")
            speak_text(f"前回から、新たに探索されたエリアが{diff / prev_map_area * 100:.2f}%増加しました。")
            print("=====================")


        # pick up one direction
        # first, ask user to select the direction
        print(f"Direction candidates: {sorted(list(set([x[1] for x in sampled_points])))}")
        if not args.auto:
            if args.keyboard:
                abbrv_dirs = ["".join([y[0] for y in x[1].split("_")]) for x in sampled_points]
                print(f"Select the direction of the next point from the following directions: {abbrv_dirs}, or 'none'")
                selected_dir = input("Enter the direction: ")
                while selected_dir not in abbrv_dirs and selected_dir != "none":
                    print(f"Invalid direction; try again (directions: {abbrv_dirs}, or 'none')")
                    selected_dir = input("Enter the direction: ")
            else:
                # accept input from ros topic (user_query_node)
                rclpy.init()
                query_node = CabotQueryNode()
                try:
                    rclpy.spin(query_node)
                except SystemExit as e:
                    print(e)
                query_node.destroy_node()
                rclpy.try_shutdown()
                print(f"Query received: {query_node.query_type}, {query_node.query_string}")
                if query_node.query_type == "direction":
                    selected_dir = query_node.query_string
                    selected_dir = "".join([x[0] for x in selected_dir.split("_")])
                else:
                    selected_dir = "coordinates"
                    y, x = query_node.query_string.split(",")
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
                    print("no candidate directions")
                else:
                    output_direction = random.choice(cand_directions)
            output_point_in_direction = [cand[0] for cand in sampled_points if cand[1] == output_direction]
            if len(output_point_in_direction) == 0:
                print(f"no candidate points in the selected direction ({output_direction})")
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

        print(f"Next point: {output_point} ({output_direction})")
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
        speak_text(f"次は、{japanese_directions[output_direction]}方向、およそ{dist:.2f}メートル先に進みます。")
        # return output_point, cand_filter.forbidden_area_centers
        
        print(f"Exploring next point: {output_point}\n")
        x, y = output_point
        explore(x, y)
        print(f"Exploration done\n")