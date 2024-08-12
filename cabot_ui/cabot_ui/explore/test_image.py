import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String
import argparse
from mf_localization_msgs.msg import MFLocalizeStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import tf_transformations
import sys
import numpy as np
import cv2
from cv2 import aruco
from typing import Tuple, Dict, List, Any, Union, Optional
import message_filters
import base64
import requests
import textwrap
import os
import datetime
import json
import cv_bridge
from cv_bridge import CvBridge
import re
import time
from packaging.version import Version
from . import test_speak
import traceback
import json
from cabot_msgs.msg import Log
import threading
from copy import copy


"""
This script will output the GPT-generated explanation from the images captured by the robot.

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
$ python3 test_image.py
```

This script will output the current local costmap to `local_costmap.npy` file.
"""

class CaBotImageNode(Node):
    def __init__(self, use_left: bool = True, use_right: bool = True):
        print("Initializing CaBotImageNode...")
        super().__init__("cabot_map_node")

        self.mode = self.declare_parameter("mode").value
        self.should_speak = self.declare_parameter("should_speak").value
        self.log_dir = self.declare_parameter("log_dir").value
        self.log_dir = os.path.join(self.log_dir, "exploration")
        self.debug = self.declare_parameter("debug").value
        self.once = self.declare_parameter("once").value
        self.no_explain_mode = self.declare_parameter("no_explain_mode").value
        self.is_sim = self.declare_parameter("is_sim").value
        self.logger = self.get_logger()
        self.apikey = self.declare_parameter("apikey").value
        self.ready = False

        if  self.mode == "semantic_map_mode":
            postfix = "s"
        elif self.mode == "intersection_detection_mode":
            postfix = "i"
        elif self.mode == "surronding_explain_mode":
            postfix = "e"
        else:
            postfix = ""
            raise ValueError("Please set the mode to either semantic_map_mode, intersection_detection_mode, or surronding_explain_mode.")

        self.log_dir += f"_{postfix}_images"
        os.makedirs(self.log_dir, exist_ok=True)
        self.logger.info(f"Saving images to {self.log_dir}")

        if self.mode == "surronding_explain_mode" or  self.mode =="semantic_map_mode":
            valid_state = "running"
        elif self.mode == "intersection_detection_mode":
            valid_state = "paused"
        
        self.valid_state = valid_state

        if self.is_sim:
            self.logger.info("Using simulation environment")
            self.front_camera_topic_name = "/camera/color/image_raw"
            self.left_camera_topic_name = "/camera/color/image_raw"
            self.right_camera_topic_name = "/camera/color/image_raw"
            self.front_depth_topic_name = "/camera/depth/image_raw"
            self.left_depth_topic_name = "/camera/depth/image_raw"
            self.right_depth_topic_name = "/camera/depth/image_raw"
        else:
            self.front_camera_topic_name = "/rs1/color/image_raw"
            self.front_depth_topic_name = "/rs1/aligned_depth_to_color/image_raw"
            if use_right:
                self.right_camera_topic_name = "/rs2/color/image_raw"
                self.right_depth_topic_name = "/rs2/aligned_depth_to_color/image_raw"
            else:
                # if right camera is not used, use the front camera as the right camera
                self.right_camera_topic_name = self.front_camera_topic_name
                self.right_depth_topic_name = self.front_depth_topic_name
            if use_left:
                self.left_camera_topic_name = "/rs1/color/image_raw"
                self.left_depth_topic_name = "/rs1/aligned_depth_to_color/image_raw"
            else:
                # if left camera is not used, use the front camera as the left camera
                self.left_camera_topic_name = self.front_camera_topic_name
                self.left_depth_topic_name = self.front_depth_topic_name
        
        # odom subscriber
        self.odom_topic_name = "/odom"
        
        self.front_image = None
        self.front_depth = None
        self.left_image = None
        self.left_depth = None
        self.right_image = None
        self.right_depth = None

        self.front_marker_detected = False
        self.left_marker_detected = False
        self.right_marker_detected = False

        self.odom = None

        self.cabot_nav_state = "running"
        # cabot nav state subscriber
        self.cabot_nav_state_sub = self.create_subscription(String, "/cabot/nav_state", self.cabot_nav_state_callback, 10)

        # transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # self.localize_status_pub = self.create_publisher(MFLocalizeStatus, "/localize_status", transient_local_qos)
        
        self.image_front_sub = message_filters.Subscriber(self, Image, self.front_camera_topic_name)
        # self.depth_front_sub = message_filters.Subscriber(self, Image, self.front_depth_topic_name)
        self.image_left_sub = message_filters.Subscriber(self, Image, self.left_camera_topic_name)
        # self.depth_left_sub = message_filters.Subscriber(self, Image, self.left_depth_topic_name)
        self.image_right_sub = message_filters.Subscriber(self, Image, self.right_camera_topic_name)
        # self.depth_right_sub = message_filters.Subscriber(self, Image, self.right_depth_topic_name)
        self.odom_sub = message_filters.Subscriber(self, Odometry, self.odom_topic_name)
        
        self.activity_sub = self.create_subscription(Log, "/cabot/activity_log", self.activity_callback, 10)
        # subscribers = [self.odom_sub, self.image_front_sub, self.depth_front_sub, self.image_left_sub, self.depth_left_sub, self.image_right_sub, self.depth_right_sub]
        subscribers = [self.odom_sub, self.image_front_sub, self.image_left_sub, self.image_right_sub]
        
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 10, 0.1)
        self.ts.registerCallback(self.image_callback)

        self.logger.info(f"Subscribed to {self.front_camera_topic_name}, {self.left_camera_topic_name}, {self.right_camera_topic_name}, {self.odom_topic_name}")
        self.logger.info(f"CaBotImageNode initialized. in mode = {self.mode} cabot nav state: {self.cabot_nav_state}")
        
        self.gpt_explainer = GPTExplainer(
                log_dir=self.log_dir,
                mode=self.mode,
                node=self,
                should_speak=self.should_speak,
                api_key=self.apikey,
                # dummy=True if is_sim else False
            )

        self.can_speak_explanation = False
        self.can_speak_timer = None

        self.timer = self.create_timer(5.0, self.loop)
    
    def activity_callback(self, msg: Log):
        self.logger.info(f"activity log: {msg}")
        if msg.category == "cabot/interface" and msg.memo == "ready":
            self.ready = True
        elif msg.category == "ble speech request completed":
            if not self.can_speak_explanation:
                self.can_speak_explanation = True
                self.logger.info("can speak explanation set to True by activity log")
                if self.can_speak_timer is not None:
                    self.can_speak_timer.cancel()

    def reset_can_speak(self):
        self.logger.info("can speak explanation set to True by timer")
        self.can_speak_explanation = True
        if self.can_speak_timer is not None:
            self.can_speak_timer.cancel()

    def cabot_nav_state_callback(self, msg: String):
        self.cabot_nav_state = msg.data
        self.logger.info(f"cabot nav state: {self.cabot_nav_state}")
        # print(f"cabot nav state: {self.cabot_nav_state}")

    def image_callback(self, msg_odom, msg_front, msg_left, msg_right):
        if self.cabot_nav_state != self.valid_state:
            return
        self.logger.info(f"image callback; {self.cabot_nav_state}")
        # convert the images to numpy arrays
        front_image = np.array(msg_front.data).reshape(msg_front.height, msg_front.width, 3)
        left_image = np.array(msg_left.data).reshape(msg_left.height, msg_left.width, -1)
        right_image = np.array(msg_right.data).reshape(msg_right.height, msg_right.width, -1)

        # # convert the depth message to image
        # bridge = CvBridge()
        # front_depth = bridge.imgmsg_to_cv2(msg_front_depth, desired_encoding="passthrough")
        # front_depth_array = np.asarray(front_depth)
        # print("front depth", front_depth_array.shape)

        # left_depth = bridge.imgmsg_to_cv2(msg_left_depth, desired_encoding="passthrough")
        # left_depth_array = np.asarray(left_depth)
        # print("left depth", left_depth_array.shape)

        # right_depth = bridge.imgmsg_to_cv2(msg_right_depth, desired_encoding="passthrough")
        # right_depth_array = np.asarray(right_depth)
        # print("right depth", right_depth_array.shape)

        # # save the depth image
        # np.save(f"{self.log_dir}/front_depth.npy", front_depth_array)
        # np.save(f"{self.log_dir}/left_depth.npy", left_depth_array)
        # np.save(f"{self.log_dir}/right_depth.npy", right_depth_array)

        # save the odom
        quaternion = (msg_odom.pose.pose.orientation.x, msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z, msg_odom.pose.pose.orientation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        odom = np.array([msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y, yaw])

        # register as class variables
        self.front_image = front_image
        self.left_image = left_image
        self.right_image = right_image
        # self.front_depth = front_depth_array
        # self.left_depth = left_depth_array
        # self.right_depth = right_depth_array
        self.odom = odom

        # detect the marker in the image
        self.front_marker_detected = self.detect_marker(front_image)
        self.left_marker_detected = self.detect_marker(left_image)
        self.right_marker_detected = self.detect_marker(right_image)

        # save the image
        cv2.imwrite(f"{self.log_dir}/front.jpg", front_image)
        cv2.imwrite(f"{self.log_dir}/left.jpg", left_image)
        cv2.imwrite(f"{self.log_dir}/right.jpg", right_image)

        #save the odom as numpy array
        np.save(f"{self.log_dir}/odom.npy", odom)

        # self.logger.info(f"front, {type(front_image)}, {front_image.shape}, left, {type(left_image)}, {left_image.shape}, right, {type(right_image)}, {right_image.shape}")
        # self.logger.info(f"odom saved: {self.log_dir}/odom.npy")

    
    def detect_marker(self, image: np.ndarray) -> bool:
        # detect the marker in the image
        # change the process depending on the cv2 version >= 4.6
        if Version(cv2.__version__) >= Version("4.6"):
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters()
            cv2_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            detector = aruco.ArucoDetector(dictionary, parameters)
            corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        else:
            dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(image, dictionary, parameters=parameters)
        if ids is not None:
            return True
        return False

    def loop(self):
        self.logger.info(f"going into loop with mode {self.mode}")
        # generate explanation
        if not self.no_explain_mode and self.ready:
            if self.mode == "surrounding_explain_mode":
                if not self.can_speak_explanation:
                    self.logger.info("can't speak explanation yet because can_speak_explanation is False no mode surrounding_explain_mode")
                    return
                
            self.logger.info(f"Generating explanation; mode: {self.mode}, state: {self.cabot_nav_state}")
            # intersection detection mode -> only rcl_publisher.cabot_nav_state is "paused", then explain
            # semantic map mode & surronding explain mode -> only rcl_publisher.cabot_nav_state is "running", then explain
            if self.cabot_nav_state != self.valid_state:
                self.logger.info(f"mode: {self.mode} current state: {self.cabot_nav_state}; not {self.valid_state}. Skip explaining")
                return
            else:
                try:
                    wait_time = self.gpt_explainer.explain(self.front_image, self.left_image, self.right_image)
                    self.can_speak_explanation = False
                    self.can_speak_timer = self.create_timer(wait_time + 10.0, self.reset_can_speak)
                except Exception as e:
                    traceback.print_exc()
                    self.logger.error(f"Error occurred: {e}")
                    return

                if self.mode == "intersection_detection_mode":
                    self.logger.info("Availability of each direction")
                    self.logger.info(f"Front marker : {self.front_marker_detected}")
                    self.logger.info(f"Left marker  : {self.left_marker_detected}")
                    self.logger.info(f"Right marker : {self.right_marker_detected}")
                    if not self.no_explain_mode:
                        self.logger.info(f"Front available (GPT) : {self.gpt_explainer.front_available}")
                        self.logger.info(f"Left available (GPT)  : {self.gpt_explainer.left_available}")
                        self.logger.info(f"Right available (GPT) : {self.gpt_explainer.right_available}")
                
                if self.debug:
                    # randomly decide the values
                    self.logger.info("random mode; setting the values randomly")
                    self.front_marker_detected = np.random.choice([True, False])
                    self.left_marker_detected = np.random.choice([True, False])
                    self.right_marker_detected = np.random.choice([True, False])
                    self.gpt_explainer.front_available = np.random.choice([True, False])
                    self.gpt_explainer.left_available = np.random.choice([True, False])
                    self.gpt_explainer.right_available = np.random.choice([True, False])
                
            if self.mode == "surronding_explain_mode":
                # wait_time = max(wait_time, 7.0)
                self.logger.info(f"surronding_explain_mode next wait time: {wait_time}")
                self.change_timer_interval(interval=wait_time)
        else:
            self.logger.info(f"NOT generating description because: not_explain_mode: {self.no_explain_mode}, ready: {self.ready}, can_speak_explanation: {self.can_speak_explanation}")
            self.logger.info(f"next wait time: 1.0 (constant)")
            self.change_timer_interval(interval=1.0)

    def change_timer_interval(self, interval: float):
        self.timer.cancel()
        self.timer = self.create_timer(interval, self.loop)


class GPTExplainer():
    def __init__(
            self, 
            log_dir: str,
            mode: str,
            node: Node,
            api_key: str,
            should_speak: bool = False,
            dummy: bool = False
        ):
        self.dummy = dummy
        self.api_key = api_key
        self.mode = mode
        self.node = node
        self.logger = self.node.get_logger()

        self.logger.info(f"Initializing GPTExplainer with api_key: {self.api_key}")

        self.headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        if self.mode == "semantic_map_mode":
            self.prompt = """
            ### 指示
            画像を説明してください。
            画像を説明するには、以下のルールに必ず従ってください。

            ### 指示に従うために必ず守るべきルール
            1. 画像の左/前/右にある物体を特定し、そのシーンを知るのに必要な大枠の情報および詳細な情報、両方を説明すること。ただ、説明する必要がない方向があれば（例：右側には何もない）その方向に関しては説明する必要はないです。
            2. 床や天井に関する説明はしなくて大丈夫です。左/前/右の説明に注力してください。
            3. 口調として、「右手には~~があります」等、視覚障害者ガイドのように話してください。
            4. 特徴的なオプジェクトや展示がある場合説明に含めること。
            5. 「何もない」という情報を含める必要ないです。
            6. 「画像は、、、」「視点」等説明を聞くユーザにとって不自然な言葉は排除してください
            7. 物体を説明する際ははっきり見える物のみ説明してください。
            8. 遠くにあって不明瞭な物の説明は不要です。また、照明の明るさや暗さに関する説明も不要です。
            9. あなたの返答は直接システムのTTSに入力されるため、ひらがなで説明してください。

            上記のルールを守り、良い説明文を生成した場合、あなたに50ドルのチップを与えます。
            JSON形式で返答してください。
            キーは、"description"キーの中に画像の説明をString型で文章で入れてください。

            最初にJSONの始まりである```json\n{から返答を開始してください。
            返答例は以下です。
            ```json
            {
            "description": "String型で周囲説明",
            }
            ```
            """
        elif self.mode == "intersection_detection_mode":
            self.prompt = """
            ### 指示1
            与えられた画像から左/前/右に歩行可能か判断してください。
            歩行可能かそうでないかの判断基準として以下を使用してください。
            また、いけるか行けないかの判断をする際には一度考えを口に出して言って、整理してください。

            ### 基準
            1. その方向が開けているか。開けていればいけるかもしれません。
            2. その方向に3mほど直進できるか。できないのであればその方向には行けません。
            3. その方向3m以内に壁があって完全に道を塞いでいるか。壁があるならその方向には行けません。
            4. その方向に3m進むとお店など別の区切られたエリアに完全に入ってしまうか。お店に入ってしまうのであれば、行けません。遠くにお店や区切られたエリアがある場合は行けるものとして大丈夫です。
            5. その方向に障害物があるか。障害物があって完全に道を塞いでいる場合は進めませんが、半分程度塞いでいて、その他は空いているスペースがある場合は行けるものとします。
            6. エリアを分けるポールによって囲まれたエリアに入ってしまうか。入ってしまうのであれば行けません。
            7. 全ての方向に行けないことはないです。全ての方向に行くことが難しいとしても最低ひとつの一番進めそうな方向を選択してください。

            上記のルールを守り、良い説明文を生成した場合、あなたに50ドルのチップを与えます。
            JSON形式で返答してください。
            キー"front_think","left_think","right_think"の中にその方向に歩行可能かの考察を書いてください。
            キー"front","left","right"の中にその方向に歩行可能かを示すbool値true,falseを入れてください。

            最初にJSONの始まりである```json\n{から返答を開始してください。
            返答例は以下です。
            ```json
            {
            "front_think": "<前方に行けるか考察>",
            "left_think": "<左方に行けるか考察>",
            "right_think": "<右方に行けるか考察>",
            "front": <true/false>,
            "left": <true/false>,
            "right": <true/false>
            }
            ```
            """
        elif self.mode == "surronding_explain_mode":
            self.prompt = """
            ### 指示1
            画像を説明してください。
            あなたの生成した文章はそのまま視覚障害者の方に読まれます。
            画像を説明するには、以下のルールに必ず従ってください。

            ### 指示1に従うために必ず守るべきルール
            1. 視覚障害者の人が歩きながら聞くので、簡潔に説明すること。すべての見える詳細を説明する必要はないです。
            2. 画像の左/前/右にある物体を特定し、そのシーンを知るのに必要な大枠の情報を説明すること。ただ、説明する必要がない方向があれば（例：右側には何もない）その方向に関しては説明する必要はないです。
            3. 床や天井に関する説明はしなくて大丈夫です。左/前/右の説明に注力してください。
            4. 口調として、「右手には~~があります」等、視覚障害者ガイドのように話してください。
            5. 細かい内容を説明する必要は必ずしもありませんが、視覚障害者の人がその場所を知ることに役に立つ詳細があれば手短に説明すること。
            6. 一まとまりの文章で説明する事。全体で1-3文で説明してください。
            7. 特徴的なオプジェクトや展示がある場合説明に含めること。
            8. 「何もない」という情報を含める必要ないです。
            9. 「画像は、、、」「視点」等説明を聞くユーザにとって不自然な言葉は排除してください
            10. 物体を説明する際ははっきり見える物のみ説明してください。
            11. 遠くにあって不明瞭な物の説明は不要です。また、照明の明るさや暗さに関する説明も不要です。
            12. あなたの返答は直接システムのTTSに入力されるため、ひらがなで説明してください。

            上記のルールを守り、良い説明文を生成した場合、あなたに50ドルのチップを与えます。
            JSON形式で返答してください。
            キーは、"description"キーの中に画像の説明を入れてください。

            最初にJSONの始まりである```json\n{から返答を開始してください。
            返答例は以下です。
            ```json
            {
            "description": "<周囲説明>",
            }
            ```
            """
        else:
            raise ValueError("Please set the mode to either semantic_map_mode, intersection_detection_mode, or surronding_explain_mode.")
        self.prompt = textwrap.dedent(self.prompt).strip()
        self.log_dir = log_dir

        self.front_available = True
        self.left_available = True
        self.right_available = True

        self.should_speak = should_speak
        self.conversation_history = []

    # Function to encode the image
    def encode_image(self, image):
        _, buffer = cv2.imencode('.jpg', image)
        image_bytes = buffer.tobytes()
        return base64.b64encode(image_bytes).decode('utf-8')

    def resize_images(self, image, max_width=None, max_height=None):
        image_width = image.shape[1]
        image_height = image.shape[0]
        if max_width is not None and image_width > max_width:
            scale = max_width / image_width
            image = cv2.resize(image, (max_width, int(image_height * scale)))
        if max_height is not None and image_height > max_height:
            scale = max_height / image_height
            image = cv2.resize(image, (int(image_width * scale), max_height))
        return image

    def explain(self, front_image: np.ndarray, left_image: np.ndarray, right_image: np.ndarray) -> float:
        if self.dummy:
            self.logger.info("This is a dummy explanation.")
            return
        
        if len(self.conversation_history) == 0:
            prompt = self.prompt
        else:
            if self.mode == "scene_description_mode" or self.mode == "intersection_detection_mode":
                prompt = self.prompt
                self.conversation_history = []
            else:
                prompt = "この画像の説明も同様にルールに従って生成してください。ただし、前の画像説明ですでに説明されているものは含めないでください。\
                    重複した説明はせずに、簡潔に説明してください。前のレスポンスと同じようにJSON形式で返してください。"

        try:
            # resize images max width 512
            front_image = self.resize_images(front_image, max_width=768)
            left_image = self.resize_images(left_image, max_width=768)
            right_image = self.resize_images(right_image, max_width=768)

            # generate explanation from the three-view images
            front_image_with_text = self.add_text_to_image(front_image, "Front")
            left_image_with_text = self.add_text_to_image(left_image, "Left")
            right_image_with_text = self.add_text_to_image(right_image, "Right")

            # save the images for debug
            cv2.imwrite(f"{self.log_dir}/front_with_text.jpg", front_image_with_text)
            cv2.imwrite(f"{self.log_dir}/left_with_text.jpg", left_image_with_text)
            cv2.imwrite(f"{self.log_dir}/right_with_text.jpg", right_image_with_text)

            gpt_response = self.query_with_images(prompt, [front_image_with_text, left_image_with_text, right_image_with_text])
            self.logger.info(f"History and response: {self.conversation_history}, {gpt_response}")              # print(f"{self.mode}: {gpt_response}")
        except Exception as e:
            self.logger.info(f"Error in GPTExplainer.explain: {e}")
            gpt_response = self.query_with_images(prompt, [])

        pretty_response = json.dumps(gpt_response, indent=4)
        self.logger.info(f"GPTExplainer.explain gpt_response:\n{pretty_response}")

        # get front/left/right availability
        extracted_json = self.extract_json_part(gpt_response["choices"][0]["message"]["content"])
        if extracted_json is None:
            self.logger.info("Could not extract JSON part from the response.")
            gpt_response["description"] = "不具合が発生しました。"
        else:
            if self.mode == "intersection_detection_mode":
                self.front_available = extracted_json.get("front", True)
                self.left_available = extracted_json.get("left", True)
                self.right_available = extracted_json.get("right", True)
                try:
                    gpt_response["description"] = extracted_json["front_think"] + extracted_json["left_think"] + extracted_json["right_think"]
                except KeyError:
                    self.logger.info("Could not extract the explanation from the response.")
                    gpt_response["description"] = "不具合が発生しました。"
            else:
                gpt_response["description"] = extracted_json["description"]
        gpt_response["log_dir"] = self.log_dir

        wait_time = 0.0
        if self.should_speak:
            wait_time = self.calculate_speak_time(gpt_response["description"])
            test_speak.speak_text(gpt_response["description"])
            self.logger.info(f"Speaking the {self.mode} explanation")

        # add the explanation to an existing file
        # create a directory with timestamp
        os.makedirs(self.log_dir, exist_ok=True)
        filename = f"{self.log_dir}/explanation.jsonl"
        if not os.path.exists(filename):
            with open(filename, "w") as f:
                f.write(json.dumps(extracted_json) + "\n")
        else:
            with open(filename, "a") as f:
                f.write(json.dumps(extracted_json) + "\n")
        self.logger.info(f"Explanation is saved to {filename}")

        filename = f"{self.log_dir}/log.jsonl"
        if not os.path.exists(filename):
            with open(filename, "w") as f:
                f.write(json.dumps(gpt_response) + "\n")
        else:
            with open(filename, "a") as f:
                f.write(json.dumps(gpt_response) + "\n")
        self.logger.info(f"Log is saved to {filename}")

        self.logger.info(f">>>>>>>\n{self.mode}: {gpt_response}\n<<<<<<<")

        return wait_time
    
    def calculate_speak_time(self, text: str) -> float:
        # calculate the time to speak the text
        # 1 character takes 0.1 seconds
        return len(text) * 0.1

    def extract_json_part(self, json_like_string: str) -> Optional[Dict[str, Any]]:
        # Regular expression to match JSON part
        json_pattern = re.compile(r'\{.*?\}', re.DOTALL)

        # Find the JSON part
        match = json_pattern.search(json_like_string)

        if match:
            json_part = match.group(0)
            try:
                # Parse the JSON part
                json_data = json.loads(json_part)
                return json_data
            except json.JSONDecodeError:
                return None
        else:
            return None

    def query_with_images(self, prompt, images, max_tokens=300) -> Dict[str, Any]:
        # Preparing the content with the prompt and images
        new_content = [{"type": "text", "text": prompt}]
        self.conversation_history.append({"role": "user", "content": copy(new_content)})
        
        for image in images:
            base64_image = self.encode_image(image)
            img_info = {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
            }
            new_content.append(img_info)
        
        if len(self.conversation_history) > 1:  
            new_input = self.conversation_history[:-1] + [{"role": "user", "content": new_content}]
        else:
            new_input = [{"role": "user", "content": new_content}]

        payload = {
            "model": "gpt-4o",
            "messages": new_input,
            "max_tokens": max_tokens
        }

        try:
            self.logger.info("Sending the request to OpenAI API...")
            request_start = time.time()
            response = requests.post("https://api.openai.com/v1/chat/completions", headers=self.headers, json=payload)
            res_json = response.json()
            request_elapsed = time.time() - request_start
            self.logger.info(f"Mode: {self.mode} Received response ({request_elapsed:.3f}s)")
        except Exception as e:
            self.logger.info(f"Error message: {e}")
            res_json = {"choices": [{"message": {"content": "Something is wrong with OpenAI API.", "role": "assistant"}}]}
        
        self.conversation_history.append({"role": "system", "content": res_json["choices"][0]["message"]["content"]})
        return res_json

    def add_text_to_image(self, image: np.ndarray, text: str):
        # add text to the image
        cv2_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # putText params: image, text, position, font, fontScale, color, thickness, lineType
        cv2.putText(cv2_image, text, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return cv2_image

def main(args=None):
    rclpy.init(args=args)
    node = CaBotImageNode()
    try:
        rclpy.spin(node)  # Keep the node spinning to process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()