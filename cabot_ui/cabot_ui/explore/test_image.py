import base64
import datetime
import json
import os
import pickle
import re
import threading
import time
import textwrap
from copy import copy
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import requests
import torch
from PIL import Image as PILImage
from packaging.version import Version
from transformers import AutoImageProcessor, AutoModel, AutoTokenizer
from cv2 import aruco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import std_msgs.msg
from tf2_ros import Buffer, TransformListener
import tf_transformations
import message_filters
from cv_bridge import CvBridge

from cabot_msgs.msg import Log
from . import test_speak
from .log_maker import log_image_and_gpt_response
from .test_semantic import concat_features, extract_image_feature, extract_text_feature
from .WebCameraManager import WebCameraManager
from .prompt import PROMPT_EXPLORE, PROMPT_MIDDLE, PROMPT_NAVIGATION


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
        self.persona = self.declare_parameter("persona").value
        self.ready = False
        self.realsense_ready = False
        self.explore_main_loop_ready = False

        self.latest_explained_front_image = None
        self.latest_explained_left_image = None
        self.latest_explained_right_image = None
        self.latest_explain = "None"

        self.latest_explained_info_pub = self.create_publisher(String, "/cabot/latest_explained_info", 10)
        self.latest_explained_front_image_pub = self.create_publisher(Image, "/cabot/latest_explained_front_image", 10)
        self.latest_explained_left_image_pub = self.create_publisher(Image, "/cabot/latest_explained_left_image", 10)
        self.latest_explained_right_image_pub = self.create_publisher(Image, "/cabot/latest_explained_right_image", 10)
        self.latest_web_camera_image_pub = self.create_publisher(Image, "/cabot/latest_web_camera_image", 10)
        self.camera_ready_pub = self.create_publisher(std_msgs.msg.Bool, "/cabot/camera_ready", 10)
        self.prompt_sub = self.create_subscription(std_msgs.msg.String, "/cabot/persona", self.persona_callback, 10)

        self.log_dir = os.path.join(self.log_dir, "gpt")
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
                self.left_camera_topic_name = "/rs3/color/image_raw"
                self.left_depth_topic_name = "/rs3/aligned_depth_to_color/image_raw"
            else:
                # if left camera is not used, use the front camera as the left camera
                self.left_camera_topic_name = self.front_camera_topic_name
                self.left_depth_topic_name = self.front_depth_topic_name
        
        self.front_image = None
        self.front_depth = None
        self.left_image = None
        self.left_depth = None
        self.right_image = None
        self.right_depth = None

        self.webcamera_image = None

        self.front_marker_detected = False
        self.left_marker_detected = False
        self.right_marker_detected = False

        self.odom = None

        self.cabot_nav_state = "running"
        self.touching = False
        if self.is_sim:
            self.touching = True
        self.consequtive_touch_count = 0
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.activity_sub = self.create_subscription(Log, "/cabot/activity_log", self.activity_callback, 10)
        self.event_sub = self.create_subscription(std_msgs.msg.String, "/cabot/event", self.event_callback, 10)
        self.touch_sub = self.create_subscription(std_msgs.msg.Int16, "/cabot/touch", self.touch_callback, 10)
        # subscribers = [self.odom_sub, self.image_front_sub, self.depth_front_sub, self.image_left_sub, self.depth_left_sub, self.image_right_sub, self.depth_right_sub]
        subscribers = [self.image_front_sub, self.image_left_sub, self.image_right_sub]
        
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 10, 0.1)
        self.ts.registerCallback(self.image_callback)

        self.logger.info(f"Subscribed to {self.front_camera_topic_name}, {self.left_camera_topic_name}, {self.right_camera_topic_name}")
        self.logger.info(f"CaBotImageNode initialized. in mode = {self.mode} cabot nav state: {self.cabot_nav_state}")
        
        self.gpt_explainer = GPTExplainer(
                log_dir=self.log_dir,
                mode=self.mode,
                node=self,
                should_speak=self.should_speak,
                api_key=self.apikey,
                persona=self.persona,
                # dummy=True if is_sim else False
            )
        self.web_camera_ready  = False

        self.logger.info("Opening web camera")
        self.web_camera_manager = WebCameraManager(logger=self.logger, log_dir=self.log_dir, resolution="4k")
        if self.web_camera_manager.is_open():
            self.logger.info("Web camera is open")
            test_speak.speak_text("Webカメラが起動しました")
            self.web_camera_ready = True
        else:
            self.logger.info("Web camera is not open in the first attempt. Trying again in a different thread")
            self.open_webcam_loop()

        self.can_speak_explanation = True
        self.can_speak_timer = None
        self.in_conversation = False
        self.last_saved_images_time = time.time()

        if self.is_sim:
            # to save OpenAI API cost, set the max loop to 10
            self.max_loop = 10
        else:
            self.max_loop = -1
        self.loop_count = 0

        #do loop in different threading no timer
        self.timer = threading.Timer(0.1, self.loop).start()

        self.webcam_timer = None

        if self.web_camera_ready:
            self.publish_camera_ready()
            # self.get_web_camera_image() # use this only when after discussing with the team

    def get_web_camera_image(self):
        # Function to capture and schedule the next image capture
        def _capture_and_schedule():
            self.webcamera_image = self.web_camera_manager.get_frame()
            self.logger.info("Captured web camera image")
            # Schedule the next capture 1 second later

        self.webcam_timer = threading.Timer(1.0, _capture_and_schedule).start()

    def persona_callback(self, msg: std_msgs.msg.String):
        self.logger.info(f"[CabotImageNode] Received persona: {msg.data}")
        self.persona = msg.data
        self.gpt_explainer.update_persona(self.persona)

    def open_webcam_loop(self):
        # open the webcam in different thread until it is opened
        def _open_webcam_loop():
            while not self.web_camera_manager.is_open():
                self.web_camera_manager = WebCameraManager(logger=self.logger, log_dir=self.log_dir, resolution="4k")
                time.sleep(1)
            self.logger.info("Web camera is open")
            test_speak.speak_text("Webカメラが起動しました")
            self.web_camera_ready = True
            self.publish_camera_ready()
            self.publish_latest_explained_info()
        thread = threading.Thread(target=_open_webcam_loop)
        thread.start()

    def publish_camera_ready(self):
        def _publish_camera_ready():
            for i in range(10):
                # publish the camera ready status after i seconds
                self.logger.info(f"Publishing camera ready status after {i} seconds")
                self.camera_ready_pub.publish(std_msgs.msg.Bool(data=True))
        thread = threading.Thread(target=_publish_camera_ready)
        thread.start()

    def touch_callback(self, msg: std_msgs.msg.Int16):
        if msg.data == 1:
            self.consequtive_touch_count += 1
        else:
            self.consequtive_touch_count -= 1

        self.consequtive_touch_count = max(0, self.consequtive_touch_count)
        self.consequtive_touch_count = min(10, self.consequtive_touch_count)

        if self.consequtive_touch_count > 5:
            self.touching = True
            if not self.touching:
                self.logger.info("[CHILOG] [TOUCHING]")
        else:
            if self.touching:
                test_speak.speak_text("", force=True)
                self.logger.info("[CHILOG] [NOT_TOUCHING]")
            self.touching = False

    def publish_latest_explained_info(self):
        self.logger.info("Publishing latest explained info")
        msg = String()
        msg.data = self.latest_explain
        self.latest_explained_info_pub.publish(msg)

        # publish the images
        bridge = CvBridge()
        if self.latest_explained_front_image is not None:
            front_image_msg = bridge.cv2_to_imgmsg(self.latest_explained_front_image, encoding="rgb8")
            self.latest_explained_front_image_pub.publish(front_image_msg)
        
        if self.latest_explained_left_image is not None:
            left_image_msg = bridge.cv2_to_imgmsg(self.latest_explained_left_image, encoding="rgb8")
            self.latest_explained_left_image_pub.publish(left_image_msg)

        if self.latest_explained_right_image is not None:
            right_image_msg = bridge.cv2_to_imgmsg(self.latest_explained_right_image, encoding="rgb8")
            self.latest_explained_right_image_pub.publish(right_image_msg)

        if self.webcamera_image is not None:
            # make the resolution smaller
            webcamera_image_msg = bridge.cv2_to_imgmsg(self.webcamera_image, encoding="rgb8")
            self.latest_web_camera_image_pub.publish(webcamera_image_msg)

        self.logger.info("Published latest explained info")
    
    def activity_callback(self, msg: Log):
        if msg.category == "cabot/interface" and msg.memo == "ready":
            self.ready = True
        elif msg.category == "ble speech request completed":
            if not self.can_speak_explanation:
                # set can_speak_explanation to True 3 sec later in different thread
                thread = threading.Timer(3.0, self.reset_can_speak).start()

                if self.can_speak_timer is not None:
                    self.can_speak_timer.cancel()

    def event_callback(self, msg):
        if msg.data == "navigation_startchat" or msg.data == "navigation_tmp_startchat": # if we start conversation on iPhone this does not get called
            self.in_conversation = True
        elif msg.data == "navigation_finishchat" or msg.data == "navigation_tmp_finishchat":
            self.in_conversation = False
        elif msg.data == "navigation_main_loop_start":
            self.explore_main_loop_ready = True
        elif "navigation;destination;goal" in msg.data:
            self.explore_main_loop_ready = True
        elif msg.data == "navigation;cancel":
            # self.explore_main_loop_ready = False
            test_speak.speak_text("", force=True)

    def reset_can_speak(self):
        self.logger.info("can speak explanation set to True by timer")
        self.can_speak_explanation = True
        if self.can_speak_timer is not None:
            self.can_speak_timer.cancel()

    def cabot_nav_state_callback(self, msg: String):
        if self.cabot_nav_state != msg.data:
            self.logger.info(f"cabot nav state: {self.cabot_nav_state}")
        self.cabot_nav_state = msg.data
    
    def image_callback(self, msg_front, msg_left, msg_right):
        # if self.cabot_nav_state != self.valid_state: return
        if time.time() - self.last_saved_images_time < 0.1: return # just not to overload the system
        self.last_saved_images_time = time.time()

        # save the odom
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = transform.transform.translation
            quaternion = transform.transform.rotation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            odom = np.array([position.x, position.y, yaw])
            self.odom = odom        

            front_image = np.array(msg_front.data).reshape(msg_front.height, msg_front.width, 3)
            left_image = np.array(msg_left.data).reshape(msg_left.height, msg_left.width, -1)
            right_image = np.array(msg_right.data).reshape(msg_right.height, msg_right.width, -1)

            # flip left_image vertically and then horizontally
            left_image = cv2.flip(left_image, -1)

            # convert colors from BGR to RGB for all images
            front_image = cv2.cvtColor(front_image, cv2.COLOR_BGR2RGB)
            left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
            right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)
            
            self.front_image = front_image
            self.left_image = left_image
            self.right_image = right_image
            if self.web_camera_manager.is_open():
                self.webcamera_image = self.web_camera_manager.get_frame()    
                self.webcamera_image = self.resize_images(self.webcamera_image, max_width=1920, max_height=1080)   

            self.front_marker_detected = self.detect_marker(front_image)
            self.left_marker_detected = self.detect_marker(left_image)
            self.right_marker_detected = self.detect_marker(right_image)

            if not self.realsense_ready:
                self.realsense_ready = True
                self.logger.info("Realsense ready")
                test_speak.speak_text("カメラが起動しました")
                self.publish_latest_explained_info()

            if not self.web_camera_ready and self.web_camera_manager.is_open():
                self.logger.info("Web camera is open")
                test_speak.speak_text("Webカメラが起動しました")
                self.web_camera_ready = True
                self.publish_latest_explained_info()

            if self.webcam_timer is not None:
                self.webcam_timer.cancel()
            
            self.camera_ready_pub.publish(std_msgs.msg.Bool(data=True))
        except Exception as e:
            self.logger.error(f"Error in image callback: {e}")

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

    def loop(self):
        if self.max_loop > 0 and self.loop_count >= self.max_loop:
            self.logger.info(f"Max loop count reached. Exiting.")
            return
        is_in_valid_state = self.cabot_nav_state == self.valid_state
        self.logger.info(f"[LOOP] State; mode: {self.mode}, state: {self.cabot_nav_state}, valid_state: {self.valid_state}")
        self.loop_count += 1
        camera_ready = self.realsense_ready or self.web_camera_ready
        self.logger.info(f"going into loop with mode {self.mode}, not_explain_mode: {self.no_explain_mode}, ready: {self.ready}, realsense_ready: {self.realsense_ready}, web_camera_ready {self.web_camera_ready}, can_speak_explanation: {self.can_speak_explanation}, in_conversation: {self.in_conversation}, explore_main_loop_ready: {self.explore_main_loop_ready}")
        if not self.no_explain_mode and camera_ready and not self.in_conversation and self.explore_main_loop_ready:

            wait_time, explain = self.gpt_explainer.explain(self.front_image, self.left_image, self.right_image, self.webcamera_image)

            is_in_valid_state = self.cabot_nav_state == self.valid_state
            if self.touching and not self.in_conversation and is_in_valid_state and self.can_speak_explanation and explain != "":
                self.logger.info(f"reading because self.touching {self.touching} and not in conversation {self.in_conversation} and in valid state {is_in_valid_state} and can_speak_explanation {self.can_speak_explanation} and explain is not empty")
                test_speak.speak_text(explain)
                self.logger.info(f"[CHILOG] [EXPLAIN] [{explain}]")
                self.can_speak_explanation = False
                self.logger.info(f"can speak explanation set to False, waiting for {wait_time} sec")
                self.can_speak_timer = self.create_timer(wait_time + 3.0, self.reset_can_speak) # add 3 sec to the wait time to make a certain gap between the explanation
                next_loop_wait_time = 1.0
            else:
                self.logger.info(f"NOT reading because self.touching {self.touching} and not in conversation {self.in_conversation} and in valid state {is_in_valid_state} and can_speak_explanation {self.can_speak_explanation} and explain is {explain}")
                next_loop_wait_time = 0.1

            self.latest_explained_front_image = self.front_image
            self.latest_explained_left_image = self.left_image
            self.latest_explained_right_image = self.right_image
            self.latest_explain = explain

            self.publish_latest_explained_info()

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
                self.logger.info(f"surronding_explain_mode next wait time: {next_loop_wait_time}")
                self.change_timer_interval(interval=next_loop_wait_time)
        else:
            self.logger.info(f"NOT generating description because: not_explain_mode: {self.no_explain_mode}, ready: {self.ready}, realsense_ready: {self.realsense_ready}, web_camera_ready {self.web_camera_ready},c an_speak_explanation: {self.can_speak_explanation}, in_conversation: {self.in_conversation}, is_in_valid_state: {is_in_valid_state}")
            self.logger.info(f"next wait time: 0.5 (constant)")
            self.change_timer_interval(interval=0.5)

    def change_timer_interval(self, interval: float):
        self.timer = threading.Timer(interval, self.loop).start()


class GPTExplainer():
    def __init__(
            self, 
            log_dir: str,
            mode: str,
            node: Node,
            api_key: str,
            should_speak: bool = False,
            persona: str = "explore",
            dummy: bool = False
        ):
        self.dummy = dummy
        self.api_key = api_key
        self.mode = mode
        self.node = node
        self.logger = self.node.get_logger()
        self.persona = persona

        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # load vision model
        HF_MODEL_PATH = 'line-corporation/clip-japanese-base'
        self.tokenizer = AutoTokenizer.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
        self.processor = AutoImageProcessor.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
        self.model = AutoModel.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
        self.model.to(self.device)
        self.model.eval()

        # load text model
        self.text_tokenizer = AutoTokenizer.from_pretrained("cl-nagoya/sup-simcse-ja-large")
        self.text_model = AutoModel.from_pretrained("cl-nagoya/sup-simcse-ja-large")
        self.text_model.to(self.device)
        self.text_model.eval()

        self.logger.info(f"Initializing GPTExplainer with api_key: {self.api_key[:3]}...{self.api_key[-3:]}")

        self.headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        if self.mode == "semantic_map_mode":
            self.prompt = """
            ### 指示
            画像を説明してください。
            %s
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
            %s
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
            if self.persona == "explore":
                self.prompt = PROMPT_EXPLORE
            elif self.persona == "middle":
                self.prompt = PROMPT_MIDDLE
            else:
                self.prompt = PROMPT_NAVIGATION
        else:
            raise ValueError("Please set the mode to either semantic_map_mode, intersection_detection_mode, or surronding_explain_mode.")
        self.prompt = textwrap.dedent(self.prompt).strip()
        self.log_dir = log_dir

        self.front_available = True
        self.left_available = True
        self.right_available = True

        self.should_speak = should_speak
        self.conversation_history = []
        test_inference = self.query_with_images(prompt="test", images=[])
        self.okay_images = False

        if  self.mode == "semantic_map_mode":
            self.postfix = "s"
        elif self.mode == "intersection_detection_mode":
            self.postfix = "i"
        elif self.mode == "surronding_explain_mode":
            self.postfix = "e"
        else:
            self.postfix = ""
            raise ValueError("Please set the mode to either semantic_map_mode, intersection_detection_mode, or surronding_explain_mode.")

    # Function to encode the image
    def encode_image(self, image):
        _, buffer = cv2.imencode('.jpg', image)
        image_bytes = buffer.tobytes()
        return base64.b64encode(image_bytes).decode('utf-8')
    
    def update_persona(self, persona: str):
        self.logger.info(f"Updating persona to {persona}")
        self.persona = persona
        if self.persona == "explore":
            self.prompt = PROMPT_EXPLORE
        elif self.persona == "middle":
            self.prompt = PROMPT_MIDDLE
        elif self.persona == "navigation":
            self.prompt = PROMPT_NAVIGATION
        else:
            self.logger.info(f"Persona {persona} is not recognized.")

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

    def explain(self, front_image: Optional[np.ndarray], left_image: Optional[np.ndarray], right_image: Optional[np.ndarray], webcamera_image: Optional[np.ndarray]) -> float:
        if self.dummy:
            self.logger.info("This is a dummy explanation.")
            return
        use_initial_prompt = False
        if len(self.conversation_history) == 0:
            prompt = copy(self.prompt)
            use_initial_prompt = True
        else:
            prompt = copy(self.prompt)
            use_initial_prompt = True
            self.conversation_history = []

        try:
            images = []

            use_webcamera = False
            use_realsense = False

            if webcamera_image is not None:
                if not self.okay_images:
                    self.logger.info("Okay")
                    self.okay_images = True
                    test_speak.speak_text("実験準備ができました")
                use_webcamera = True
                    
                # resize to 1080p
                webcamera_image_with_text = self.add_text_to_image(webcamera_image, "High View: Left, Right, Front")
                images.append(webcamera_image_with_text)

            if (front_image is not None) and (left_image is not None) and (right_image is not None):
                if not self.okay_images:
                    self.logger.info("Okay")
                    self.okay_images = True
                    test_speak.speak_text("実験準備ができました")
                use_realsense = True

                left_image = self.resize_images(left_image, max_width=768)
                left_image_with_text = self.add_text_to_image(left_image, "Left")
                images.append(left_image_with_text)

                front_image = self.resize_images(front_image, max_width=768)
                front_image_with_text = self.add_text_to_image(front_image, "Front")
                images.append(front_image_with_text)

                right_image = self.resize_images(right_image, max_width=768)
                right_image_with_text = self.add_text_to_image(right_image, "Right")
                images.append(right_image_with_text)

            if use_webcamera and use_realsense:
                prompt = prompt % "画像は4枚あります。順番に全体、左、前、右を撮影した広角の画像です。"
                images_with_text = [left_image_with_text, front_image_with_text, right_image_with_text, webcamera_image_with_text]
            elif use_webcamera:
                prompt = prompt % "画像は1枚あります。周囲を撮影した広角の画像です。"
                images_with_text = [webcamera_image_with_text]
            elif use_realsense:
                prompt = prompt % "画像は3枚あります。順番に左、前、右の画像です。"
                images_with_text = [front_image_with_text, left_image_with_text, right_image_with_text]
            
            if webcamera_image is None and (front_image is None or left_image is None or right_image is None):
                self.logger.info(f"Webcamera image: {webcamera_image}")
                self.logger.info(f"Front image: {front_image}")
                self.logger.info(f"Left image: {left_image}")
                self.logger.info(f"Right image: {right_image}")
                self.logger.info("Error in GPTExplainer.explain: Images are None.")
                return 1.0, "エラー"

            self.logger.info(f"Persona: {self.persona} Prompt: {prompt}")
            gpt_response = self.query_with_images(prompt, images)
            gpt_response["log_dir"] = self.log_dir

            # get front/left/right availability
            self.logger.info(f"Extracting JSON from the response: {gpt_response}")
            extracted_json = gpt_response["choices"][0]["message"]["content"]
            
            if extracted_json is None:
                extracted_json = {"description": ""}
                self.logger.info("Could not extract JSON part from the response.")
                self.logger.info(f"Error in GPTExplainer.explain: extracted_json is None: {gpt_response}")
            else:
                if self.mode == "intersection_detection_mode":
                    self.front_available = extracted_json.get("front", True)
                    self.left_available = extracted_json.get("left", True)
                    self.right_available = extracted_json.get("right", True)
                    try:
                        extracted_json["description"] = extracted_json["front_think"] + extracted_json["left_think"] + extracted_json["right_think"]
                    except KeyError:
                        self.logger.info("Could not extract the explanation from the response.")
                        extracted_json["description"] = "" # error so set the description to empty
                        self.logger.info(f"Error in GPTExplainer.explain: KeyError: {extracted_json}")
                else:
                    self.logger.info(f"{extracted_json} {extracted_json}")
                    if extracted_json == "Error":
                        extracted_json["description"] = ""
                        self.logger.info(f"Error in GPTExplainer.explain: extracted_json is 'Error': {extracted_json}")
                    else:
                        extracted_json["description"] = extracted_json["description"]

            self.logger.info(f"length of gpt_response: {len(extracted_json['description'])} actual response: {extracted_json['description']}")

            ##save odom and img
            current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
            folder_name = os.path.join(self.log_dir,current_time + '_' + self.postfix)
            self.folder_name = folder_name
            os.makedirs(folder_name, exist_ok=True)
            #save the odom as numpy array
            np.save(f"{folder_name}/odom.npy", self.node.odom)

            if not front_image is None:
                cv2.imwrite(os.path.join(folder_name,"front.jpg"), front_image)
            if not left_image is None:
                cv2.imwrite(os.path.join(folder_name,"left.jpg"), left_image)
            if not right_image is None:
                cv2.imwrite(os.path.join(folder_name,"right.jpg"), right_image)
            if not webcamera_image is None:
                cv2.imwrite(os.path.join(folder_name,"webcamera.jpg"), webcamera_image)

            with open(os.path.join(folder_name,"explanation.jsonl"), "w") as f:
                description_json = {"description": extracted_json["description"]}
                json.dump(description_json, f, ensure_ascii=False)

            with open(os.path.join(folder_name,"log.jsonl"), "w") as f:
                json.dump(gpt_response, f, ensure_ascii=False)

            pretty_response = json.dumps(gpt_response, indent=4)

            # if not webcamera_image is None:
            #     images_with_text.append(webcamera_image_with_text)

            log_image_and_gpt_response(images_with_text, str(extracted_json["description"]), self.folder_name)
            self.logger.info(f"History and response: {self.conversation_history}, {gpt_response}")              # print(f"{self.mode}: {gpt_response}")
        except Exception as e:
            self.logger.info(f"Error in GPTExplainer.explain: {e}")
            self.logger.info(traceback.format_exc())
            return  1.0, "エラー"

        self.logger.info(f"GPTExplainer.explain gpt_response:\n{pretty_response}")

        wait_time = 2.0
        if self.should_speak and not extracted_json["description"] == "":
            wait_time = self.calculate_speak_time(extracted_json["description"])
            self.logger.info(f"Speaking the {self.mode} explanation")
        
        # extract features from image & text
        left_feature = extract_image_feature(PILImage.fromarray(left_image), self.processor, self.model, self.device)
        front_feature = extract_image_feature(PILImage.fromarray(front_image), self.processor, self.model, self.device)
        right_feature = extract_image_feature(PILImage.fromarray(right_image), self.processor, self.model, self.device)

        # feature dir is the parent directory of the log_dir
        image_feature_path = os.path.join(self.log_dir, "image_features.pickle")
        # image_file_key is the dir name of self.log_dir
        image_file_key = os.path.basename(self.folder_name)
        if os.path.exists(image_feature_path):
            with open(image_feature_path, "rb") as f:
                image_features = pickle.load(f)
            image_features["front"] = concat_features(image_features["front"], front_feature)
            image_features["left"] = concat_features(image_features["left"], left_feature)
            image_features["right"] = concat_features(image_features["right"], right_feature)
            image_features["odoms"] = image_features.get("odoms", []) + [self.node.odom]
            image_features["image_file_key"] = image_features.get("image_file_key", []) + [image_file_key]
        else:
            image_features = {
                "front": front_feature,
                "left": left_feature,
                "right": right_feature,
                "odoms": [self.node.odom],
                "image_file_key": [image_file_key]
            }
        # save the image features
        with open(image_feature_path, "wb") as f:
            pickle.dump(image_features, f)

        # extract text features
        text_feature = extract_text_feature(extracted_json["description"], self.text_tokenizer, self.text_model, self.device)
        text_feature_path = os.path.join(self.log_dir, "text_features.pickle")
        if os.path.exists(text_feature_path):
            with open(text_feature_path, "rb") as f:
                text_features = pickle.load(f)
            # text_features has four keys: "features", "odoms", "text_file_key", "texts"
            text_features["features"] = concat_features(text_features["features"], text_feature)
            text_features["odoms"] = text_features.get("odoms", []) + [self.node.odom]
            text_features["text_file_key"] = text_features.get("text_file_key", []) + [image_file_key]
            text_features["texts"] = text_features.get("texts", []) + [extracted_json["description"]]
        else:
            text_features = {
                "features": text_feature,
                "odoms": [self.node.odom],
                "text_file_key": [image_file_key],
                "texts": [extracted_json["description"]]
            }
        # save the text features
        with open(text_feature_path, "wb") as f:
            pickle.dump(text_features, f)

        self.logger.info(f">>>>>>>\n{self.mode}: {gpt_response}\n<<<<<<<")

        return wait_time, extracted_json["description"]
    
    def calculate_speak_time(self, text: str) -> float:
        # calculate the time to speak the text
        # assume 1 character takes 0.125 seconds to speak (a bit longer than the average which is 0.1 seconds)
        return len(text) * 0.15

    def extract_json_part(self, json_like_string: str) -> Optional[Dict[str, Any]]:
        # if json is already in the correct format, return it
        if type(json_like_string) == dict:
            return json_like_string
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

    def query_with_images(self, prompt, images, max_tokens=2000) -> Dict[str, Any]:
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

        self.logger.info("Sending the request to OpenAI API...")
        request_start = time.time()
        response = requests.post("https://api.openai.com/v1/chat/completions", headers=self.headers, json=payload)
        try:
            res_json = response.json()
            extracted_json = self.extract_json_part(res_json["choices"][0]["message"]["content"])
            res_json["choices"][0]["message"]["content"] = extracted_json

            request_elapsed = time.time() - request_start
            self.logger.info("OpenAI API Request success.")
            self.logger.info(f"Mode: {self.mode} Received response ({request_elapsed:.3f}s)")
            self.conversation_history.append({"role": "system", "content": str(res_json["choices"][0]["message"]["content"])})
        except Exception as e:
            self.logger.info(f"OpenAI API Request failed. Error message: {e}")
            self.logger.info(f"OpenAI Error Response: {response}")
            res_json = {"choices": [{"message": {"content": "Error", "role": "assistant"}}]}
        
        return res_json

    def add_text_to_image(self, image: np.ndarray, text: str, fontscale=1):
        # putText params: image, text, position, font, fontScale, color, thickness, lineType
        cv2.putText(image, text, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, fontscale, (0, 0, 255), 2, cv2.LINE_AA)
        return image

def main(args=None):
    re_init = False
    if not rclpy.ok():
        rclpy.init(args=args)
        re_init = True
    
    node = CaBotImageNode()
    try:
        node.logger.info("Starting CaBotImageNode...")
        rclpy.spin(node)  # Keep the node spinning to process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.logger.info("Shutting down CaBotImageNode...")
        node.destroy_node()
        # if re_init:
        #     rclpy.shutdown()

if __name__ == "__main__":
    main()