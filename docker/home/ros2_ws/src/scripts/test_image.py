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
from test_speak import speak_text


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
    def __init__(self, log_dir: str = ".", is_sim: bool = False, use_left: bool = True, use_right: bool = True, valid_state: str = ""):
        print("Initializing CaBotImageNode...")
        assert valid_state != "", "Please set the valid state."
        super().__init__("cabot_map_node")
        
        self.valid_state = valid_state
        self.log_dir = log_dir

        if is_sim:
            print("Using simulation environment")
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

        self.cabot_nav_state = ""
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
        
        # subscribers = [self.odom_sub, self.image_front_sub, self.depth_front_sub, self.image_left_sub, self.depth_left_sub, self.image_right_sub, self.depth_right_sub]
        subscribers = [self.odom_sub, self.image_front_sub, self.image_left_sub, self.image_right_sub]
        
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 10, 0.1)
        self.ts.registerCallback(self.image_callback)
    
    def cabot_nav_state_callback(self, msg: String):
        self.cabot_nav_state = msg.data
        self.get_logger().info(f"cabot nav state: {self.cabot_nav_state}")
        # print(f"cabot nav state: {self.cabot_nav_state}")

    def image_callback(self, msg_odom, msg_front, msg_left, msg_right):
        if self.cabot_nav_state != self.valid_state:
            return
        print(f"image callback; {self.cabot_nav_state}")
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

        # finish the node
        if self.cabot_nav_state != "":
            print("front", type(front_image), front_image.shape)
            print("left", type(left_image), left_image.shape)
            print("right", type(right_image), right_image.shape)
            # save the image
            cv2.imwrite(f"{self.log_dir}/front.jpg", front_image)
            cv2.imwrite(f"{self.log_dir}/left.jpg", left_image)
            cv2.imwrite(f"{self.log_dir}/right.jpg", right_image)

            np.save(f"{self.log_dir}/odom.npy", odom)
            print(f"odom saved: {self.log_dir}/odom.npy")

            sys.exit(0)
    
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



class GPTExplainer:
    def __init__(
            self, 
            log_dir: str,
            semantic_map_mode: bool,
            intersection_detection_mode: bool,
            surronding_explain_mode: bool,
            should_speak: bool = False,
            dummy: bool = False
        ):
        self.dummy = dummy
        if dummy:
            self.api_key = "dummy"
        else:
            self.api_key = os.environ.get('OPENAI_API_KEY')
            if self.api_key is None:
                raise ValueError("Please set the OPENAI_API_KEY environment variable.")

        self.semantic_map_mode = semantic_map_mode
        self.intersection_detection_mode = intersection_detection_mode
        self.surronding_explain_mode = surronding_explain_mode

        self.headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        if semantic_map_mode:
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
        elif intersection_detection_mode:
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
        elif surronding_explain_mode:
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

        self.wait_time = 0.0
    
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

    def explain(self, front_image: np.ndarray, left_image: np.ndarray, right_image: np.ndarray) -> str:
        if self.dummy:
            print("This is a dummy explanation.")
            return
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

        gpt_response = self.query_with_images([front_image_with_text, left_image_with_text, right_image_with_text])
        print(gpt_response)

        # get front/left/right availability
        extracted_json = self.extract_json_part(gpt_response["choices"][0]["message"]["content"])
        if extracted_json is None:
            print("Could not extract JSON part from the response.")
            gpt_response["description"] = "不具合が発生しました。"
        else:
            if self.intersection_detection_mode:
                self.front_available = extracted_json.get("front", True)
                self.left_available = extracted_json.get("left", True)
                self.right_available = extracted_json.get("right", True)
                try:
                    gpt_response["description"] = extracted_json["front_think"] + extracted_json["left_think"] + extracted_json["right_think"]
                except KeyError:
                    print("Could not extract the explanation from the response.")
                    gpt_response["description"] = "不具合が発生しました。"
            else:
                gpt_response["description"] = extracted_json["description"]
        gpt_response["log_dir"] = self.log_dir

        if self.should_speak:
            self.wait_time = self.calculate_speak_time(gpt_response["description"])
            speak_text(gpt_response["description"])

        # add the explanation to an existing file
        # create a directory with timestamp
        os.makedirs(self.log_dir, exist_ok=True)
        filename = f"{self.log_dir}/explanation.jsonl"
        if not os.path.exists(filename):
            with open(filename, "w") as f:
                f.write(json.dumps(gpt_response))
        else:
            with open(filename, "a") as f:
                f.write(json.dumps(gpt_response))
        print(f"Explanation is saved to {filename}")
        print(">>>>>>>")
        print(gpt_response)
        print("<<<<<<<")
    
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

    def query_with_images(self, images, max_tokens=300) -> Dict[str, Any]:
        # Preparing the content with the prompt and images
        content = [{"type": "text", "text": self.prompt}]
        
        for image in images:
            base64_image = self.encode_image(image)
            img_info = {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
            }
            content.append(img_info)

        payload = {
            "model": "gpt-4o",
            "messages": [{"role": "user", "content": content}],
            "max_tokens": max_tokens
        }

        try:
            print("Sending the request to OpenAI API...")
            request_start = time.time()
            response = requests.post("https://api.openai.com/v1/chat/completions", headers=self.headers, json=payload)
            res_json = response.json()
            request_elapsed = time.time() - request_start
            print(f"Received response ({request_elapsed:.3f}s)")
        except Exception as e:
            print(f"Error message: {e}")
            res_json = {"choices": [{"message": {"content": "Something is wrong with OpenAI API.", "role": "assistant"}}]}
        return res_json

    def add_text_to_image(self, image: np.ndarray, text: str):
        # add text to the image
        cv2_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # putText params: image, text, position, font, fontScale, color, thickness, lineType
        cv2.putText(cv2_image, text, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return cv2_image


def main(
        log_dir: str = ".", 
        no_explain_mode: bool = False, 
        once: bool = False, 
        is_sim: bool = False,
        semantic_map_mode: bool = False,
        intersection_detection_mode: bool = False,
        surronding_explain_mode: bool = False,
        should_speak: bool = False,
        debug: bool = False
    ) -> Dict[str, Any]:
    print("Starting the image node...")
    timestamp = datetime.datetime.now().strftime("%m%d-%H%M%S")

    if semantic_map_mode:
        postfix = "s"
    elif intersection_detection_mode:
        postfix = "i"
    elif surronding_explain_mode:
        postfix = "e"
    else:
        postfix = ""

    log_dir = f"{log_dir}/{timestamp}{postfix}_images"
    os.makedirs(log_dir, exist_ok=True)
    print(f"Saving images to {log_dir}")

    if surronding_explain_mode or semantic_map_mode:
        valid_state = "running"
    elif intersection_detection_mode:
        valid_state = "paused"
    else:
        raise ValueError("Please set the mode to either semantic_map_mode, intersection_detection_mode, or surronding_explain_mode.")
    
    while True:
        if not rclpy.ok():
            rclpy.init()
        rcl_publisher = CaBotImageNode(log_dir=log_dir, is_sim=is_sim, valid_state=valid_state)
        try:
            rclpy.spin(rcl_publisher)
        except SystemExit as e:
            print(f"finished rclpy node")
        
        rcl_publisher.destroy_node()
        rclpy.try_shutdown()

        # generate explanation
        if not no_explain_mode:
            # intersection detection mode -> only rcl_publisher.cabot_nav_state is "paused", then explain
            # semantic map mode & surronding explain mode -> only rcl_publisher.cabot_nav_state is "running", then explain
            if rcl_publisher.cabot_nav_state != valid_state:
                print(f"current state: {rcl_publisher.cabot_nav_state}; not {valid_state}. Skip explaining")
                time.sleep(1.0)
            else:
                gpt_explainer = GPTExplainer(
                    log_dir=log_dir,
                    semantic_map_mode=semantic_map_mode,
                    intersection_detection_mode=intersection_detection_mode,
                    surronding_explain_mode=surronding_explain_mode,
                    should_speak=should_speak,
                    # dummy=True if is_sim else False
                )
                gpt_explainer.explain(rcl_publisher.front_image, rcl_publisher.left_image, rcl_publisher.right_image)

                if intersection_detection_mode:
                    print("Availability of each direction")
                    print(f"Front marker : {rcl_publisher.front_marker_detected}")
                    print(f"Left marker  : {rcl_publisher.left_marker_detected}")
                    print(f"Right marker : {rcl_publisher.right_marker_detected}")
                    if not no_explain_mode:
                        print(f"Front available (GPT) : {gpt_explainer.front_available}")
                        print(f"Left available (GPT)  : {gpt_explainer.left_available}")
                        print(f"Right available (GPT) : {gpt_explainer.right_available}")
                
                if debug:
                    # randomly decide the values
                    print("random mode; setting the values randomly")
                    rcl_publisher.front_marker_detected = np.random.choice([True, False])
                    rcl_publisher.left_marker_detected = np.random.choice([True, False])
                    rcl_publisher.right_marker_detected = np.random.choice([True, False])
                    gpt_explainer.front_available = np.random.choice([True, False])
                    gpt_explainer.left_available = np.random.choice([True, False])
                    gpt_explainer.right_available = np.random.choice([True, False])
                
                if once:
                    return {
                        "front_marker": rcl_publisher.front_marker_detected,
                        "left_marker": rcl_publisher.left_marker_detected,
                        "right_marker": rcl_publisher.right_marker_detected,
                        "front_available": gpt_explainer.front_available,
                        "left_available": gpt_explainer.left_available,
                        "right_available": gpt_explainer.right_available
                    }
                else:
                    print("Waiting for the next image...")
                    if surronding_explain_mode:
                        # we need to avoid overlapping the speaking time
                        time.sleep(max(gpt_explainer.wait_time, 5.0))
                    else:
                        time.sleep(5.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--no_explain", action="store_true", help="Do not explain the images")
    parser.add_argument("--log_dir", type=str, default=".", help="Directory to save the images")
    parser.add_argument("--once", action="store_true", help="Run only once")
    parser.add_argument("--semantic_map", "-s", action="store_true", help="generate descriptions for the semantic map")
    parser.add_argument("--intersection_detection", "-i", action="store_true", help="detect the intersection")
    parser.add_argument("--surronding_explain", "-e", action="store_true", help="generate descriptions for the surrounding")
    parser.add_argument("--sim", action="store_true", help="Use the simulation environment")
    parser.add_argument("--speak", action="store_true", help="Speak the generated text")
    args = parser.parse_args()

    # in semantic map mode, we should not use speak option
    if args.semantic_map and args.speak:
        raise ValueError("In semantic map mode, we should not use the speak option.")

    main(
        no_explain_mode=args.no_explain, 
        log_dir=args.log_dir, 
        once=args.once, 
        is_sim=args.sim,
        semantic_map_mode=args.semantic_map,
        intersection_detection_mode=args.intersection_detection,
        surronding_explain_mode=args.surronding_explain,
        should_speak=args.speak
    )