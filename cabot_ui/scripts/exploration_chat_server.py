#!/usr/bin/env python3

from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
import threading
import cv2
import os
from flask import Flask, request, jsonify
from cabot_ui.explore.test_chat_server import *
import std_msgs.msg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
import time
import base64
from copy import copy
from cv_bridge import CvBridge

class ExplorationChatServer(Node):
    def __init__(self):
        super().__init__('exploration_chat_server')

        self.log_dir = self.declare_parameter('log_dir').value
        self.log_dir_search = os.path.join(self.log_dir, "exploration", "gpt")

        self.use_openai = self.declare_parameter('use_openai').value
        self.apikey = self.declare_parameter("apikey").value

        self.logger = self.get_logger()

        self._eventPub = self.create_publisher(std_msgs.msg.String, "/cabot/event", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.query_pub = self.create_publisher(String, "/cabot/user_query", 10)


        self.latest_explained_info_sub = self.create_subscription(String, "/cabot/latest_explained_info", self.latest_explained_info_callback, 10)
        self.latest_explained_info = None

        self.latest_explained_front_image_sub = self.create_subscription(Image, "/cabot/latest_explained_front_image", self.latest_explained_front_image_callback, 10)
        self.latest_explained_front_image = None

        self.latest_explained_back_image_sub = self.create_subscription(Image, "/cabot/latest_explained_right_image", self.latest_explained_right_image_callback, 10)
        self.latest_explained_back_image = None

        self.latest_explained_left_image_sub = self.create_subscription(Image, "/cabot/latest_explained_left_image", self.latest_explained_left_image_callback, 10)
        self.latest_explained_left_image = None

        self.dir_to_jp = {
            "front": "前",
            "left": "左",
            "right": "右",
            "back": "後ろ",
            "front_left": "左前",
            "front_right": "右前",
            "back_left": "左後ろ",
            "back_right": "右後ろ"
        }

        self.prompt = """
            ### 指示
            まず、ユーザから与えられた文字列を三つに分類してください。三つの分類は、"search"、"direction"、"question"、"failed"です。
            ユーザが特定の場所に行きたい場合、"search"として分類してください。
            ユーザが特定の方向に行きたい場合、"direction"として分類してください。
            どちらにも当てはまらない場合や、どちらかに分類されるけれども行く場所や方向が特定できない場合、"failed"として分類してください。

            次に、"search"として分類された場合、ユーザから与えられた文字列からユーザがいきたい場所("target_location")を抽出してください。

            また、"direction"として分類された場合、ユーザから与えられた文字列からユーザがいきたい方向("target_direction")を抽出してください。
            いきたい方向は"front", "left", "right", "back", "front_left", "front_right", "back_left", "back_right"の中から選んでください。

            以下のルールを守り、良い返答を生成した場合、あなたに50ドルのチップを与えます。

            ### 指示に従うために必ず守るべきルール
            1. JSON形式で返答してください。
            2. まず一番最初に"thought"キーの中に入力に対しての考察を入れてください。
            3. その後"classification"キーに"search"、"direction"、"question"、"failed"のいずれかを入れてください。
            4. "search"の場合、"target_location"キーにユーザがいきたい場所を入れてください。
            5. "direction"の場合、"target_direction"キーにユーザがいきたい方向を入れてください。
            6. "question"の場合、"target_location"と"target_direction"のキーは入れないでください。
            7. "failed"の場合、"target_location"と"target_direction"のキーは入れないでください。
            8. "search"の場合、ユーザが行きたい場所は１箇所です。"target_location"キーには１箇所の情報だけ入れてください。
            9. "direction"の場合、ユーザが行きたい方向は１つです。"target_direction"キーには１つの方向だけ入れてください。

            例：
            入力：木製の展示に行きたいわ
            出力：
            {
                "thought": "<あなたが考察したこと>",
                "classification": "search",
                "target_location": "木製の展示",
            }

            入力：さっきの、えっと、なんだっけ黒いパネル？みたいなところに行きたい
            出力：
            {
                "thought": "<あなたが考察したこと>",
                "classification": "search",
                "target_location": "黒いパネル",
            }

            入力：右に行きたい
            出力：
            {
                "thought": "<あなたが考察したこと>",
                "classification": "direction",
                "target_direction": "right",
            }

            入力：うーん、じゃあ、あの、左斜め前に行きたい
            出力：
            {
                "thought": "<あなたが考察したこと>",
                "classification": "direction",
                "target_direction": "front_left",
            }

            入力：うーん、どうしようかなぁ。木製の展示はさっき行ったしなぁ。
            出力：
            {
                "thought": "<あなたが考察したこと>",
                "classification": "failed",
            }

            入力：右と左どっちに行けばいいかなぁ
            出力：
            {
                "thought": "<あなたが考察したこと>",
                "classification": "failed",
            }

            入力：その展示は何色？
            出力：
            {
                "thought": "<あなたが考察したこと>",
                "classification": "question",
            }

            <あなたが考察したこと>の中には、入力に対しての考察を入れてください。例えば、入力が"木製の展示に行きたいわ"の場合、"木製の展示"が目的地であると考察してください。
            JSONの始まりである{から返答を開始してください。

            入力：
            """
        
        self.prompt_question = """
        視覚障害者の方からの質問に対して、適切な返答を生成してください。
        これまであなたが伝えたことは次のようになってます。%s
        JSON形式で返答してください。
        "answer"キーに返答を入れてください。
        """

        # Ensure Flask app runs in a separate thread to avoid blocking ROS 2
        flask_thread = threading.Thread(target=self.run_flask_app)
        flask_thread.start()

    def latest_explained_info_callback(self, msg):
        self.latest_explained_info = msg.data
        self.logger.info(f"latest_explained_info: {self.latest_explained_info}")

    def latest_explained_front_image_callback(self, msg):
        # save under log_dir
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        self.latest_explained_front_image = cv_image
        self.logger.info("latest_explained_front_image is received")

    def latest_explained_right_image_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        self.latest_explained_right_image = cv_image
        self.logger.info("latest_explained_right_image is received")

    def latest_explained_left_image_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        self.latest_explained_left_image = cv_image
        self.logger.info("latest_explained_left_image is received")

    def run_flask_app(self):

        app = self.create_app()
        app.run(host='0.0.0.0', port=5050)

    def specify_destination(self, query_type, query_string, user_query_message):
        self.logger.info("User query is called")
        
        query_msg = String()
        query_msg.data = f"{query_type};{query_string}"
        self.query_message = query_msg.data

        self.query_pub.publish(query_msg)
        self.get_logger().info(f"Query published: {query_msg.data}")

        if query_type == "search":
            query_message = f"ご要望いただいた{user_query_message}の場所を検索いたしましたので、ご案内します。"
        elif query_type == "direction":
            dir_jp = self.dir_to_jp.get(query_string, query_string)
            query_message = f"ご要望いただいた{dir_jp}の方向に進みます。"
        else:
            query_message = f"入力に何か問題がありそうです。もう一度入力してください。"

        return query_message

    def create_app(self):
        app = Flask(__name__)
        @app.route('/service', methods=['POST'])
        def handle_post():
            data = request.json

            logging.info(data)
            self.logger.info(f"Received data: {data}")

            gpt_input = ""
            if "input" in data:
                if "text" in data["input"]:
                    if data["input"]["text"]:
                        gpt_input = data["input"]["text"]
            
            if self.use_openai and gpt_input != "":
                # Preparing the content with the prompt and images
                prompt = textwrap.dedent(self.prompt).strip()
                content = [{"type": "text", "text": prompt + gpt_input}]

                payload = {
                    "model": "gpt-4o",
                    "messages": [{"role": "user", "content": content}],
                    "max_tokens": 300
                }

                headers = {
                    "Content-Type": "application/json",
                    "Authorization": f"Bearer {self.apikey}"
                }

                try:
                    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
                    res_json = response.json()
                except Exception as e:
                    print(f"Error: {e}")
                    res_json = {"choices": [{"message": {"content": "Something is wrong with OpenAI API.", "role": "assistant"}}]}

                # $ curl -X POST http://127.0.0.1:5000/service -H "Content-Type: application/json" -d '{"input":{"text": "木製の展示"}}'
                # >>> {"choices":[{"finish_reason":"stop","index":0,"logprobs":null,"message":{"content":"Hello! How can I assist you today?","role":"assistant"}}],"created":1721701880,"id":"chatcmpl-9nzb64fOmcBAO3c963JGK55WuB0yx","model":"gpt-4o-2024-05-13","object":"chat.completion","system_fingerprint":"fp_400f27fa1f","usage":{"completion_tokens":9,"prompt_tokens":10,"total_tokens":19}}
                query_string = res_json["choices"][0]["message"]["content"]
                print(f"openai response: {query_string}")
                query_json = extract_json_part(query_string)
                if query_json is not None:
                    query_type = query_json.get("classification", "failed")
                    if query_type == "search":
                        query_target = query_json.get("target_location", "unknown")
                        query_string = query_target + "があります"
                        print(f"query type: {query_type}, query target: {query_target}, query string: {query_string}")
                    elif query_type == "direction":
                        query_target = query_json("target_direction", "unknown")
                        query_string = query_target
                        print(f"query type: {query_type}, query target: {query_target}, query string: {query_string}")
                    elif query_type == "question":
                        # make user input quert string
                        query_type = "question"
                        query_string = gpt_input
                    else:
                        query_type = "failed"
                        query_target = "unknown"
                        query_string = "failed"
                    print(f"query type: {query_type}, query target: {query_target}, query string: {query_string}")
                else:
                    query_type = "failed"
                    query_target = "unknown"
                    query_string = "failed"
            else:
                query_string = gpt_input
                query_type = "direction"
                query_target = gpt_input  # same as gpt_input; e.g., right, left, front, back
            
            if query_target == "unknown":
                query_type = "failed"
                query_string = "failed"

            navi = False
            dest_info = {}

            if query_string == "":
                print("skip chat")
                res_text = ["はい。行きたい場所や方向を指定してください。"]
            elif query_type == "failed":
                print("failed to extract JSON")
                res_text = ["入力を理解できませんでした。もう一度入力してください。"]
            elif query_type == "question":
                prompt_question = self.prompt_question % self.latest_explained_info
                prompt_question = prompt_question + query_string
                gpt_explainer = GPTExplainer("question", self, self.apikey)
                res_json = gpt_explainer.query_with_images(prompt_question, [self.latest_explained_front_image, self.latest_explained_back_image, self.latest_explained_left_image])
                answer = res_json["choices"][0]["message"]["content"]["answer"]
                res_text = [answer]
            else:
                if query_type == "search":
                    odom = search(query_string, self.log_dir_search)
                    query_string = f"{odom[0]},{odom[1]}"
                    self.logger.info(f"searched odom: {query_string}")
                    draw_destination_on_rviz([odom], [[0.0, 1.0, 0.0]])

                self.logger.info(f"specify_destination; Query type: {query_type}, Query string: {query_string}")
                query_message = self.specify_destination(query_type, query_string, query_target)

                navi = True
                dest_info = {"nodes": ""}
                self.publish_finish_chat()

                res_text = [query_message]

            self.logger.info(f"response: {res_text}")

            response = {
                "output": {
                    "log_messages": [],
                    "text": res_text
                },
                "intents": [],
                "entities": [],
                "context": {
                    "navi": navi,
                    "dest_info": dest_info,
                    "system": {
                        "dialog_request_counter": 0
                    }
                }
            }
            self.logger.info(f"response: {response}")
            return jsonify(response)
        return app
    
    def publish_finish_chat(self):
        msg = std_msgs.msg.String()
        msg.data = "navigation_finishchat"
        self._eventPub.publish(msg)

class GPTExplainer():
    def __init__(
            self, 
            node: Node,
            api_key: str,
            dummy: bool = False
        ):
        self.dummy = dummy
        self.api_key = api_key
        self.node = node
        self.logger = self.node.get_logger()
        self.conversation_history = []

        self.logger.info(f"Initializing GPTExplainer with api_key: {self.api_key}")

        self.headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }

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

        self.logger.info("Sending the request to OpenAI API...")
        request_start = time.time()
        response = requests.post("https://api.openai.com/v1/chat/completions", headers=self.headers, json=payload)
        try:
            res_json = response.json()
            extracted_json = extract_json_part(res_json["choices"][0]["message"]["content"])
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

    def add_text_to_image(self, image: np.ndarray, text: str):
        # putText params: image, text, position, font, fontScale, color, thickness, lineType
        cv2.putText(image, text, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationChatServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    parser = ArgumentParser()
    main()
