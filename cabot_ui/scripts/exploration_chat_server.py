#!/usr/bin/env python3

from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
import threading
import sys
import os
from flask import Flask, request, jsonify
from cabot_ui.explore.test_chat_server import *

class ExplorationChatServer(Node):
    def __init__(self):
        super().__init__('exploration_chat_server')

        self.log_dir = self.declare_parameter('log_dir').value
        self.log_dir = os.path.join(self.log_dir, "img_and_odom")

        self.use_openai = self.declare_parameter('use_openai').value
        self.apikey = self.declare_parameter("apikey").value

        self.logger = self.get_logger()

        # Ensure Flask app runs in a separate thread to avoid blocking ROS 2
        flask_thread = threading.Thread(target=self.run_flask_app)
        flask_thread.start()

    def run_flask_app(self):

        app = self.create_app()
        app.run(host='0.0.0.0', port=5050)

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
                prompt = """
                ### 指示
                まず、ユーザから与えられた文字列を三つに分類してください。三つの分類は、"search"、"direction"、"failed"です。
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
                3. その後"classification"キーに"search"、"direction"、"failed"のいずれかを入れてください。
                4. "search"の場合、"target_location"キーにユーザがいきたい場所を入れてください。
                5. "direction"の場合、"target_direction"キーにユーザがいきたい方向を入れてください。
                6. "failed"の場合、"target_location"と"target_direction"のキーは入れないでください。
                7. "search"の場合、ユーザが行きたい場所は１箇所です。"target_location"キーには１箇所の情報だけ入れてください。
                8. "direction"の場合、ユーザが行きたい方向は１つです。"target_direction"キーには１つの方向だけ入れてください。

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

                <あなたが考察したこと>の中には、入力に対しての考察を入れてください。例えば、入力が"木製の展示に行きたいわ"の場合、"木製の展示"が目的地であると考察してください。
                JSONの始まりである{から返答を開始してください。

                入力：
                """
                prompt = textwrap.dedent(prompt).strip()
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
            else:
                if query_type == "search":
                    odom = search(query_string, self.log_dir)
                    query_string = f"{odom[0]},{odom[1]}"
                    draw_destination_on_rviz([odom], [0, 1.0, 0])

                rclpy.init()
                rcl_publisher = RosQueryNode(query_type, query_string, user_query_message=query_target)

                try:
                    rclpy.spin(rcl_publisher)
                except SystemExit as e:
                    print(e)
                rcl_publisher.destroy_node()
                rclpy.try_shutdown()

                navi = True
                dest_info = {"nodes": ""}

                res_text = [rcl_publisher.query_message]

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

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationChatServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    parser = ArgumentParser()
    main()
