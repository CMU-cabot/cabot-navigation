from flask import Flask, request, jsonify
import logging
import requests
import os
from typing import Dict, Any, Optional, List, Union
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, UInt8MultiArray, Int8, Int16, Float32, String
import sys
import numpy as np
import torch
import json
import pickle
from transformers import AutoTokenizer, AutoModel, AutoImageProcessor
import argparse
import textwrap
import re

from test_semantic import main as build_semantic_map
from test_semantic import extract_text_feature


logging.basicConfig(level=logging.INFO)

app = Flask(__name__)


class RosQueryNode(Node):
    def __init__(self, query_type, query_string, user_query_message: str):
        super().__init__("user_query_node")
        self.query_pub = self.create_publisher(String, "/cabot/user_query", 10)
        self.query_type = query_type  # search or direction
        self.query_string = query_string
        # user_query_message は、ユーザーの入力をGPTでいい感じに整形したもの
        # e.g., "右", "木製の展示", "黒いパネル"
        self.user_query_message = user_query_message

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

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.query_message = ""

    def timer_callback(self):
        self.timer.cancel()
        print("User query is called")
        
        query_msg = String()
        query_msg.data = f"{self.query_type};{self.query_string}"
        self.query_message = query_msg.data

        self.query_pub.publish(query_msg)
        self.get_logger().info(f"Query published: {query_msg.data}")

        if self.query_type == "search":
            self.query_message = f"ご要望いただいた{self.user_query_message}の場所を検索いたしましたので、ご案内します。"
        elif self.query_type == "direction":
            dir_jp = self.dir_to_jp.get(self.query_string, self.query_string)
            self.query_message = f"ご要望いただいた{dir_jp}の方向に進みます。"
        else:
            self.query_message = f"入力に何か問題がありそうです。もう一度入力してください。"

        sys.exit(0)



def search(user_query: str, log_dir: str, use_default_query: bool = False):
    # 1. build semantic map
    image_feature_path, text_feature_path = build_semantic_map(
        images_dir=None, gpt_dir=None, 
        extract_images=True, extract_text=True, log_dir=log_dir
    )
    
    # 2. search the semantic map with the query
    device = "cuda" if torch.cuda.is_available() else "cpu"
    
    # wait for user input
    if use_default_query:
        user_query = "木製の展示"

    # 1. search from CLIP feature
    print("\nSearching from CLIP feature")
    # 1.1 load model
    HF_MODEL_PATH = 'line-corporation/clip-japanese-base'
    tokenizer = AutoTokenizer.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
    model = AutoModel.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
    model.to(device)

    print(f"\nSearching from {log_dir} ...")
    # 1.2 load pre-extracted CLIP image feature
    with open(image_feature_path, "rb") as f:
        image_features: Dict[str, Any] = pickle.load(f)
    image_odoms = image_features.pop("odoms")
    image_files = image_features.pop("image_file_key")

    # similarity_sum = np.zeros(len(image_odoms))
    
    # 1.3 get text features
    text = tokenizer(user_query).to(device)
    sims = {
        "front": {"odom": None, "similarity": 0.0},
        "left": {"odom": None, "similarity": 0.0},
        "right": {"odom": None, "similarity": 0.0},
        "text": {"odom": None, "similarity": 0.0}
    }

    for direction, feature in image_features.items():
        clip_image_features = torch.from_numpy(feature).to(device)
        # torch.Size([76, 1, 512]) -> torch.Size([76, 512])
        clip_image_features = clip_image_features.squeeze(1)
        
        with torch.no_grad():
            text_clip_feature = model.get_text_features(**text)
        # 1.4 get similarity
        similarity = (text_clip_feature @ clip_image_features.T).softmax(dim=-1).cpu().numpy().squeeze()
        max_sim_idx = np.argmax(similarity)
        print(f"similar image for {direction} is: {max_sim_idx} at {image_odoms[max_sim_idx]} with similarity {similarity[max_sim_idx]}")
        sims[direction] = {
            "odom": image_odoms[max_sim_idx],
            "similarity": similarity[max_sim_idx]
        }

    # 2. search from text feature
    # 2.1 load model
    tokenizer = AutoTokenizer.from_pretrained("cl-nagoya/sup-simcse-ja-large")
    model = AutoModel.from_pretrained("cl-nagoya/sup-simcse-ja-large")
    model.to(device)

    # 2.2 load pre-extracted text feature
    print("\nSearching from text feature")
    # text_pre_features = np.load(f"{log_dir}/text_features.npy")
    with open(text_feature_path, "rb") as f:
        text_features_and_texts = pickle.load(f)
    text_features = text_features_and_texts["features"]
    texts = text_features_and_texts["texts"]
    text_odoms = text_features_and_texts["odoms"]
    text_pre_features = torch.from_numpy(text_features).to(device)
    # torch.Size([76, 1, 1024]) -> torch.Size([76, 1024])
    if len(text_pre_features.shape) == 3:
        text_pre_features = text_pre_features.squeeze(1)
        # 2.3 get text features
        text_input_feature = extract_text_feature(user_query, tokenizer, model, device, return_tensors=True)
        # 2.4 get similarity
        similarity = (text_input_feature @ text_pre_features.T).softmax(dim=-1).cpu().numpy().squeeze()
        max_sim_idx = np.argmax(similarity)
        print(f"similar text for {user_query} is: {max_sim_idx} ({texts[max_sim_idx]}) at {text_odoms[max_sim_idx]} with similarity {similarity[max_sim_idx]}")

        sims["text"] = {
            "odom": text_odoms[max_sim_idx],
            "similarity": similarity[max_sim_idx]
        }
    else:
        print(f"text_pre_features.shape: {text_pre_features.shape}")
        sims["text"] = {
            "odom": None,
            "similarity": -10.0
        }

    # get max similarity
    max_sim = 0.0
    max_sim_direction = None
    for direction, sim in sims.items():
        if sim["similarity"] > max_sim:
            max_sim = sim["similarity"]
            max_sim_direction = direction

    print(f"Max similarity is {max_sim} at {max_sim_direction} with odom {sims[max_sim_direction]['odom']}")
    return sims[max_sim_direction]["odom"]
    

def direction_or_search(response: str) -> str:
    # Check if the response is a direction query or a search query
    if response == "":
        return "direction"
    else:
        return "search"


def extract_json_part(json_like_string: str) -> Optional[Dict[str, Any]]:
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


@app.route('/service', methods=['POST'])
def handle_post():
    data = request.json
    use_openai = args.use_openai

    logging.info(data)

    gpt_input = ""
    if "input" in data:
        if "text" in data["input"]:
            if data["input"]["text"]:
                gpt_input = data["input"]["text"]
    
    if use_openai and gpt_input != "":
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

        api_key = os.environ.get('OPENAI_API_KEY')
        if api_key is None:
            raise ValueError("Please set the OPENAI_API_KEY environment variable.")
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}"
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
                query_target = query_json["target_location"]
                query_string = query_target + "があります"
                print(f"query type: {query_type}, query target: {query_target}, query string: {query_string}")
            elif query_type == "direction":
                query_target = query_json["target_direction"]
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

    navi = False
    dest_info = {}
    if query_type != "failed":
        navi = True
        dest_info = {"nodes": []}

    response = {
        "output": {
            "log_messages": [],
            "text": []
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

    if query_string == "":
        print("skip chat")
        response["output"]["text"] = ["はじめ"]
    elif query_type == "failed":
        print("failed to extract JSON")
        response["output"]["text"] = ["入力を理解できませんでした。もう一度入力してください。"]
    else:
        if query_type == "search":
            odom = search(query_string, args.log_dir)
            query_string = f"{odom[0]},{odom[1]}"

        rclpy.init()
        rcl_publisher = RosQueryNode(query_type, query_string, user_query_message=query_target)

        try:
            rclpy.spin(rcl_publisher)
        except SystemExit as e:
            print(e)
        rcl_publisher.destroy_node()
        rclpy.try_shutdown()

        response["output"]["text"] = [rcl_publisher.query_message]
        print(f"response: {response}")
    print(f"response: {response['output']['text']}")
    return jsonify(response)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_openai", action="store_true")
    parser.add_argument("--log_dir", type=str, help="Log directory", required=True)
    args = parser.parse_args()

    app.run(host='0.0.0.0', port=5050)