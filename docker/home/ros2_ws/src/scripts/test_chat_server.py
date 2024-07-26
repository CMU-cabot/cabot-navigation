from flask import Flask, request, jsonify
import logging
import requests
import os
from typing import Dict, Any
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

from test_semantic import main as build_semantic_map
from test_semantic import extract_text_feature


logging.basicConfig(level=logging.INFO)

app = Flask(__name__)


class RosQueryNode(Node):
    def __init__(self, query_type, query_string):
        super().__init__("user_query_node")
        self.query_pub = self.create_publisher(String, "/cabot/user_query", 10)
        self.query_type = query_type  # search or direction
        self.query_string = query_string
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
    # not implemented
    return "direction"


@app.route('/service', methods=['POST'])
def handle_post():
    data = request.json
    use_openai = False

    logging.info(data)

    gpt_input = ""
    if "input" in data:
        if "text" in data["input"]:
            if data["input"]["text"]:
                gpt_input = data["input"]["text"]
    
    if use_openai:
        # Preparing the content with the prompt and images
        prompt = ""  # TODO: Add the prompt here
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
        query_type = direction_or_search(query_string)
    else:
        query_string = gpt_input
        query_type = "search"
    
    if query_type == "search":
        odom = search(query_string, args.log_dir)
        query_string = f"{odom[0]},{odom[1]}"

    rclpy.init()
    rcl_publisher = RosQueryNode(query_type, query_string)

    try:
        rclpy.spin(rcl_publisher)
    except SystemExit as e:
        print(e)
    rcl_publisher.destroy_node()
    rclpy.try_shutdown()

    
    return jsonify(rcl_publisher.query_message)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_openai", action="store_true")
    parser.add_argument("--log_dir", type=str, help="Log directory")
    args = parser.parse_args()

    app.run(host='0.0.0.0', port=5001)