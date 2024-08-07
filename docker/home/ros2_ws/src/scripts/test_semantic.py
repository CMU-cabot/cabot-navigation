import os
import json
from tqdm import tqdm
import argparse
import io
import requests
from PIL import Image
import torch
from transformers import AutoImageProcessor, AutoModel, AutoTokenizer
import numpy as np
from typing import Union, List, Optional, Set, Dict
import pickle


def cls_pooling(model_output, attention_mask):
    return model_output[0][:,0]


def extract_image_feature(image, processor, model, device) -> np.ndarray:
    inputs = processor(images=image, return_tensors="pt").to(device)
    with torch.no_grad():
        features = model.get_image_features(**inputs)
    return features.cpu().numpy()


def extract_text_feature(text: str, tokenizer, model, device, return_tensors: bool = False) -> Union[np.ndarray, torch.Tensor]:
    inputs = tokenizer(text, return_tensors="pt", padding=True, truncation=True).to(device)
    with torch.no_grad():
        outputs = model(**inputs)
    text_feature = cls_pooling(outputs, inputs["attention_mask"])
    if return_tensors:
        return text_feature
    else:
        return text_feature.cpu().numpy()


def concat_features(feature_a, feature_b):
    if feature_a.size > 0 and feature_b.size > 0:
        return np.concatenate([feature_a, feature_b], axis=0)
    elif feature_a.size == 0:
        return feature_b
    elif feature_b.size == 0:
        return feature_a


def main(images_dir: str, gpt_dir: str, extract_images: bool = False, extract_text: bool = False, log_dir: Optional[str] = None):
    device = "cuda" if torch.cuda.is_available() else "cpu"
    image_feature_path = None
    text_feature_path = None

    image_paths: List[str] = []
    if log_dir is None:
        # get image paths from images_dir
        # inside image_dir, there are 3 folders: log_frames_front, log_frames_left, log_frames_right
        # maybe some folders do not exist
        log_dirs = ["log_frames_front", "log_frames_left", "log_frames_right"]
        for each_log_dir in log_dirs:
            each_log_dir = os.path.join(images_dir, each_log_dir)
            if os.path.exists(each_log_dir):
                image_files = [x for x in os.listdir(each_log_dir) if x.endswith(".jpg")]
                image_paths.extend([os.path.join(each_log_dir, x) for x in image_files])
    else:
        # get image paths from log_dir
        image_dirs = [x for x in os.listdir(log_dir) if os.path.isdir(os.path.join(log_dir, x))]
        image_dirs = [x for x in image_dirs if x.endswith("images")]
        for image_dir in image_dirs:
            log_dir_image = os.path.join(log_dir, image_dir)
            image_files = [x for x in os.listdir(log_dir_image) if x.endswith(".jpg")]
            image_paths.extend([os.path.join(log_dir_image, x) for x in image_files])

    if extract_images:
        # load pre-extracted features
        image_feature_dict = None
        image_feature_path = f"{log_dir}/image_features.pickle"
        if os.path.exists(image_feature_path):
            print(f"loading pre-extracted image features ...")
            with open(image_feature_path, "rb") as f:
                image_feature_dict = pickle.load(f)
            already_extracted_image_paths: Set[str] = set(image_feature_dict.get("image_file_key", []))
        else:
            already_extracted_image_paths = set()
        image_paths = [
            x for x in image_paths
              if os.path.dirname(x).split("/")[-1] not in already_extracted_image_paths
        ]

        # set up CLIP image feature extractor
        HF_MODEL_PATH = 'line-corporation/clip-japanese-base'
        tokenizer = AutoTokenizer.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
        processor = AutoImageProcessor.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
        model = AutoModel.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
        model.to(device)

        # get image features
        all_features_front, all_features_left, all_features_right = [], [], []
        odoms = []
        image_file_key = []
        for img_path in tqdm(image_paths, ncols=80):
            image = Image.open(img_path)
            image_file_key.append(os.path.dirname(img_path).split("/")[-1])
            
            feature = extract_image_feature(image, processor, model, device)
            if "front" in img_path:
                all_features_front.append(feature)
            elif "left" in img_path:
                all_features_left.append(feature)
            elif "right" in img_path:
                all_features_right.append(feature)
            
            if "front" in img_path:
                odom_path = os.path.join(os.path.dirname(img_path), "odom.npy")
                if os.path.exists(odom_path):
                    odom = np.load(odom_path)
                    odoms.append(odom)
        all_features_front_array = np.array(all_features_front)
        all_features_left_array = np.array(all_features_left)
        all_features_right_array = np.array(all_features_right)

        # update features
        if image_feature_dict is not None:
            all_features_front_array = concat_features(image_feature_dict["front"], all_features_front_array)
            all_features_left_array = concat_features(image_feature_dict["left"], all_features_left_array)
            all_features_right_array = concat_features(image_feature_dict["right"], all_features_right_array)
            odoms = image_feature_dict.get("odoms", []) + odoms
            image_file_key = image_feature_dict.get("image_file_key", []) + image_file_key
        
        with open(image_feature_path, "wb") as f:
            pickle.dump({
                "front": all_features_front_array,
                "left": all_features_left_array,
                "right": all_features_right_array,
                "odoms": odoms,
                "image_file_key": image_file_key
            }, f)
        # np.save(image_feature_path, all_features_array)
        print(f"Features (shape ({all_features_front_array.shape}) x 3) saved to {image_feature_path} !")
        # release memory
        del processor, model

    if extract_text:
        # load pre-extracted features
        text_feature_path = f"{log_dir}/text_features.pickle"
        already_extracted_text_paths = set()
        if os.path.exists(text_feature_path):
            print(f"loading pre-extracted text features from {text_feature_path} ...")
            with open(text_feature_path, "rb") as f:
                text_feature_dict = pickle.load(f)
            already_extracted_text_paths: Set[str] = set(text_feature_dict.get("text_file_key", []))
        else:
            text_feature_dict = None
            already_extracted_text_paths = set()

        if log_dir is None:
            annos_path = f"{gpt_dir}/semantic_map/log.json"
            with open(annos_path, "r") as f:
                annos = json.load(f)
        else:
            annos = []
            image_dirs = [x for x in os.listdir(log_dir) if os.path.isdir(os.path.join(log_dir, x))]
            image_dirs = [x for x in image_dirs if x.endswith("images") and x not in already_extracted_text_paths]
            odoms = []
            for image_dir in image_dirs:
                gpt_jsonl = os.path.join(log_dir, image_dir, "explanation.jsonl")
                if not os.path.exists(gpt_jsonl):
                    continue
                with open(gpt_jsonl, "r") as f:
                    anno = [json.loads(line) for line in f.readlines()]
                annos.append(anno)

                odom_path = os.path.join(log_dir, image_dir, "odom.npy")
                if os.path.exists(odom_path):
                    odom = np.load(odom_path)
                    odoms.append(odom)
                else:
                    odoms.append(None)
                
        # load model
        # Load model from HuggingFace Hub
        tokenizer = AutoTokenizer.from_pretrained("cl-nagoya/sup-simcse-ja-large")
        model = AutoModel.from_pretrained("cl-nagoya/sup-simcse-ja-large")
        model.to(device)

        # get text features
        all_text_features = []
        for anno in tqdm(annos, ncols=80):
            text = anno["description"]
            text_feature = extract_text_feature(text, tokenizer, model, device)
            all_text_features.append(text_feature)
        all_text_features_array = np.array(all_text_features)

        # update features
        if text_feature_dict is not None:
            all_text_features_array = concat_features(text_feature_dict["features"], all_text_features_array)
            odoms = text_feature_dict.get("odoms", []) + odoms
            text_file_keys = text_feature_dict.get("text_file_key", []) + image_dirs
            print(image_dirs)
            texts = text_feature_dict.get("texts", []) + [anno["description"] for anno in annos]
        else:
            texts = [anno["description"] for anno in annos]
            text_file_keys = image_dirs

        if gpt_dir is not None:
            text_feature_path = f"{gpt_dir}/text_features.pickle"
        elif log_dir is not None:
            text_feature_path = f"{log_dir}/text_features.pickle"
        else:
            raise ValueError("gpt_dir or log_dir must be specified!")
        # np.save(text_feature_path, all_text_features_array)
        with open(text_feature_path, "wb") as f:
            pickle.dump({
                "features": all_text_features_array,
                "texts": texts,
                "odoms": odoms,
                "text_file_key": text_file_keys
            }, f)
        print(f"Text features (shape ({all_text_features_array.shape})) saved to {text_feature_path} !")
        # release memory
        del model
    return image_feature_path, text_feature_path


def extract_images_from_video(video_path: str, output_dir: str, annotation_file: str):
    with open(annotation_file, "r") as f:
        annos = json.load(f)
    
    seconds = [x["second"] for x in annos]

    os.makedirs(output_dir, exist_ok=True)
    for sec in tqdm(seconds, ncols=80):
        sec = int(sec)
        os.system(f"ffmpeg -i {video_path} -vf fps=1/1 -ss {sec} -frames:v 1 {output_dir}/{sec}.jpg -loglevel quiet")


def search(image_dir: str, gpt_dir: str, use_default_query: bool = False):
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # load annos
    annos_path = f"{gpt_dir}/semantic_map/log.json"
    with open(annos_path, "r") as f:
        annos = json.load(f)
    idx_to_sec = {i: int(anno["second"]) for i, anno in enumerate(annos)}
    
    # wait for user input
    if use_default_query:
        user_query = "木製の展示"
    else:
        user_query = input("Enter your query: ")

    # 1. search from CLIP feature
    print("\nSearching from CLIP feature")
    # 1.1 load model
    HF_MODEL_PATH = 'line-corporation/clip-japanese-base'
    tokenizer = AutoTokenizer.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
    processor = AutoImageProcessor.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
    model = AutoModel.from_pretrained(HF_MODEL_PATH, trust_remote_code=True)
    model.to(device)

    log_dirs = ["log_frames_front", "log_frames_left", "log_frames_right"]
    
    similarity_sum = np.zeros(len(annos))
    for log_dir in log_dirs:
        print(f"\nSearching from {log_dir} ...")
        # 1.2 load pre-extracted CLIP image feature
        log_dir = os.path.join(image_dir, log_dir)
        clip_image_features = np.load(f"{log_dir}/image_features.npy")
        clip_image_features = torch.from_numpy(clip_image_features).to(device)
        # torch.Size([76, 1, 512]) -> torch.Size([76, 512])
        clip_image_features = clip_image_features.squeeze(1)

        # 1.3 get text features
        text = tokenizer(user_query).to(device)
        with torch.no_grad():
            text_clip_feature = model.get_text_features(**text)
        # 1.4 get similarity
        similarity = (text_clip_feature @ clip_image_features.T).softmax(dim=-1).cpu().numpy().squeeze()
        max_sim_idx = np.argmax(similarity)
        print(f"Most similar image is {max_sim_idx} ({idx_to_sec[max_sim_idx]}) with similarity {similarity[max_sim_idx]}")
        print(f"  text: {annos[max_sim_idx]['description']}")
        print(f"  image: {os.path.join(log_dir, f'{idx_to_sec[max_sim_idx]}.jpg')}")
        similarity_sum += similarity

    # 2. search from text feature
    log_dir = os.path.join(image_dir, "log_frames_front")
    # 2.1 load model
    tokenizer = AutoTokenizer.from_pretrained("cl-nagoya/sup-simcse-ja-large")
    model = AutoModel.from_pretrained("cl-nagoya/sup-simcse-ja-large")
    model.to(device)

    # 2.2 load pre-extracted text feature
    print("\nSearching from text feature")
    text_pre_features = np.load(f"{gpt_dir}/text_features.npy")
    text_pre_features = torch.from_numpy(text_pre_features).to(device)
    # torch.Size([76, 1, 1024]) -> torch.Size([76, 1024])
    text_pre_features = text_pre_features.squeeze(1)

    # 2.3 get text features
    text_input_feature = extract_text_feature(user_query, tokenizer, model, device, return_tensors=True)
    # 2.4 get similarity
    similarity = (text_input_feature @ text_pre_features.T).softmax(dim=-1).cpu().numpy().squeeze()
    max_sim_idx = np.argmax(similarity)
    print(f"Most similar text is {max_sim_idx} ({idx_to_sec[max_sim_idx]}) with similarity {similarity[max_sim_idx]}")
    print(f"  text: {annos[max_sim_idx]['description']}")
    print(f"  image: {os.path.join(log_dir, f'{idx_to_sec[max_sim_idx]}.jpg')}")
    similarity_sum += similarity


    # calculate sum of similarity
    print("\nSearching from sum of CLIP and text feature...")
    max_sim_idx = np.argmax(similarity_sum)
    print(f"\nMost similar image is {max_sim_idx} ({idx_to_sec[max_sim_idx]}) with similarity {similarity_sum[max_sim_idx]}")
    print(f"  text: {annos[max_sim_idx]['description']}")
    print(f"  image: {os.path.join(log_dir, f'{idx_to_sec[max_sim_idx]}.jpg')}")


if __name__ == '__main__':
    args = argparse.ArgumentParser()
    # args.add_argument("--images_dir", type=str, help="Path to the images directory")
    # args.add_argument("--gpt_dir", type=str, help="Path to the gpt captions directory")
    args.add_argument("--extract", action="store_true", help="Extract images from video")
    args.add_argument("--extract_image_features", "-i", action="store_true", help="Extract image features from images")
    args.add_argument("--extract_text_features", "-t", action="store_true", help="Extract text features from text")
    args.add_argument("--search", action="store_true", help="Search from user query")
    args.add_argument("--use_default_query", action="store_true", help="Use default query")
    args.add_argument("--log_dir", type=str, help="Log directory")
    args.add_argument("--video_dir", type=str, help="Path to the video directory")
    args.add_argument("--save_dir", type=str, help="Path to the save directory")
    args.add_argument("--annotation_dir", type=str, help="Path to the annotation directory")
    args.add_argument("--image_dir", type=str, help="Path to the image directory")
    args.add_argument("--gpt_dir", type=str, help="Path to the gpt directory")
    args = args.parse_args()

    if args.extract:
        # extract images from video; front, left, right
        print("Extracting images from front video")
        extract_images_from_video(
            video_path=f"{args.video_dir}/out_front.mp4",
            output_dir=f"{args.save_dir}/log_frames_front",
            annotation_file=f"{args.annotation_dir}/log.json"
        )
        print("Extracting images from left video")
        extract_images_from_video(
            video_path=f"{args.video_dir}/out_left.mp4",
            output_dir=f"{args.save_dir}/log_frames_left",
            annotation_file=f"{args.annotation_dir}/log.json"
        )
        print("Extracting images from right video")
        extract_images_from_video(
            video_path=f"{args.video_dir}/out_right.mp4",
            output_dir=f"{args.save_dir}/log_frames_right",
            annotation_file=f"{args.annotation_dir}/log.json"
        )


    if args.extract_image_features or args.extract_text_features:
        if args.log_dir:
            main(
                images_dir=None,
                gpt_dir=None,
                extract_images=args.extract_image_features,
                extract_text=args.extract_text_features,
                log_dir=args.log_dir
            )
        else:
            main(
                images_dir=args.image_dir,
                gpt_dir=args.gpt_dir,
                extract_images=args.extract_image_features,
                extract_text=args.extract_text_features
            )
    
    if args.search:
        search(
            image_dir=args.image_dir,
            gpt_dir=args.gpt_dir,
            use_default_query=args.use_default_query
        )
