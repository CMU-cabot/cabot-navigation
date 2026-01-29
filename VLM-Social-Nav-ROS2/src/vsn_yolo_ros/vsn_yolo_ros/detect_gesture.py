#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
except Exception:
    # Fallback for older ultralytics layouts that don't export YOLO in __init__.
    from ultralytics.yolo.engine.model import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.utils import torch_utils

from sensor_msgs.msg import Image, CompressedImage
from vsn_yolo_msgs.msg import Detections, Detection, BoundingBox

import cv2, torch
from typing import List, Dict
from cv_bridge import CvBridge

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_gesture')
        
        try:
             ROOT = get_package_share_directory('vsn_yolo_ros')
        except:
             ROOT = "."

        self.declare_parameter("yolo_model", "yolov8_gesture.pt")
        self.yolo_model = self.get_parameter("yolo_model").value
        
        self.declare_parameter("confidence_threshold", 0.25)
        self.conf_thres = self.get_parameter("confidence_threshold").value

        self.declare_parameter("iou_threshold", 0.45)
        self.iou_thres = self.get_parameter("iou_threshold").value

        self.declare_parameter("maximum_detections", 300)
        self.max_det = self.get_parameter("maximum_detections").value

        self.declare_parameter("result_conf", True)
        self.result_conf = self.get_parameter("result_conf").value

        self.declare_parameter("result_line_width", 1) 
        self.result_line_width = self.get_parameter("result_line_width").value

        self.declare_parameter("result_font_size", 1.0)
        self.result_font_size = self.get_parameter("result_font_size").value

        self.declare_parameter("result_font", "Arial.ttf")
        self.result_font = self.get_parameter("result_font").value

        self.declare_parameter("result_labels", True)
        self.result_labels = self.get_parameter("result_labels").value

        self.declare_parameter("result_boxes", True)
        self.result_boxes = self.get_parameter("result_boxes").value

        self.declare_parameter("view_image", False)
        self.view_image = self.get_parameter("view_image").value

        self.declare_parameter("input_image_topic", "/zed2i/zed_node/rgb/image_rect_color/compressed")
        input_image_topic = self.get_parameter("input_image_topic").value
        
        self.compressed_input = "compressed" in input_image_topic

        self.declare_parameter("output_topic", "/yolo_gesture_result")
        output_topic = self.get_parameter("output_topic").value

        self.declare_parameter("publish_image", False)
        self.publish_image = self.get_parameter("publish_image").value
        
        self.declare_parameter("output_image_topic", "/yolo_gesture_image")
        output_image_topic = self.get_parameter("output_image_topic").value
        
        self.declare_parameter("device", "cuda:0")
        device_str = self.get_parameter("device").value

        model_path = f"{ROOT}/models/{self.yolo_model}"
        try:
            self.model = YOLO(model_path)
        except Exception as e:
             self.get_logger().error(f"Failed to load model from {model_path}: {e}")
             self.model = YOLO("yolov8n.pt") # Fallback?

        self.bridge = CvBridge()
        try:
            self.device = torch_utils.select_device(device_str)
        except:
             self.device = "cpu"

        if self.compressed_input:
            self.create_subscription(CompressedImage, input_image_topic, self.image_callback, 1)
        else:
            self.create_subscription(Image, input_image_topic, self.image_callback, 1)

        self.pub = self.create_publisher(Detections, output_topic, 10)
        
        if self.publish_image:
            self.result_image_pub = self.create_publisher(Image, output_image_topic, 10)

    def image_callback(self, msg):
        try:
            if self.compressed_input:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            h, w, c = cv_image.shape
            if h > 0:
                new_size = (int(640*w/h), 640)
                cv_image = cv2.resize(cv_image, new_size)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        results = self.model.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=self.conf_thres,
            device=self.device,
            classes=3
        )
        if len(results) == 0:
             return
             
        results: Results = results[0].cpu()

        hypothesis = []
        boxes = []

        if results.boxes:
            hypothesis = self.parse_hypothesis(results)
            boxes = self.parse_boxes(results)

        detections_msg = Detections()
        detections_msg.header = msg.header

        for i in range(len(hypothesis)):
            aux_msg = Detection()
            aux_msg.class_id = hypothesis[i]["class_id"]
            aux_msg.class_name = hypothesis[i]["class_name"]
            aux_msg.score = hypothesis[i]["score"]
            if i < len(boxes):
                aux_msg.bbox = boxes[i]
            detections_msg.detections.append(aux_msg)

        self.pub.publish(detections_msg)

        if len(results) != 0:
            result_image_msg, plotted_image = self.create_result_image(results)

            if self.publish_image:
                result_image_msg.header = msg.header
                self.result_image_pub.publish(result_image_msg)

            if self.view_image and plotted_image is not None:
                cv2.imshow("YOLO Result", plotted_image)
                cv2.waitKey(1)


    def parse_hypothesis(self, results: Results) -> List[Dict]:
        hypothesis_list = []
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.model.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)
        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox]:
        boxes_list = []
        for box_data in results.boxes:
            msg = BoundingBox()
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])
            boxes_list.append(msg)
        return boxes_list

    def create_result_image(self, results):
        plotted_image = results.plot(
            conf=self.result_conf,
            line_width=self.result_line_width,
            font_size=self.result_font_size,
            font=self.result_font,
            labels=self.result_labels,
            boxes=self.result_boxes,
        )
        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
        return result_image_msg, plotted_image


def main(args=None):
    rclpy.init(args=args)
    detector = YoloDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
