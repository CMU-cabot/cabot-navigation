import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from cabot_msgs.msg import Log
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import requests
import json
import os
import math

class Description:
    DESCRIPTION_API = 'description'
    DESCRIPTION_WITH_IMAGES_API = 'description_with_live_image'

    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()
        self.max_size = 512
        self.max_distance = node.declare_parameter("description_max_distance", 100).value
        self.last_images = {'left': None, 'front': None, 'right': None}
        self.last_plan_distance = None
        self.host = os.environ.get("CABOT_DESCRIPTION_SERVER", "http://localhost:8000")
        self._logger = rclpy.logging.get_logger("cabot_ui_manager.description")
        
        self.subscriptions = {
            'left': self.node.create_subscription(
                Image,
                '/rs3/color/image_raw',  # Change this to your left image topic name
                lambda msg: self.image_callback('left', msg),
                10
            ),
            'front': self.node.create_subscription(
                Image,
                '/rs1/color/image_raw',  # Change this to your front image topic name
                lambda msg: self.image_callback('front', msg),
                10
            ),
            'right': self.node.create_subscription(
                Image,
                '/rs2/color/image_raw',  # Change this to your right image topic name
                lambda msg: self.image_callback('right', msg),
                10
            )
        }

        # rotate modes
        self.image_rotate_modes = {
            'left': cv2.ROTATE_180,
            'front': cv2.ROTATE_180,
            'right': None
        }

        self.plan_sub = self.node.create_subscription(Path, '/plan', self.plan_callback, 10)

    def image_callback(self, position: str, msg: Image):
        try:
            # Store the raw ROS Image message
            self.last_images[position] = msg
        except Exception as e:
            self._logger.error(f'Error processing {position} image: {e}')

    def plan_callback(self, msg):
        prev = None
        dist = 0
        for pose in msg.poses:
            if prev is None:
                prev = pose
                continue
            dx = pose.pose.position.x - prev.pose.position.x
            dy = pose.pose.position.y - prev.pose.position.y
            dist += math.sqrt(dx * dx + dy * dy)
            prev = pose
        self.last_plan_distance = dist

    def request_description(self, gp):
        self._logger.info(F"Request Description at {gp}")
        try:
            req = requests.get(
                F"{self.host}/{Description.DESCRIPTION_API}?lat={gp.lat}&lng={gp.lng}&rotation={gp.r}&max_distance={self.max_distance}"
            )
            data = json.loads(req.text)
            if req.status_code != requests.codes.ok:
                self._logger.error(F"Request Error {data=}")
                return
            self._logger.info(F"Request result {data['description']=}")
        except Exception as error:
            self._logger.error(F"Request Error {error=}")

    def request_description_with_images(self, gp, length_index=0):
        self._logger.info(F"Request Description with images at {gp}")
        image_data_list = []
        distance_to_travel = 100
        if self.last_plan_distance:
            distance_to_travel = self.last_plan_distance
        for position, img_msg in self.last_images.items():
            if img_msg is not None:
                try:
                    # Convert ROS Image message to OpenCV image
                    cv_image_raw = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

                    # Rotate image if necessary
                    rotate_mode = self.image_rotate_modes[position]
                    if rotate_mode is not None:
                        cv_image = cv2.rotate(cv_image_raw, rotate_mode)
                        self._logger.info(f'Rotating {position} image: with {rotate_mode}')
                    else:
                        cv_image = cv_image_raw
                    
                    # Resize image while keeping the aspect ratio
                    h, w = cv_image.shape[:2]
                    if max(h, w) > self.max_size:
                        scale = self.max_size / float(max(h, w))
                        new_w = int(w * scale)
                        new_h = int(h * scale)
                        resized_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
                    else:
                        resized_image = cv_image
                    
                    # Convert OpenCV image to JPEG
                    _, buffer = cv2.imencode('.jpg', resized_image)
                    
                    # Encode JPEG to base64 and create image URI
                    base64_image = base64.b64encode(buffer).decode('utf-8')
                    image_uri = f'data:image/jpeg;base64,{base64_image}'
                    
                    # Prepare JSON data
                    image_data_list.append({
                        'position': position,
                        'image_uri': image_uri
                    })
                except Exception as e:
                    self._logger.error(f'Error converting {position} image: {e}')
            else:
                self._logger.warn(f'No {position} image available to process.')
        
        # Send HTTP request with the image data
        try:
            headers = {'Content-Type': 'application/json'}
            json_data = json.dumps(image_data_list)
            self._logger.debug(F"Request data: {image_data_list}")
            lat = gp.lat
            lng = gp.lng
            rotation = gp.r
            max_distance = self.max_distance
            response = requests.post(
                F"{self.host}/{Description.DESCRIPTION_WITH_IMAGES_API}?{lat=}&{lng=}&{rotation=}&{max_distance=}&{length_index=}&{distance_to_travel=}",
                headers=headers,
                data=json_data
            )
            if response.status_code == 200:
                response_json = response.json()
                self._logger.info('Successfully sent images to server.')
                return response_json
            else:
                self._logger.error(f'Failed to send images to server. Status code: {response.status_code}')
        except Exception as e:
            self._logger.error(f'Error sending HTTP request: {e}')
        return None
