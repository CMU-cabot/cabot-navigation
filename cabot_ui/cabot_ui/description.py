# Copyright (c) 2024  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import traceback
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Path
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import requests
import json
import os
import math
from enum import Enum


class DescriptionMode(Enum):
    SURROUND = 'surround'
    STOP_REASON = 'stop-reason'
    STOP_REASON_DATA_COLLECT = 'stop-reason-data-collect'


class Description:
    DESCRIPTION_API = 'description'
    DESCRIPTION_WITH_IMAGES_API = 'description_with_live_image'
    STOP_REASON_API = 'stop_reason'

    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()
        self.max_size = 512
        self.max_distance = node.declare_parameter("description_max_distance", 100).value
        self.last_images = {'left': None, 'front': None, 'right': None}
        self.last_plan_distance = None
        self.host = os.environ.get("CABOT_IMAGE_DESCRIPTION_SERVER", "http://localhost:8000")
        self.enabled = (os.environ.get("CABOT_IMAGE_DESCRIPTION_ENABLED", "false").lower() == "true")
        if self.enabled:
            # handle modes (default=surround)
            modes = os.environ.get("CABOT_IMAGE_DESCRIPTION_MODE", DescriptionMode.SURROUND.value).split(",")
            self.surround_enabled = DescriptionMode.SURROUND.value in modes
            self.stop_reason_enabled = DescriptionMode.STOP_REASON.value in modes
            self.stop_reason_data_collect_enabled = DescriptionMode.STOP_REASON_DATA_COLLECT.value in modes
            if self.stop_reason_data_collect_enabled:
                self.stop_reason_enabled = False
                self.surround_enabled = False
                self.memo_pub = self.node.create_publisher(String, '/memo', 10)
        else:
            self.surround_enabled = False
            self.stop_reason_enabled = False
            self.stop_reason_data_collect_enabled = False
        self._logger = rclpy.logging.get_logger("cabot_ui_manager.description")
        self._logger.info(F"Description enabled: {self.enabled}")
        self._logger.info(F"Description surround: {self.surround_enabled}")
        self._logger.info(F"Description stop reason: {self.stop_reason_enabled}")
        self._logger.info(F"Description stop reason data collect: {self.stop_reason_data_collect_enabled}")

        self.subscriptions = {
            'left': self.node.create_subscription(
                CompressedImage,
                '/rs3/color/image_raw/compressed',  # Change this to your left image topic name
                lambda msg: self.image_callback('left', msg),
                10
            ),
            'front': self.node.create_subscription(
                CompressedImage,
                '/rs1/color/image_raw/compressed',  # Change this to your front image topic name
                lambda msg: self.image_callback('front', msg),
                10
            ),
            'right': self.node.create_subscription(
                CompressedImage,
                '/rs2/color/image_raw/compressed',  # Change this to your right image topic name
                lambda msg: self.image_callback('right', msg),
                10
            )
        }

        # rotate modes
        rotate_left = (os.environ.get("CABOT_IMAGE_DESCRIPTION_ROTATE_LEFT", "false").lower() == "true")
        rotate_front = (os.environ.get("CABOT_IMAGE_DESCRIPTION_ROTATE_FRONT", "false").lower() == "true")
        rotate_right = (os.environ.get("CABOT_IMAGE_DESCRIPTION_ROTATE_RIGHT", "false").lower() == "true")
        self.image_rotate_modes = {
            'left': cv2.ROTATE_180 if rotate_left else None,
            'front': cv2.ROTATE_180 if rotate_front else None,
            'right': cv2.ROTATE_180 if rotate_right else None,
        }

        self.plan_sub = self.node.create_subscription(Path, '/plan', self.plan_callback, 10)

    def image_callback(self, position: str, msg: CompressedImage):
        try:
            # Store the raw ROS CompressedImage message
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

    def request_description(self, global_position):
        self._logger.info(F"Request Description at {global_position}")
        try:
            req = requests.get(
                F"{self.host}/{Description.DESCRIPTION_API}?lat={global_position.lat}&lng={global_position.lng}&rotation={global_position.r}&max_distance={self.max_distance}"
            )
            data = json.loads(req.text)
            if req.status_code != requests.codes.ok:
                self._logger.error(F"Request Error {data=}")
                return
            self._logger.info(F"Request result {data['description']=}")
        except Exception as error:
            self._logger.error(F"Request Error {error=}")

    def request_description_with_images(self, global_position, length_index=0):
        if not self.enabled:
            return None
        if not self.surround_enabled and not self.stop_reason_data_collect_enabled:
            return None

        API_URL = F"{self.host}/{Description.DESCRIPTION_WITH_IMAGES_API}"
        if self.stop_reason_data_collect_enabled:
            self.memo_pub.publish(String(data="stop_reason_data_collect"))
            API_URL = F"{self.host}/{Description.STOP_REASON_API}"

        self._logger.info(F"Request Description with images at {global_position} to {API_URL}")
        image_data_list = []
        distance_to_travel = 100
        if self.last_plan_distance:
            distance_to_travel = self.last_plan_distance
        for position, img_msg in self.last_images.items():
            if img_msg is not None:
                try:
                    img = self.bridge.compressed_imgmsg_to_cv2(img_msg)
                    # Rotate the image if necessary
                    if self.image_rotate_modes[position] is not None:
                        img = cv2.rotate(img, self.image_rotate_modes[position])
                    # Resize the image while maintaining the aspect ratio
                    height, width = img.shape[:2]
                    if height > width:
                        new_height = self.max_size
                        new_width = int(width * new_height / height)
                    else:
                        new_width = self.max_size
                        new_height = int(height * new_width / width)
                    img = cv2.resize(img, (new_width, new_height))
                    img_msg = self.bridge.cv2_to_compressed_imgmsg(img)

                    # Use the raw JPEG data from the ROS CompressedImage message
                    base64_image = base64.b64encode(img_msg.data).decode('utf-8')
                    image_uri = f'data:image/jpeg;base64,{base64_image}'

                    # Prepare JSON data
                    image_data_list.append({
                        'position': position,
                        'image_uri': image_uri
                    })
                except Exception as e:
                    self._logger.error(f'Error converting {position} image: {e}')
                    self._logger.error(traceback.format_exc())
            else:
                self._logger.warn(f'No {position} image available to process.')

        # Send HTTP request with the image data
        try:
            headers = {'Content-Type': 'application/json'}
            json_data = json.dumps(image_data_list)
            self._logger.debug(F"Request data: {image_data_list}")
            lat = global_position.lat
            lng = global_position.lng
            rotation = global_position.r
            max_distance = self.max_distance
            response = requests.post(
                F"{API_URL}?{lat=}&{lng=}&{rotation=}&{max_distance=}&{length_index=}&{distance_to_travel=}",
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
