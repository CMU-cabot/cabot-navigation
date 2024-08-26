# Copyright (c) 2020, 2022  Carnegie Mellon University
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

from cabot_ui import navigation
from std_msgs.msg import String
from rclpy.node import Node


class Exploration():
    def __init__(self, node: Node):

        super(Exploration, self).__init__()
        self.node = node
        self.pause = False
        self.in_conversation = False
        self.in_button_control = False
        self.publisher = self.node.create_publisher(String, '/cabot/user_query', 10)

    def send_query(self, query_type, query_string):
        msg = String()
        msg.data = f"{query_type};{query_string}"
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: {msg.data}")

    def set_pause_control(self, pause):
        self.pause = pause

    def get_pause_control(self):
        return self.pause
    
    def set_conversation_control(self, in_conversation):
        self.in_button_control = False
        self.in_conversation = in_conversation

    def get_conversation_control(self):
        return self.in_conversation
    
    def set_button_control(self, in_button_control):
        self.in_conversation = False
        self.in_button_control = in_button_control

    def get_button_control(self):
        return self.in_button_control