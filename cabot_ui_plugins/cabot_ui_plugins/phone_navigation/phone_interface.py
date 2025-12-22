# Copyright (c) 2025  Carnegie Mellon University
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

from cabot_ui import geojson
from cabot_ui import i18n
import cabot_msgs.action
from rclpy.action import ActionClient
import traceback


class SpeechPriority:
    REQUIRED = 90
    HIGH = 60
    NORMAL = 30
    MODERATE = 25
    LOW = 10


class PhoneInterface:
    def __init__(self, node):
        self.node = node
        self._speak_client = ActionClient(self.node, cabot_msgs.action.Speak, "/speak")

    def start_navigation(self, to_id):
        target_facility = None
        facilities = geojson.Object.get_objects_by_exact_type(geojson.Facility)
        for facility in facilities:
            for entrance in facility.entrances:
                if entrance.node._id == to_id:
                    target_facility = facility
                    break
        if not target_facility:
            self.node.get_logger().warn(f"Could not find facility for to_id {to_id}")
            return

        name = target_facility.name_pron if target_facility.name_pron else target_facility.name
        text = i18n.localized_string("PHONE_STARTING_NAVIGATION_TO_FACILITY", name)
        self.speak(text, force=True, priority=SpeechPriority.REQUIRED)

    def have_completed(self, to_id):
        target_facility = None
        facilities = geojson.Object.get_objects_by_exact_type(geojson.Facility)
        for facility in facilities:
            for entrance in facility.entrances:
                if entrance.node._id == to_id:
                    target_facility = facility
                    break
        if not target_facility:
            self.node.get_logger().warn(f"Could not find facility for to_id {to_id}")
            return

        name = target_facility.name_pron if target_facility.name_pron else target_facility.name
        description = target_facility.description if target_facility.description else ""
        text = i18n.localized_string("PHONE_ARRIVED_AT_FACILITY", name, description)
        self.speak(text, force=True, priority=SpeechPriority.REQUIRED)

    def speak(self, text, force=False, pitch=50, volume=50, rate=50, priority=SpeechPriority.NORMAL):
        if text is None:
            return

        # TODO:
        voice = 'male'
        rate = 50

        try:
            if not self._speak_client.wait_for_server(timeout_sec=1):
                self.node.get_logger().error("Speak action server not available")
                return
            self.node.get_logger().info(F"try to speak {text} (v={voice}, r={rate}, p={pitch}, priority={priority}) {force}")
            goal = cabot_msgs.action.Speak.Goal()
            goal.request_id = ""
            goal.text = text
            goal.rate = rate
            goal.pitch = pitch
            goal.volume = volume
            goal.lang = i18n._lang
            goal.voice = voice
            goal.force = force
            goal.priority = priority
            goal.timeout = 2.0
            goal.channels = cabot_msgs.action.Speak.Goal.CHANNEL_BOTH
            future = self._speak_client.send_goal_async(goal)
            future.add_done_callback(self._handle_speak_goal)
            self.node.get_logger().info("speak goal sent")
        except:  # noqa: E722
            self.node.get_logger().error(traceback.format_exc())

    def _handle_speak_goal(self, future):
        if future.cancelled():
            self.node.get_logger().error("Speak goal sending cancelled")
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Speak goal rejected")
            return
        self.node.get_logger().info("Speak goal accepted")
