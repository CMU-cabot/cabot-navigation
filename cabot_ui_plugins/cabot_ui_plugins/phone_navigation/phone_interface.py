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

    def start_navigation(self, to_id, route_overview=None, leaving=False, callback=None):
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
        leaving_text = i18n.localized_string("PHONE_LEAVING_INSTRUCTION") if leaving else None

        def after_start(_result):
            def speak_route_overview(_result):
                if route_overview:
                    self.speak(route_overview, force=True, priority=SpeechPriority.REQUIRED, callback=callback)
                elif callback is not None:
                    callback(_result)
            if leaving:
                self.speak(leaving_text, force=True, priority=SpeechPriority.REQUIRED, callback=speak_route_overview)
            else:
                speak_route_overview(_result)

        if route_overview or leaving:
            self.speak(text, force=True, priority=SpeechPriority.REQUIRED, callback=after_start)
        else:
            self.speak(text, force=True, priority=SpeechPriority.REQUIRED, callback=callback)

    def start_navigation_suitcase(self, to_id, callback=None):
        self.start_navigation(to_id, route_overview=None, leaving=False, callback=callback)

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

    def have_completed_suitcase(self, to_id):
        self.have_completed(to_id)

    def speak(self, text, force=False, pitch=50, volume=50, rate=50, priority=SpeechPriority.NORMAL, callback=None):
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
            future.add_done_callback(lambda future: self._handle_speak_goal(future, callback))
            self.node.get_logger().info("speak goal sent")
        except:  # noqa: E722
            self.node.get_logger().error(traceback.format_exc())

    def _handle_speak_goal(self, future, callback):
        if future.cancelled():
            self.node.get_logger().error("Speak goal sending cancelled")
            if callback is not None:
                callback(None)
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Speak goal rejected")
            if callback is not None:
                callback(None)
            return
        self.node.get_logger().info("Speak goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self._handle_speak_result(future, callback))

    def _handle_speak_result(self, future, callback):
        if future.cancelled():
            self.node.get_logger().error("Speak result cancelled")
            if callback is not None:
                callback(None)
            return
        result = future.result().result
        self.node.get_logger().info(
            f"Speak result: success={result.success} message='{result.message}'"
        )
        if callback is not None:
            callback(result)
