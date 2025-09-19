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

import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf2_ros
import tf2_geometry_msgs  # noqa: to register class for transform
import std_msgs.msg
from cabot_msgs.srv import LookupTransform


class BufferProxy():
    def __init__(self, node):
        self._clock = node.get_clock()
        self._logger = node.get_logger()
        self.lookup_transform_service = node.create_client(LookupTransform, 'lookup_transform', callback_group=MutuallyExclusiveCallbackGroup())
        self.countPub = node.create_publisher(std_msgs.msg.Int32, "transform_count", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.transformMap = {}
        self.min_interval = rclpy.duration.Duration(seconds=0.2)

    def debug(self):
        if not hasattr(self, "count"):
            self.count = 0
        self.count += 1
        msg = std_msgs.msg.Int32()
        msg.data = self.count
        self.countPub.publish(msg)

    # buffer interface
    def lookup_transform(self, target, source, time=None):
        # find the latest saved transform first
        key = f"{target}-{source}"
        now = self._clock.now()
        if key in self.transformMap:
            (transform, last_time) = self.transformMap[key]
            if now - last_time < self.min_interval:
                self._logger.debug(f"found old lookup_transform({target}, {source}, {(now - last_time).nanoseconds/1000000000:.2f}sec)")
                return transform

        if __debug__:
            self.debug()
        self._logger.debug(f"lookup_transform({target}, {source})")
        req = LookupTransform.Request()
        req.target_frame = target
        req.source_frame = source
        if not self.lookup_transform_service.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("lookup transform service is not available")
        # usage of call() can cause SIGSEGV at exit due to infinite wait
        # however, call(req, timeout_sec=1.0) is implemented in J-turtle release not in humble
        #
        # result = self.lookup_transform_service.call(req)
        #
        # so implemented call with timeout from here
        # reference implementation is
        # https://github.com/ros2/rclpy/blob/7a7f23e0d7e51dd244001ef606b97f1153e5a97e/rclpy/rclpy/client.py#L72-L110
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()
        future = self.lookup_transform_service.call_async(req)
        future.add_done_callback(unblock)
        # Check future.done() before waiting on the event.
        # The callback might have been added after the future is completed,
        # resulting in the event never being set.
        if not future.done():
            if not event.wait(1.0):
                # Timed out. remove_pending_request() to free resources
                self.lookup_transform_service.remove_pending_request(future)
                raise RuntimeError("timeout")
        if future.exception() is not None:
            raise future.exception()
        result = future.result()
        # sync call end here

        if result.error.error > 0:
            raise RuntimeError(result.error.error_string)
        self.transformMap[key] = (result.transform, now)
        return result.transform

    def transform(self, pose_stamped, target):
        do_transform = tf2_ros.TransformRegistration().get(type(pose_stamped))
        transform = self.lookup_transform(target, pose_stamped.header.frame_id)
        return do_transform(pose_stamped, transform)
