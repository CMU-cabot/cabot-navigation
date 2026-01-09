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
import traceback
from typing import Callable

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ProcessQueue:
    def __init__(self, node: Node):
        self._node = node
        self._queue = []
        self._queue_lock = threading.Lock()
        self._process_timer = node.create_timer(0.01, self._process_queue_func, callback_group=MutuallyExclusiveCallbackGroup())

    def add(self, func: Callable, *args, **kwargs):
        with self._queue_lock:
            self._queue.append((func, args, kwargs))

    def _process_queue_func(self):
        with self._queue_lock:
            if not self._queue:
                return
            func, args, kwargs = self._queue.pop(0)
        try:
            func(*args, **kwargs)
        except Exception as e:
            self._node.get_logger().error(f"Error processing queue item: {e}\n{traceback.format_exc()}")
