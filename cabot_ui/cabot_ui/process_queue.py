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
