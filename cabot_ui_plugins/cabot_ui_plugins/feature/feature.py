from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from cabot_ui.event import NavigationEvent
from cabot_ui.process_queue import ProcessQueue
import std_msgs.msg


class Feature():
    def __init__(self, node_manager):
        self._node = node_manager.get_node("feature", True)
        self.delegate = None
        self._process_queue = ProcessQueue(self._node)
        self._eventPub = self._node.create_publisher(std_msgs.msg.String, "/cabot/event", 10, callback_group=MutuallyExclusiveCallbackGroup())

        # request language
        e = NavigationEvent("getlanguage", None)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

        def handleside_callback(msg):
            # request handleside
            self.handleside = msg.data
            self._send_handleside()
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.handleside_sub = self._node.create_subscription(std_msgs.msg.String, "/cabot/features/handleside", handleside_callback, qos_profile)

        def touchmode_callback(msg):
            # request touchmode
            self.touchmode = msg.data
            self._send_touchmode()
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.touchmode_sub = self._node.create_subscription(std_msgs.msg.String, "/cabot/features/touchmode", touchmode_callback, qos_profile)

    @property
    def ready(self) -> bool:
        return True

    def process_event(self, event) -> bool:
        if event.subtype == "reqfeatures":
            self._send_handleside()
            self._send_touchmode()
            return False   # can be handled by other plugins

        # ignore get event
        if event.subtype == "getlanguage":
            return True
        if event.subtype == "gethandleside":
            return True
        if event.subtype == "gettouchmode":
            return True

        # operations indepent from the navigation state
        if event.subtype == "language":
            self.delegate.change_language(event.param)
            return True

        if event.subtype == "handleside":
            self._logger.info("calling set_handle_side")
            self._process_queue.add(self._set_handle_side, event.param)
            return True

        if event.subtype == "touchmode":
            self._logger.info("calling set_touch_mode")
            self._process_queue.add(self._set_touch_mode, event.param)
            return True
        return False

    def _send_handleside(self):
        e = NavigationEvent("gethandleside", self.handleside)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

    def _send_touchmode(self):
        e = NavigationEvent("gettouchmode", self.touchmode)
        msg = std_msgs.msg.String()
        msg.data = str(e)
        self._eventPub.publish(msg)

    def _set_handle_side(self, side):
        self._logger.info("_set_handle_side is called")

        def callback(result):
            self.delegate.activity_log("cabot/navigation", "set_handle_side", "side")
            self._logger.info(f"set_handle_side {side=}, {result=}")
        if side == "left":
            offset_sign = +1.0
        elif side == "right":
            offset_sign = -1.0
        else:
            self._logger.info(f"set_handle_side {side=} should be 'left' or 'right'")
            return
        self.change_parameters({
            "/footprint_publisher": {
                "offset_sign": offset_sign
            }
        }, callback)

    def _set_touch_mode(self, mode):
        self._logger.info("_set_touch_mode is called")

        def callback(result):
            self.delegate.activity_log("cabot/navigation", "set_touch_mode", "mode")
            self._logger.info(f"set_touch_mode {mode=}, {result=}")
        self.change_parameters({
            "/cabot/cabot_can": {
                "touch_mode": mode
            }
        }, callback)
