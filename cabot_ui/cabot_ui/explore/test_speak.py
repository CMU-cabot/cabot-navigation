import rclpy
from rclpy.node import Node
from rclpy.exceptions import InvalidServiceNameException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import cabot_msgs.msg
import cabot_msgs.srv


class Speaker(object):
    def __init__(self, node: Node, debug: bool = False):
        self._node = node
        self.lang = node.declare_parameter("language", "en").value
        # Comment out for debug
        if debug:
            self.speak('hello world!こんにちは')

    def speak(self, text, force=False, pitch=50, volume=50, rate=50):
        if text is None:
            return

        voice = 'male'
        rate = 50

        speak_proxy = self._node.create_client(cabot_msgs.srv.Speak, '/speak', callback_group=MutuallyExclusiveCallbackGroup())
        try:
            if not speak_proxy.wait_for_service(timeout_sec=1):
                print(f"Service cannot be found; tried to speak '{text}' (v={voice}, r={rate}, p={pitch}) {force}")
                return
            print(f"try to speak {text} (v={voice}, r={rate}, p={pitch}) {force}")
            request = cabot_msgs.srv.Speak.Request()
            request.text = text
            request.rate = rate
            request.pitch = pitch
            request.volume = volume
            request.lang = self.lang
            request.voice = voice
            request.force = force
            request.priority = 50
            request.timeout = 2.0
            request.channels = cabot_msgs.srv.Speak.Request.CHANNEL_BOTH
            speak_proxy.call_async(request)
            # CaBotRclpyUtil.info("speak requested")
        except InvalidServiceNameException as e:
            print(f"Service call failed: {e}")
            pass
            # CaBotRclpyUtil.error(F"Service call failed: {e}")


def speak_text(text: str, force=False):
    if not rclpy.ok():
        reinit = True
    else:
        reinit = False
    
    if reinit:
        rclpy.init()
    node = Node('speaker', start_parameter_services=False)
    speaker = Speaker(node=node)
    speaker.speak(text=text, force=force)
    node.destroy_node()
    # if reinit:
    #     rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    node = Node('speaker', start_parameter_services=False)
    speaker = Speaker(node=node, debug=True)
