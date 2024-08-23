import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class CancelNode:
    def __init__(self, node):
        self.node = node
        self.cancel_pub = self.node.create_publisher(String, "/cabot/event", 10)
        self.event_sub = self.node.create_subscription(String, "/cabot/event", self.event_callback, 10)
        self.logger = self.node.get_logger()
        self.logger.info("Cancel node initialized")
        self.state = ""
        self.cancel_state_published = False

        self.timer = self.node.create_timer(1.0, self.publish_state)

        self.srv = self.node.create_service(Trigger, 'trigger_navigation_cancel', self.handle_service_request)
    
    def event_callback(self, msg):
        if msg.data == "navigation_startchat" or msg.data == "navigation_button_control":
            if self.state != "navigation;cancel":
                self.logger.info("Received startchat event")
                self.state = "navigation;cancel"
            else:
                self.state = "auto_mode"
        # elif msg.data == "navigation;event;navigation_start":
        #     self.logger.info(f"Received navigation start event: {msg.data}")
        #     self.state = msg.data
        #     self.cancel_state_published = False
        else:
            self.logger.info(f"Received event: {msg.data}")
        self.logger.info(f"Current state: {self.state}")

    def handle_service_request(self, request, response):
        self.logger.info("Service request received to get current state")
        response.success = True
        if self.state == "navigation;cancel":
            response.message = "cancelled_state"
        else:
            response.message = "running_state"
        self.logger.info(f"Service response: {response.message}")
        return response
    
    def run(self):
        self.logger.info("Cancel node started")
        rclpy.spin(self.node)
    
    def publish_state(self):
        if self.state == "navigation;cancel" and not self.cancel_state_published:
            msg = String()
            msg.data = "navigation;cancel"
            self.cancel_pub.publish(msg)
            self.logger.info(f"State published: {msg.data}")
            self.cancel_state_published = True
        else:
            pass


def main():
    if not rclpy.ok():
        rclpy.init()

    node = Node("cancel_node")
    cancel_node = CancelNode(node=node)
    try:
        cancel_node.run()
    except SystemExit as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
