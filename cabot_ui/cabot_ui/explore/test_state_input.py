import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8



class StateInput:
    # accept user keyboard input and publish the state to /cabot/nav_state_input
    def __init__(self, node):
        self.node = node
        self.state_idx = 1
        self.state_str = "paused"
        self.idx_to_state = {0: "running", 1: "paused", 2: "finished"}
        self.abbrv_to_state = {'r': 'running', 'p': 'paused', 'f': 'finished'}
        self.state_to_idx = {k: v for v, k in self.idx_to_state.items()}
        
        self.state_pub = self.node.create_publisher(String, '/cabot/nav_state_input', 10)
    
    def run(self):
        self.node.get_logger().info("State input started")
        self.get_input()

    def get_input(self):
        while True:
            user_input = input("Enter state ([r]unning, [p]aused, [f]inished): ")
            if user_input in self.abbrv_to_state.keys():
                self.state_str = self.abbrv_to_state[user_input]
                self.state_idx = self.state_to_idx[self.state_str]
                self.publish_state()
            else:
                self.node.get_logger().info(f"Invalid state: {user_input}")

    def publish_state(self):
        msg = String()
        msg.data = self.state_str
        self.state_pub.publish(msg)
        self.node.get_logger().info(f"State published: {msg.data}")
    
    def set_state(self, state_str):
        self.state_str = state_str
        self.state_idx = self.state_to_idx[self.state_str]
        self.publish_state()


if __name__ == '__main__':
    rclpy.init()
    node = Node('state_input', start_parameter_services=False)
    state_control = StateInput(node=node)
    state_control.run()
    node.destroy_node()
    rclpy.shutdown()