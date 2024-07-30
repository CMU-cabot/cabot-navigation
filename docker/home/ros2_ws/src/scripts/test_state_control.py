import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Header

class StateControl:
    # publish the state and update the state every second
    # state is: "running", "paused", "finished"
    def __init__(self, node):
        self.node = node
        self.state_idx = 1
        self.state_str = "paused"
        self.idx_to_state = {0: "running", 1: "paused", 2: "finished"}
        self.state_to_idx = {k: v for v, k in self.idx_to_state.items()}
        
        self.state_pub = self.node.create_publisher(String, '/cabot/nav_state', 10)
        self.state_sub = self.node.create_subscription(String, '/cabot/nav_state_input', self.state_callback, 10)
        self.timer = self.node.create_timer(1.0, self.timer_callback)
    
    def state_callback(self, msg):
        self.state_str = msg.data
        self.state_idx = self.state_to_idx[self.state_str]
        # self.node.get_logger().info(f"State received: {self.state_str}")
    
    def timer_callback(self):
        # self.state_update()
        self.publish_state()
    
    def run(self):
        # self.node.get_logger().info("State control started")
        rclpy.spin(self.node)
    
    def state_update(self):
        self.state_idx = (self.state_idx + 1) % 3
        self.state_str = self.idx_to_state[self.state_idx]
    
    def publish_state(self):
        msg = String()
        msg.data = self.state_str
        self.state_pub.publish(msg)
        # self.node.get_logger().info(f"State published: {msg.data}")



if __name__ == '__main__':
    rclpy.init()
    node = Node('state_control', start_parameter_services=False)
    state_control = StateControl(node=node)
    state_control.run()
    node.destroy_node()
    rclpy.shutdown()