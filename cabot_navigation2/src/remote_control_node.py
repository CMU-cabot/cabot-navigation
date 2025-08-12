import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RemoteControlNode(Node):
    def __init__(self):
        super().__init__('remote_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Send commands every 0.5 seconds
        self.timer = self.create_timer(0.5, self.send_command)
        self.get_logger().info('Remote control node started.')

    def send_command(self):
        msg = Twist()
        # Set your desired velocities
        msg.linear.x = 0.2   # move forward at 0.2 m/s
        msg.angular.z = 0.0  # no rotation

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = RemoteControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
