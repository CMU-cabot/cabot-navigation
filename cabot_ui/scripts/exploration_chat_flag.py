#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class PersistentCancelClient(Node):
    def __init__(self):
        super().__init__('persistent_cancel_client')
        self.cli = self.create_client(Trigger, 'trigger_navigation_cancel')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run(self):
        self.get_logger().info("PersistentCancelClient started and running")
        while rclpy.ok():
            response = self.send_request()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response.message}")
            else:
                self.get_logger().warn(f"Service call failed: {response.message}")

            time.sleep(10)

def main(args=None):
    rclpy.init(args=args)

    client = PersistentCancelClient()
    client.run()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
