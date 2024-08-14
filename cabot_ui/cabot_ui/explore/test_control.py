import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


"""
0. put this script in cabot/cabot-navigation/docker/home/ros2_ws/src

1. launch simulator
```
$ cd cabot/cabot-navigation
$ docker compose up gazebo gui navigation
```

2. (Optional) launch mapping script
```
$ cd cabot
$ ./mapping-launch.sh -c -s
```

3. Run this script
```
$ cd cabot/cabot-navigation
$ docker compose exec navigation bash
--- in container ---
$ source install/setup.bash
$ cd /home/developer/ros2_ws/src
$ python3 test_control.py
```
"""


class RCLPublisher(Node):
    def __init__(self, timer_period=0.5):
        super().__init__("cmd_vel_publisher")
        self.timer_period = timer_period
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.7
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")
        self.i += 1


def main():
    # set log level to debug
    # rclpy.logging._root_logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

    rclpy.init()
    rcl_publisher = RCLPublisher()
    rclpy.spin(rcl_publisher)
    rcl_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()