#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
from people_msgs.msg import People

class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to RViz clicked point
        self.create_subscription(PointStamped, '/clicked_point', self.point_callback, 10)

        # Subscribe to people messages
        self.create_subscription(People, '/people', self.people_callback, 10)
    
    def people_callback(self, msg: People):
        self.get_logger().info(
            f'Detected people: {msg}'
        )

    def point_callback(self, msg: PointStamped):
        self.get_logger().info(
            f'Received point: x={msg.point.x:.2f}, y={msg.point.y:.2f}, z={msg.point.z:.2f}'
        )

        goal_pose = self.create_pose_from_point(msg)
        self.send_goal(goal_pose)

    def send_goal(self, pose: PoseStamped):
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.current_pose.pose.position}')


    def create_pose_from_point(self, msg: PointStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = msg.point

        # Default orientation: facing forward (yaw = 0)
        q = quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
