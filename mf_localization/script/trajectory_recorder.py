#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2024  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import csv

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
import traceback
from std_srvs.srv import Trigger
from cartographer_ros_msgs.srv import TrajectoryQuery


class TrajectoryRecorder(Node):
    def __init__(self, node, output, timer_period=10.0):
        self._node = node
        self._output = output
        self._timer_period = timer_period

        self._logger = self._node.get_logger()

        self._service = self._node.create_service(Trigger, '~/call_trajectory_query', self.call_trajectory_query_callback)
        self._save_trajectory = self._node.create_service(Trigger, '~/save_trajectory', self.save_trajectory_callback)

        # cartographer_node /trajectory_query service
        self._client = self._node.create_client(TrajectoryQuery, '/trajectory_query')

        self._timer = self._node.create_timer(self._timer_period, self.timer_callback)

        self._trajectory = None

    def call_trajectory_query_callback(self, request: Trigger.Request, response: Trigger.Response):
        self._logger.info("TrajectoryRecorder.call_trajectory_query_callback called")
        success, message = self.call_trajectory_query()
        response.success = success
        response.message = message
        return response

    def timer_callback(self):
        self._logger.info("TrajectoryRecorder.timer_callback called")
        self.call_trajectory_query()

    def call_trajectory_query(self) -> bool | str:
        success = False
        message = ""

        if not self._client.service_is_ready():
            message = f"service is not ready. service={self._client.srv_name}"
            self._logger.info(message)
            success = False
            return success, message

        trajectory_query_req = TrajectoryQuery.Request()
        trajectory_query_req.trajectory_id = 0  # first trajectory

        self._future = self._client.call_async(trajectory_query_req)

        def done_callback(future):
            self._logger.info("TrajectoryRecorder.call_trajectory_query.done_callback called.")
            response: TrajectoryQuery.Response = future.result()
            status = response.status
            self._trajectory = response.trajectory
            self._logger.info(f"status={status}")

        self._future.add_done_callback(done_callback)
        success = True
        message = f"called {self._client.srv_name} service"
        return success, message

    def save_trajectory(self) -> bool | str:
        success = False
        message = ""
        if (self._output is not None and self._output != "") and self._trajectory is not None:
            self._logger.info("writing to output")
            with open(self._output, "w") as f:
                writer = csv.writer(f, lineterminator="\n")
                writer.writerow(["timestamp", "frame_id", "x", "y", "z", "qx", "qy", "qz", "qw"])
                for pose_stamped in self._trajectory:
                    header = pose_stamped.header
                    pose = pose_stamped.pose
                    timestamp = header.stamp.sec + 1.0e-9 * header.stamp.nanosec
                    frame_id = header.frame_id
                    x = pose.position.x
                    y = pose.position.y
                    z = pose.position.z
                    qx = pose.orientation.x
                    qy = pose.orientation.y
                    qz = pose.orientation.z
                    qw = pose.orientation.w
                    writer.writerow([timestamp, frame_id, x, y, z, qx, qy, qz, qw])
            success = True
            message = f"trajectory was written to output file (ouput={self._output})"
        else:
            success = False
            message = f"trajectory was not written to output file (ouput={self._output}, trajectory={self._trajectory})"
            self._logger.info(message)
        return success, message

    def save_trajectory_callback(self, request: Trigger.Request, response: Trigger.Response):
        self._logger.info("TrajectoryRecorder.save_trajectory_callback called")
        success, message = self.save_trajectory()
        response.success = success
        response.message = message
        return response

    def get_trajectory(self):
        return self._trajectory

    def on_shutdown(self):
        self._logger.info("TrajectoryRecorder.on_shutdown")
        self.save_trajectory()


def main():
    rclpy.init()
    node = Node("trajectory_recorder")
    output = node.declare_parameter("output", "").value
    timer_period = node.declare_parameter("timer_period", 10.0).value

    trajectory_recorder = TrajectoryRecorder(node, output, timer_period)

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        traceback.print_exc()
        trajectory_recorder.on_shutdown()


if __name__ == "__main__":
    main()
