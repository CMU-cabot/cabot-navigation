#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025 IBM Corporation
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

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from gazebo_msgs.msg import ModelStates


class PressureSimulator():

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        node.declare_parameter('verbose', False)
        self._verbose = node.get_parameter('verbose').value

        node.declare_parameter('config_file', '')
        config_file = node.get_parameter('config_file').value
        config = None
        with open(config_file) as stream:
            config = yaml.safe_load(stream)

        node.declare_parameter('frequency', 2.0)  # Hz
        self._frequency = node.get_parameter('frequency').value

        model = config.get('model')
        robot = config.get('robot')
        self._model_name = model.get('name') if model else None
        self._robot_name = robot.get('name') if robot else None

        self._pressure_pub = node.create_publisher(FluidPressure, 'pressure', 10)
        self._model_sub = node.create_subscription(ModelStates, "/gazebo/model_states", self.model_callback, 10)

        # parameter
        self.p0: float = 101325.0  # [Pa]
        self.c0: float = 44300.0
        self.c1: float = 5.255

        self._timer = node.create_timer(1.0/self._frequency, self.publish_pressure)

        self._model_message = None
        self._z = None

    def publish_pressure(self):
        if self._model_message is None:
            return

        # get robot pose from the model_states message
        x_r2m, y_r2m, z_r2m = self.get_robot_pose(self._model_message)

        if x_r2m is None or y_r2m is None or z_r2m is None:
            return

        if self._verbose:
            self._logger.info(F"x_r2m={x_r2m}, y_r2m={y_r2m}, z_r2m={z_r2m}")

        self._z = z_r2m
        now = self._node.get_clock().now()
        msg = self.simulate_message(now, self._z)

        # publish message
        self._pressure_pub.publish(msg)

    def simulate_message(self, timestamp, z):
        p = self.p0 * (1.0 - z/self.c0)**self.c1
        msg = FluidPressure()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = "base_link"
        msg.fluid_pressure = p
        return msg

    def get_robot_pose(self, message):
        # get the position of the robot relative to the model
        mesh_name = self._model_name
        robot_name = self._robot_name

        if self._verbose:
            self._logger.info(F"mesh_name={mesh_name}, robot_name={robot_name}")

        names = message.name
        poses = message.pose

        pose_dict = {}
        for i, name in enumerate(names):
            pose_dict[name] = poses[i]
            if self._verbose:
                self._logger.info(F"{name} - {poses[i].position}")

        if mesh_name not in pose_dict:
            return None, None, None
        if robot_name not in pose_dict:
            return None, None, None

        x_mesh = pose_dict[mesh_name].position.x
        y_mesh = pose_dict[mesh_name].position.y
        z_mesh = pose_dict[mesh_name].position.z

        x_robot = pose_dict[robot_name].position.x
        y_robot = pose_dict[robot_name].position.y
        z_robot = pose_dict[robot_name].position.z

        x_r2m = x_robot - x_mesh
        y_r2m = y_robot - y_mesh
        z_r2m = z_robot - z_mesh

        return x_r2m, y_r2m, z_r2m

    def model_callback(self, message):
        self._model_message = message


def main():
    rclpy.init()
    node = rclpy.node.Node('pressure_simulator')
    _ = PressureSimulator(node)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
