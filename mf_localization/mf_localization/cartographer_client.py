#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025  IBM Corporation
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

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf_transformations

from geometry_msgs.msg import Pose

from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import GetTrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory
from cartographer_ros_msgs.srv import StartTrajectory
from cartographer_ros_msgs.srv import ReadMetrics
from cartographer_ros_msgs.srv import TrajectoryQuery

from mf_localization.rclpy_utils import call_service


def compute_relative_pose(pose1: Pose, pose2: Pose) -> Pose:
    relative_position = [
        pose2.position.x - pose1.position.x,
        pose2.position.y - pose1.position.y,
        pose2.position.z - pose1.position.z,
        1  # for homogeneous transformation
    ]
    q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
    q2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
    q1_inv = tf_transformations.quaternion_inverse(q1)
    R = tf_transformations.quaternion_matrix(q1_inv)  # homogeneous rotation matrix
    rotated_relative_position = R.dot(relative_position)
    relative_orientation = tf_transformations.quaternion_multiply(q1_inv, q2)

    relative_pose = Pose()
    relative_pose.position.x = rotated_relative_position[0]
    relative_pose.position.y = rotated_relative_position[1]
    relative_pose.position.z = rotated_relative_position[2]
    relative_pose.orientation.x = relative_orientation[0]
    relative_pose.orientation.y = relative_orientation[1]
    relative_pose.orientation.z = relative_orientation[2]
    relative_pose.orientation.w = relative_orientation[3]

    return relative_pose


class CartographerClient:

    def __init__(self,
                 node,
                 logger,
                 node_id,
                 mode,
                 configuration_directory,
                 configuration_basename,
                 min_hist_count=1,
                 callback_group=MutuallyExclusiveCallbackGroup(),
                 ):
        # callback group accessing the same ros node
        self.node_service_callback_group = callback_group
        # rclpy services
        self._get_trajectory_states = node.create_client(GetTrajectoryStates, node_id+"/"+str(mode)+'/get_trajectory_states', callback_group=self.node_service_callback_group)
        self._finish_trajectory = node.create_client(FinishTrajectory, node_id+"/"+str(mode)+'/finish_trajectory', callback_group=self.node_service_callback_group)
        self._start_trajectory = node.create_client(StartTrajectory, node_id+"/"+str(mode)+'/start_trajectory', callback_group=self.node_service_callback_group)
        self._read_metrics = node.create_client(ReadMetrics, node_id+"/"+str(mode)+'/read_metrics', callback_group=self.node_service_callback_group)
        self._trajectory_query = node.create_client(TrajectoryQuery, node_id+"/"+str(mode)+'/trajectory_query', callback_group=self.node_service_callback_group)

        self.logger = logger
        self.configuration_directory = configuration_directory
        self.configuration_basename = configuration_basename

        # constant
        self.relative_to_trajectory_id = 0

        # parameters
        self.min_hist_count = min_hist_count

        # states
        self.constraints_count = 0

        # memory
        self.trajectory_initial_pose = None

    def reset_states(self):
        self.constraints_count = 0

    def start_trajectory(self, pose: Pose, timeout_sec, max_retries):
        self.logger.info(F"wait for {self._start_trajectory.srv_name} service")
        self._start_trajectory.wait_for_service()

        configuration_directory = self.configuration_directory
        configuration_basename = self.configuration_basename
        use_initial_pose = True
        relative_to_trajectory_id = 0

        # compute relative pose to trajectory initial pose
        if self.trajectory_initial_pose is None:
            # trajectory query (for the first time)
            self.trajectory_initial_pose = self.get_trajectory_initial_pose(timeout_sec=timeout_sec)

        relative_pose: Pose = compute_relative_pose(self.trajectory_initial_pose, pose)
        self.logger.info(F"converted initial_pose ({pose}) to relative_pose ({relative_pose}) on trajectory {relative_to_trajectory_id}")

        self.logger.info("prepare request")
        req = StartTrajectory.Request(
            configuration_directory=configuration_directory,
            configuration_basename=configuration_basename,
            use_initial_pose=use_initial_pose,
            initial_pose=relative_pose,
            relative_to_trajectory_id=relative_to_trajectory_id)
        try:
            res2: StartTrajectory.Response = call_service(self._start_trajectory,
                                                          req,
                                                          timeout_sec=timeout_sec,
                                                          max_retries=max_retries,
                                                          logger=self.logger,
                                                          )
        except (TimeoutError, Exception) as e:
            self.logger.error(F"Failed to call start_trajectory. error={type(e).__name__}({e})")
            raise e
        self.logger.info(F"start_trajectory response = {res2}")
        status_code = res2.status.code

        return status_code

    def get_trajectory_initial_pose(self, timeout_sec) -> Pose:

        trajectory_query = self._trajectory_query
        self.logger.info(F"wait for {trajectory_query.srv_name} service")
        trajectory_query.wait_for_service()
        req = TrajectoryQuery.Request(
            trajectory_id=self.relative_to_trajectory_id
        )
        try:
            res: TrajectoryQuery.Response = call_service(trajectory_query, req, timeout_sec=timeout_sec)
        except (TimeoutError, Exception) as e:
            self.logger.error(F"Failed to call trajectory_query. error={type(e).__name__}({e})")
            raise e
        trajectory = res.trajectory
        trajectory_initial_pose: Pose = trajectory[0].pose  # PoseSamped -> Pose

        self.logger.info(F"trajectory {self.relative_to_trajectory_id} initial pose = {trajectory_initial_pose}")
        return trajectory_initial_pose

    def finish_trajectory(self, timeout_sec, max_retries):
        # wait for services
        self.logger.info(F"wait for {self._get_trajectory_states.srv_name} service")
        self._get_trajectory_states.wait_for_service()
        self.logger.info(F"wait for {self._finish_trajectory.srv_name} service")
        self._finish_trajectory.wait_for_service()

        req = GetTrajectoryStates.Request()
        try:
            res0: GetTrajectoryStates.Response = call_service(self._get_trajectory_states,
                                                              req,
                                                              timeout_sec=timeout_sec
                                                              )
        except (TimeoutError, Exception) as e:
            self.logger.error(F"Failed to call get_trajectory_states. error={type(e).__name__}({e})")
            raise e
        self.logger.info(F"{res0}")
        last_trajectory_id = res0.trajectory_states.trajectory_id[-1]
        last_trajectory_state = res0.trajectory_states.trajectory_state[-1]  # uint8 -> int

        # finish trajectory only if the trajectory is active.
        if last_trajectory_state in [TrajectoryStates.ACTIVE]:
            trajectory_id_to_finish = last_trajectory_id
            req = FinishTrajectory.Request(trajectory_id=trajectory_id_to_finish)
            try:
                res1: FinishTrajectory.Response = call_service(self._finish_trajectory, req,
                                                               timeout_sec=timeout_sec,
                                                               max_retries=max_retries,
                                                               logger=self.logger,
                                                               )
            except (TimeoutError, Exception) as e:
                self.logger.error(F"Failed to call finish_trajectory. error={type(e).__name__}({e})")
                raise e
            self.logger.info(F"{res1}")

        status_code = res1.status.code
        return status_code

    def read_metrics(self, timeout_sec):
        self.logger.info(F"wait for {self._read_metrics.srv_name} service")
        self._read_metrics.wait_for_service()

        req = ReadMetrics.Request()
        self.logger.info("request read_metrics")
        try:
            res: ReadMetrics.Response = call_service(self._read_metrics, req, timeout_sec=timeout_sec)
        except (TimeoutError, Exception) as e:
            self.logger.error(F"failed to call read_metrics. error={type(e).__name__}({e})")
            return False

        if res.status.code != 0:  # OK
            self.logger.info(f"read_metrics fails {res.status}")
            return False

        return res

    def is_optimized(self, timeout_sec):
        res = self.read_metrics(timeout_sec)
        if res is False:
            return False

        # comment out the following two lines to debug
        # from rosidl_runtime_py import message_to_yaml
        # self.logger.info(message_to_yaml(res))

        # monitor mapping_2d_pose_graph_constraints -> inter_submap -> different trajectory to detect inter trajectory pose graph optimization
        # because this value is updated after running optimization
        optimized = False
        for metric_family in res.metric_families:
            if metric_family.name == "mapping_2d_pose_graph_constraints":
                for metric in metric_family.metrics:
                    # inter_submap -> different trajectory
                    if metric.labels[0].key == "tag" and metric.labels[0].value == "inter_submap" \
                            and metric.labels[1].key == "trajectory" and metric.labels[1].value == "different":
                        self.logger.info(f"inter_submap different trajectory constraints. count={metric.value}")
                        # check if the number of constraints changed
                        if self.constraints_count != metric.value:
                            self.constraints_count = metric.value
                            if metric.value >= self.min_hist_count:
                                optimized = True
                                self.logger.info("pose graph optimization detected.")
        return optimized
