#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2024  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
###############################################################################

import message_filters
from dataclasses import dataclass, field
from typing import Optional
import evaluation_metrics
from pedestrian_plugin_msgs.msg import Agent, Agents, Metric


@dataclass
class EvaluationParameter():
    metrics: Optional[list] = field(default_factory=list)
    robot_radius: Optional[float] = None


class Evaluator:
    def __init__(self, node):
        self.node = node

        # variables
        self.robot_list = []
        self.human_list = []
        self._recording = False
        self._ready = False
        self._logger = None
        self._metrics_to_compute = []
        self._metrics_func_list = []

        # self evaluation parameters
        self._robot_radius = None

        # subscriber and publisher
        self.robot_sub = message_filters.Subscriber(node, Agent, "/robot_states")
        self.human_sub = message_filters.Subscriber(node, Agents, "/human_states")
        self.metric_pub = self.node.create_publisher(Metric, "/metric", 10)
        self.time_synchronizer = message_filters.TimeSynchronizer([self.robot_sub, self.human_sub], 10)
        self.time_synchronizer.registerCallback(self.agents_callback)

        # timer
        self._timer_period_sec = 1.0  # [s]
        self._evaluation_timer = None

    def set_logger(self, logger):
        self._logger = logger

    def set_timer_period(self, timer_period_sec: float):
        self._timer_period_sec = timer_period_sec

    def set_metrics_to_compute(self, metrics_to_compute):
        self._metrics_to_compute = metrics_to_compute
        self._metrics_func_list = [getattr(evaluation_metrics, metric) for metric in metrics_to_compute]

    def set_evaluation_parameters(self, **kwargs):
        eval_params = EvaluationParameter(**kwargs)
        self.set_metrics_to_compute(eval_params.metrics)
        if eval_params.robot_radius is not None:
            self._robot_radius = float(eval_params.robot_radius)
        self._logger.debug(f"evaluation_parameters={eval_params}")

    def start(self):
        self.reset()
        self._recording = True
        if self._evaluation_timer is None:
            self._evaluation_timer = self.node.create_timer(self._timer_period_sec, self.run_evaluation)

    def stop(self):
        self._recording = False
        self.run_evaluation()
        if self._evaluation_timer is not None:
            self._evaluation_timer.cancel()
            self._evaluation_timer.destroy()
            self._evaluation_timer = None

    def get_evaluation_results(self):
        results = []
        for metric, func in zip(self._metrics_to_compute, self._metrics_func_list):
            result = func(self.human_list, self.robot_list)
            results.append({"name": metric, "value": result[0]})
        return results

    def reset(self):
        self.robot_list = []
        self.human_list = []
        self.results = []
        self._ready = False

    def agents_callback(self, robot, human):
        if self._recording:
            # robot
            if self._robot_radius is not None:
                robot.radius = self._robot_radius

            self.robot_list.append(robot)
            # record only active human
            humans_active = [a for a in human.agents if a.behavior_state == Agent.ACTIVE]
            human.agents = humans_active
            self.human_list.append(human)

        if 2 <= len(self.robot_list):
            self._ready = True

    def run_evaluation(self):
        if not self._ready:
            return

        for metric, func in zip(self._metrics_to_compute, self._metrics_func_list):
            result = func(self.human_list, self.robot_list)
            value = result[0]

            if self._logger is not None:
                self._logger.debug(f"{metric}={value}")

            msg = Metric()
            msg.name = metric
            msg.value = float(value)
            self.metric_pub.publish(msg)
