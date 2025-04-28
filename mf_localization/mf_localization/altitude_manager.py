#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2021  IBM Corporation
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

from dataclasses import dataclass
import bisect
import math
import random
import numpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from std_msgs.msg import Float64


class AltitudeManager():
    def __init__(self, node, verbose=False):
        self.node = node
        self.logger = node.get_logger()
        self.queue = []
        self.verbose = verbose
        self.queue_limit = 60
        self.timestamp_interval_limit = 3.0
        self.window_size = 20
        self.threshold = 0.3 * 12

        self.initial_pressure = None
        self.pressure_std_pub = self.node.create_publisher(
            Float64, "pressure_std", QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def put_pressure(self, pressure):
        if not self.initial_pressure:
            self.initial_pressure = pressure

        if self.queue_limit <= len(self.queue):
            self.queue.pop(0)

        if len(self.queue) == 0:
            self.queue.append(pressure)
            return

        if Time.from_msg(self.queue[-1].header.stamp) - Time.from_msg(pressure.header.stamp) > Duration(seconds=self.timestamp_interval_limit):
            if self.verbose:
                self.logger.error(F"timestamp interval between two altimters is too large ({self.timestamp_interval_limit} sec). "
                                  "AltitudeManager was reset.")
            self.queue.clear()

        self.queue.append(pressure)

    def is_height_changed(self):
        if len(self.queue) < self.window_size:
            return False

        relative = []
        for i in range(1, self.window_size+1):
            relative.append(
                self.queue[-i].fluid_pressure - self.initial_pressure.fluid_pressure)

        stdev = numpy.std(relative)

        msg = Float64()
        msg.data = stdev
        self.pressure_std_pub.publish(msg)

        if self.verbose:
            self.logger.info("Altimeter changed: {}".format(stdev))

        if self.threshold < stdev:
            return True

        return False


@dataclass
class AltitudeFloorEstimatorParameters:
    enable: bool = True
    height_per_floor: float = 4.0  # [m]
    floor_threshold_height_coeff: float = 0.5
    height_diff_threshold_coeff: float = 0.2
    current_window: int = 5
    history_window: int = 10
    p0: float = 101325.0  # [Pa]
    c0: float = 44300.0
    c1: float = 5.255


@dataclass
class AltitudeFloorEstimatorResult:
    ref_height: float
    floor_est: int
    height_est: float
    floor_vel_est: float
    height_vel_est: float
    current_state: int
    floor_change_event: int


class AltitudeFloorEstimator:
    def __init__(self, parameters: AltitudeFloorEstimatorParameters = AltitudeFloorEstimatorParameters()):
        self._enabled = parameters.enable
        # set constants
        self._p0 = parameters.p0
        self._c0 = parameters.c0
        self._c1 = parameters.c1
        # set parameters
        self._CW = parameters.current_window
        self._HW = parameters.history_window
        self.height_per_floor = parameters.height_per_floor
        self.floor_threshold_height_coeff = parameters.floor_threshold_height_coeff
        self.height_diff_threshold_coeff = parameters.height_diff_threshold_coeff

        # calculate parameters
        self.floor_threshold_height = parameters.floor_threshold_height_coeff * self.height_per_floor
        self.height_diff_threshold = parameters.height_diff_threshold_coeff * self.height_per_floor

        # initialize variables
        self.reset()

    def reset(self, floor_est=0, height_est=0.0):
        self.time_queue = []
        self.height_queue = []

        self.height_est_memory = height_est

        self.floor_est_memory = floor_est
        self.last_floor_est = floor_est

        self.last_state = 0  # no motion

        self.floor_array = None
        self.height_array = None

        return

    def set_floor_height_list(self, floor_list, height_list):
        self.floor_array = numpy.array(floor_list)
        self.height_array = numpy.array(height_list)

    def use_floor_height_list(self):
        if self.floor_array is None or self.height_array is None:
            return False
        if len(self.floor_array) == 0 or len(self.height_array) == 0:
            return False
        if numpy.isnan(self.floor_array).any() or numpy.isnan(self.height_array).any():
            return False
        if self.floor_est_memory not in self.floor_array:
            return False
        return True

    def calculate_height_difference(self, height_array):
        height_difference_array = numpy.zeros(height_array.shape)
        for i in range(len(height_array)):
            if height_array[i] < 0:
                height_difference_array[i] = height_array[i] - height_array[i+1]
            elif height_array[i] > 0:
                height_difference_array[i] = height_array[i] - height_array[i-1]
            else:  # ==0
                height_difference_array[i] = 0.0
        return height_difference_array

    def put_pressure(self, pressure_msg) -> AltitudeFloorEstimatorResult:
        # convert pressure to reference height
        pressure = pressure_msg.fluid_pressure
        timestamp = pressure_msg.header.stamp.sec + pressure_msg.header.stamp.nanosec * 1e-9  # [seconds]
        return self._put_pressure(timestamp, pressure)

    def _put_pressure(self, timestamp, pressure) -> AltitudeFloorEstimatorResult:
        ref_height = self._c0 * (1.0 - (pressure / self._p0)**(1.0 / self._c1))

        # temporal estimated values
        height_est = self.height_est_memory
        floor_est = self.floor_est_memory
        height_vel_est = 0.0
        floor_vel_est = 0.0
        current_state = self.last_state
        floor_change_event = None

        # prepare height windows
        self.time_queue.append(timestamp)
        self.height_queue.append(ref_height)
        if len(self.height_queue) > self._CW + self._HW:
            self.time_queue.pop(0)
            self.height_queue.pop(0)
        else:  # return if not ready
            return AltitudeFloorEstimatorResult(ref_height, floor_est, height_est, floor_vel_est, height_vel_est, current_state, floor_change_event)

        time_CW_window = self.time_queue[self._HW:]
        height_CW_window = self.height_queue[self._HW:]
        height_HW_window = self.height_queue[:-self._CW]

        height_CW = numpy.average(height_CW_window)
        height_HW = numpy.average(height_HW_window)

        diff_CW_HW = height_CW - height_HW
        if diff_CW_HW > self.height_diff_threshold:
            current_state = 1  # up
        elif diff_CW_HW < -self.height_diff_threshold:
            current_state = -1  # down
        else:
            current_state = 0  # stable

        # velocity estimation
        time_CW_window = numpy.array(time_CW_window)
        height_CW_window = numpy.array(height_CW_window)
        A = numpy.vstack([time_CW_window - time_CW_window[0], numpy.ones(len(time_CW_window))]).T  # subtract t[0] for stable computation
        height_vel_est, _ = numpy.linalg.lstsq(A, height_CW_window, rcond=None)[0]
        floor_vel_est = height_vel_est / self.height_per_floor

        # no motion
        if self.last_state == 0:
            if current_state != 0:
                self.last_stable_height = height_HW
        # up / down
        elif self.last_state == 1 or self.last_state == -1:
            height_est = self.height_est_memory + (ref_height - self.last_stable_height)

            # estimate floor change by thresholding
            if self.use_floor_height_list():
                height_diff = ref_height - self.last_stable_height
                index = self.floor_array == floor_est
                relative_height_array = self.height_array - self.height_array[index]
                height_difference_array = self.calculate_height_difference(relative_height_array)
                threshold_height_array = relative_height_array - height_difference_array * self.floor_threshold_height_coeff
                # update floor only when floor change state and height_diff match
                idx_new_floor = index
                if self.last_state == 1 and height_diff > 0:  # up
                    idx_new_floor = bisect.bisect_right(threshold_height_array, height_diff) - 1
                elif self.last_state == -1 and height_diff < 0:  # down
                    idx_new_floor = bisect.bisect_left(threshold_height_array, height_diff)
                floor_est = self.floor_array[idx_new_floor]
            else:
                floor_diff_float, floor_diff_int = math.modf((ref_height - self.last_stable_height) / self.height_per_floor)
                height_diff_float = floor_diff_float * self.height_per_floor
                floor_diff_final = floor_diff_int + numpy.sign(height_diff_float) * (self.floor_threshold_height < numpy.abs(height_diff_float))
                floor_est = self.floor_est_memory + floor_diff_final

            # emit floor up/down change event
            if self.last_floor_est != floor_est:
                floor_change_event = self.last_state

            # end of moving up/down
            if current_state == 0:
                self.height_est_memory = height_est
                self.floor_est_memory = floor_est

                # up/down -> stable event
                floor_change_event = 0

        self.last_state = current_state
        self.last_floor_est = floor_est

        return AltitudeFloorEstimatorResult(ref_height, floor_est, height_est, floor_vel_est, height_vel_est, current_state, floor_change_event)

    def is_height_changed(self):
        if self.last_state == 0:  # no motion
            return False
        else:
            return True

    def enabled(self):
        return self._enabled


class BalancedSampler:
    def __init__(self):
        self._x: list = None
        self._p: list = None
        self._updated = False
        self._reserved_init = []
        self._reserved = []

    def update(self, x, p):
        _x = list(x)
        _p = list(p)
        if self._x == _x and self._p == _p:
            self._updated = False
        else:
            self._x = _x
            self._p = _p
            self._updated = True

        if self._updated:
            min_p = numpy.min(self._p)
            min_N = int(numpy.ceil(1.0/min_p))
            counts = min_N * numpy.array(self._p)
            reserved = []
            for i in range(len(self._x)):
                for j in range(int(counts[i])):
                    reserved.append(self._x[i])
            self._reserved_init = reserved
            self._reserved = []

    def sample(self):
        if len(self._reserved) == 0:
            self._reserved = self._reserved_init.copy()

        selected = random.choice(self._reserved)
        self._reserved.remove(selected)
        return selected

    def select(self, x):
        if len(self._reserved) == 0:
            self._reserved = self._reserved_init.copy()

        if x not in self._reserved:
            if x not in self._reserved_init:
                raise RuntimeError(f"x = {x} is not in reserved_init {self._reserved_init}")
            else:
                return None
        else:
            self._reserved.remove(x)
            return x

    def selectable(self, x):
        if len(self._reserved) == 0:
            self._reserved = self._reserved_init.copy()

        return x in self._reserved


class FloorHeightMapper:
    def __init__(self, X, radius=5.0, logger=None):
        # X [x, y, floor, area, height]
        _X = numpy.array(X)
        self._floors = numpy.unique(_X[:, 2])

        self.X_dict = {}
        self.nbrs_dict = {}
        self._radius = radius
        self._logger = logger

        from sklearn.neighbors import NearestNeighbors
        for _floor in self._floors:
            _X_floor = _X[_X[:, 2] == _floor]
            self.X_dict[_floor] = _X_floor
            self.nbrs_dict[_floor] = NearestNeighbors(n_neighbors=1).fit(_X_floor[:, :2])

    def get_floor_list(self, x, radius=None):
        floor_list, _ = self.get_floor_height_list(x, radius)
        return floor_list

    def get_height_list(self, x, radius=None):
        _, height_list = self.get_floor_height_list(x, radius)
        return height_list

    def get_floor_height_list(self, x, radius=None):
        _x = numpy.array(x).reshape(1, -1)
        radius_global = self._radius if radius is None else radius

        floor_list = []
        height_list = []

        # search the closest sample for each floor
        for _floor in self._floors:
            nbrs = self.nbrs_dict[_floor]
            distances, indices = nbrs.kneighbors(_x)
            dist = distances[0]
            idx = indices[0]

            # update radius for sample
            radius_sample = self.X_dict[_floor][idx, 5].item()
            if not numpy.isnan(radius_sample):
                _radius = radius_sample
            else:
                _radius = radius_global

            if dist < _radius:
                floor_list.append(_floor)
                height = self.X_dict[_floor][idx, 4].item()
                height_list.append(height)
        return floor_list, height_list


def main():
    import argparse
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser("Sample program to run altitude floor estimator")
    parser.add_argument("-i", "--input_bag", required=True)
    parser.add_argument("-p", "--pressure_topic", default="/pressure")

    args = parser.parse_args()

    input_bag = args.input_bag
    pressure_topic = args.pressure_topic

    altitude_floor_estimator = AltitudeFloorEstimator()
    altitude_floor_estimator.reset(floor_est=0)

    X = []
    with AnyReader([Path(input_bag)]) as reader:
        connections = [x for x in reader.connections if x.topic == pressure_topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            t_sec = timestamp * 1e-9
            msg = reader.deserialize(rawdata, connection.msgtype)
            fluid_pressure = msg.fluid_pressure
            result = altitude_floor_estimator.put_pressure(msg)
            X.append([t_sec, fluid_pressure, result.ref_height, result.floor_est, result.height_est, result.floor_vel_est, result.height_vel_est, result.current_state])
    X = numpy.array(X)

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ax1.plot(X[:, 0], X[:, 2], marker="o", label="height", color="red")
    ax2.plot(X[:, 0], X[:, 3], marker="x", label="floor", color="blue")
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    plt.legend(lines1 + lines2, labels1 + labels2)
    plt.show()


if __name__ == "__main__":
    main()
