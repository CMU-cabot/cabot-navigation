#!/usr/bin/env python
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

import json
import traceback

import rclpy
import rclpy.time
import rclpy.duration
import tf2_ros
from std_msgs.msg import String

from trajectory_based_interpolator import TrajectoryBasedInterpolator
from trajectory_recorder import TrajectoryRecorder


class BeaconMapper:
    def __init__(self, node, save_empty_beacon_sample, data_inverval=1.2, position_interval=0.5, verbose=False):
        # parameters
        self.node = node
        self.logger = node.get_logger()
        self.clock = node.get_clock()
        self._save_empty_beacon_sample = save_empty_beacon_sample
        self._data_interval = rclpy.duration.Duration(seconds=data_inverval)
        self._position_interval = position_interval
        self._verbose = verbose

        # variables
        self._current_position = None
        self._fingerprints = []
        self._count = 0
        self._previous_fingerprint_time = None
        self._previous_fingerprint_position = None

    def beacons_callback(self, message):
        beacons_obj = json.loads(message.data)
        # print(beacons_obj)
        if self._current_position is not None:
            # print(self._current_position)
            t = self._current_position
            fp_data = {
                "information": {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "z": t.transform.translation.z,
                    "tags": [
                        beacons_obj["phoneID"]
                    ],
                    "rotation": {
                        "x": t.transform.rotation.x,
                        "y": t.transform.rotation.y,
                        "z": t.transform.rotation.z,
                        "w": t.transform.rotation.w
                    }
                },
                "data": {
                    "timestamp": beacons_obj["timestamp"],
                    "beacons": beacons_obj["data"]
                }
            }
            self._fingerprints.append(fp_data)
            self._count += 1
            self._previous_fingerprint_time = self.clock.now()
            self._previous_fingerprint_position = t
            self.logger.info(F"sampling data count = {self._count}")

    def set_current_position(self, position):
        self._current_position = position

        # create dummy fingerprint data when beacons_callback is not called.
        if not self._save_empty_beacon_sample:
            return

        if self._current_position is not None:
            timestamp = self.clock.now()
            t = self._current_position

            # check time interval
            if self._previous_fingerprint_time is not None:
                if not self._previous_fingerprint_time + self._data_interval < timestamp:
                    return
            else:
                self._previous_fingerprint_time = timestamp
                return

            # check position interval
            if self._previous_fingerprint_position is not None:
                tp = self._previous_fingerprint_position
                d2d = ((t.transform.translation.x - tp.transform.translation.x)**2 + (t.transform.translation.y - tp.transform.translation.y)**2)**0.5
                if d2d < self._position_interval:
                    return

            # convert timestamp to float because timestamp object is not JSON serializable
            timestamp_sec = timestamp.nanoseconds * 1.0e-9
            fp_data = {
                "information": {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "z": t.transform.translation.z,
                    "tags": [],
                    "rotation": {
                        "x": t.transform.rotation.x,
                        "y": t.transform.rotation.y,
                        "z": t.transform.rotation.z,
                        "w": t.transform.rotation.w
                    }
                },
                "data": {
                    "timestamp": timestamp_sec,
                    "beacons": []  # empty list
                }
            }
            self._fingerprints.append(fp_data)
            self._previous_fingerprint_time = timestamp
            self._previous_fingerprint_position = self._current_position
            if self._verbose:
                self.logger.info(F"dummy fingerprint data created at t={timestamp_sec}, x={t.transform.translation.x}, y={t.transform.translation.y}")


def main():
    rclpy.init()
    node = rclpy.create_node('tf2_beacons_listener')
    logger = node.get_logger()

    tfBuffer = tf2_ros.Buffer(node=node)
    tf2_ros.TransformListener(tfBuffer, node)

    # parameters
    sub_topics = node.declare_parameter("topics", ['/wireless/beacons', '/wireless/wifi']).value
    output = node.declare_parameter("output", '').value
    verbose = node.declare_parameter("verbose", True).value

    map_frame = node.declare_parameter("map_frame", "map").value
    tracking_frame = node.declare_parameter("tracking_frame", "base_link").value

    save_empty_beacon_sample = node.declare_parameter("save_empty_beacon_sample", True).value
    fingerprint_data_interval = node.declare_parameter("fingerprint_data_interval", 1.2).value  # should be larger than 1.0 s because beacon data interval is about 1.0 s.
    fingerprint_position_interval = node.declare_parameter("fingerprint_position_interval", 0.5).value  # to prevent the mapper from creating dummy fingerprint data at the same position

    # use trajectory recorder extention
    output_trajectory = node.declare_parameter("output_trajectory", '').value
    trajectory_recorder_timer_period = node.declare_parameter("trajectory_recorder_timer_period", 10.0).value
    interpolate_by_trajectory = node.declare_parameter("interpolate_by_trajectory", False).value
    trajectory_recoder = None  # default

    mapper = BeaconMapper(node,
                          save_empty_beacon_sample=save_empty_beacon_sample,
                          data_inverval=fingerprint_data_interval,
                          position_interval=fingerprint_position_interval,
                          verbose=verbose
                          )

    if output_trajectory != '' or interpolate_by_trajectory:
        trajectory_recoder = TrajectoryRecorder(node, output_trajectory, trajectory_recorder_timer_period)

    subscribers = []
    for sub_topic in sub_topics:
        logger.info("set " + sub_topic + " subscriber.")
        sub = node.create_subscription(String, sub_topic, mapper.beacons_callback, 10)
        subscribers.append(sub)

    def transform_check_loop():
        try:
            t = tfBuffer.lookup_transform(map_frame, tracking_frame, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=node.get_clock().clock_type))
            mapper.set_current_position(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            node.get_logger().error('LookupTransform Error', throttle_duration_sec=5.0)

    # run transform check 100 Hz
    node.create_timer(0.01, transform_check_loop)

    # spin
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        node.get_logger().info('node is externally shutdown')
    except Exception:
        node.get_logger().info("caught an exception other than ExternalShutdownException: "+traceback.format_exc())  # e.g. RCLError
    except KeyboardInterrupt:
        node.get_logger().info("caught KeyboardInterrupt")

    # trajectory recorder post process
    if trajectory_recoder is not None:
        trajectory_recoder.on_shutdown()

    # save to a file after node shutdown
    if output and 0 < len(mapper._fingerprints):
        node.get_logger().info("output fingerprint data before exiting")
        if interpolate_by_trajectory:
            trajectory = trajectory_recoder.get_trajectory()
            interpolator = TrajectoryBasedInterpolator(trajectory)
            samples = interpolator.interpolate_samples(mapper._fingerprints)
            node.get_logger().info(F"Reconstructed fingerprint poses from trajectory data. #samples={len(mapper._fingerprints)} -> #samples_converted={len(samples)}")
        else:
            samples = mapper._fingerprints

        with open(output, "w") as f:
            json.dump(samples, f)


if __name__ == "__main__":
    main()
