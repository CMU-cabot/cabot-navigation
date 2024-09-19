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

import argparse
import json
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy
import scipy.interpolate
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

from geometry_msgs.msg import PoseStamped, Point, Quaternion


def csv_to_trajectory(csv_file: str):
    df_trajectory: pd.DataFrame = pd.read_csv(csv_file)
    trajectory: list[PoseStamped] = []
    for index, point in df_trajectory.iterrows():
        pose_stamped = PoseStamped()
        sec = int(point.timestamp)
        nanosec = int((point.timestamp - sec) * 1.0e9)
        pose_stamped.header.stamp.sec = sec
        pose_stamped.header.stamp.nanosec = nanosec
        pose_stamped.header.frame_id = point.frame_id
        pose_stamped.pose.position = Point(x=point.x, y=point.y, z=point.z)
        pose_stamped.pose.orientation = Quaternion(x=point.qx, y=point.qy, z=point.qz, w=point.qw)
        trajectory.append(pose_stamped)
    return trajectory


def trajectory_to_dataframe(trajectory) -> pd.DataFrame:
    columns = ["timestamp", "frame_id", "x", "y", "z", "qx", "qy", "qz", "qw"]
    X = []
    for pose_stamped in trajectory:
        header = pose_stamped.header
        pose = pose_stamped.pose
        timestamp = header.stamp.sec + 1.0e-9 * header.stamp.nanosec
        frame_id = header.frame_id
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        X.append([timestamp, frame_id, x, y, z, qx, qy, qz, qw])

    return pd.DataFrame(X, columns=columns)


class TrajectoryBasedInterpolator:
    def __init__(self, trajectory):
        df_trajectory = trajectory_to_dataframe(trajectory)
        # create xyz interpolator
        self.xyz_interpolator = scipy.interpolate.interp1d(df_trajectory["timestamp"].values, df_trajectory[["x", "y", "z"]].T)
        # create rotation interpolator
        q_list = []
        for index, point in df_trajectory.iterrows():
            q = [point.qw, point.qx, point.qy, point.qz]  # w, x, y, z order
            q_list.append(q)
        rotations = Rotation.from_quat(q_list)
        self.slerp = Slerp(df_trajectory["timestamp"].values, rotations)

    def interpolate_pose(self, timestamp):
        try:
            xyz_interpolated = self.xyz_interpolator(timestamp)
            rotation_interpolated = self.slerp(timestamp)
        except ValueError as e:
            raise e
        q_interpolated = rotation_interpolated.as_quat()  # w, x, y, z order
        return [timestamp, xyz_interpolated[0], xyz_interpolated[1], xyz_interpolated[2], q_interpolated[1], q_interpolated[2], q_interpolated[3], q_interpolated[0]]  # timestamp, x, y, z, qx, qy, qz, w

    def interpolate_samples(self, samples):
        samples_new = []
        for s in samples:
            timestamp = s["data"]["timestamp"]

            try:
                pose_interp = self.interpolate_pose(timestamp)
            except ValueError as e:
                print(F"Skipped interpolation because a ValueError was raised. ValueError: {e}")
                continue

            s_new = s.copy()
            info = s["information"]
            info_new = info.copy()

            info_new["x"], info_new["y"], info_new["z"] = pose_interp[1], pose_interp[2], pose_interp[3]
            info_new["rotation"]["x"], info_new["rotation"]["y"], info_new["rotation"]["z"], info_new["rotation"]["w"] = pose_interp[4], pose_interp[5], pose_interp[6], pose_interp[7]

            s_new["information"] = info_new
            samples_new.append(s_new)

        return samples_new


def interpolate_samples_pose_by_trajectory(samples, trajectory):
    trajectory_based_interpolator = TrajectoryBasedInterpolator(trajectory)
    samples_new = trajectory_based_interpolator.interpolate_samples(samples)
    return samples_new


if __name__ == "__main__":
    parser = argparse.ArgumentParser("interpolate poses in samples by optimized trajectory")
    parser.add_argument("--samples", required=True, help="path to samples json data")
    parser.add_argument("--trajectory", required=True, help="path to trajectory csv data")
    parser.add_argument("--output_samples", default=None, help="output path to interpolated samples json data")
    parser.add_argument("--plot", default=False, action="store_true", help="plot data")
    args = parser.parse_args()

    input_samples = args.samples
    input_trajectory = args.trajectory
    output_samples = args.output_samples
    plot = args.plot

    # load samples
    with open(input_samples) as f:
        samples = json.load(f)

    # load trajectory csv data as a trajectory object (PoseStamped[])
    trajectory = csv_to_trajectory(input_trajectory)

    samples_new = interpolate_samples_pose_by_trajectory(samples, trajectory)

    if output_samples is not None:
        with open(output_samples, "w") as f:
            json.dump(samples_new, f)

    # plot data
    if plot:
        X_raw = []
        X_new = []

        for s in samples:
            info = s["information"]
            X_raw.append([info["x"], info["y"], info["z"]])

        for s in samples_new:
            info = s["information"]
            X_new.append([info["x"], info["y"], info["z"]])

        X_raw = np.array(X_raw)
        X_new = np.array(X_new)

        plt.scatter(X_raw[:, 0], X_raw[:, 1], label="original")
        plt.scatter(X_new[:, 0], X_new[:, 1], label="interpolated")
        plt.legend()
        plt.gca().set_aspect("equal")
        plt.show()
