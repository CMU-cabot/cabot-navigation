#!/usr/bin/env python3

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
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy
import scipy.interpolate

from mf_localization import geoutil


def main():
    parser = argparse.ArgumentParser("Compare optimized trajectory and gnss data for evaluation/debug purpose")
    parser.add_argument("--trajectory", required=True, help="trajectory log csv file")
    parser.add_argument("--gnss", default=None, help="gnss log csv file")
    parser.add_argument("--fixed_frame_pose", default=None, help="fixed frame pose data csv file")
    parser.add_argument("--gnss_status_threshold", default=2, type=int, help="status threshold for gnss data. gnss data with gnss_status_threshold<=status are used to evaluate error")
    args = parser.parse_args()
    status_threshold = args.gnss_status_threshold

    # trajectory data
    # expected format
    # timestamp,frame_id,x,y,z,qx,qy,qz,qw
    df_trajectory = pd.read_csv(args.trajectory)

    # gnss data
    # expected format
    # timestamp,header_stamp,frame_id,status,service,latitude,longitude,altitude,position_covariance_type,position_covariance_0,...
    if args.gnss is not None:
        df_gnss = pd.read_csv(args.gnss)
        point = df_gnss.iloc[0]
        gnss_anchor = geoutil.Anchor(lat=point.latitude, lng=point.longitude, rotate=0.0)

        X_gnss = []  # gnss local coordinate
        for index, point in df_gnss.iterrows():
            # convert to local coordinate
            latlng = geoutil.Latlng(lat=point.latitude, lng=point.longitude)
            xy = geoutil.global2local(latlng, gnss_anchor)
            X_gnss.append([xy.x, xy.y])
        X_gnss = np.array(X_gnss)
        df_gnss["x"] = X_gnss[:, 0]
        df_gnss["y"] = X_gnss[:, 1]

        plt.scatter(df_gnss["x"], df_gnss["y"], label="GNSS position converted by geoutil")

    # fixed frame pose data
    # expected format
    # timestamp,x,y,z,w
    if args.fixed_frame_pose is not None:
        df_pose = pd.read_csv(args.fixed_frame_pose)
        plt.scatter(df_pose["x"], df_pose["y"], label="Fixed frame pose data")

    plt.scatter(df_trajectory["x"], df_trajectory["y"], label="Optimized trajectory")

    plt.legend()
    plt.gca().set_aspect('equal')
    plt.show()

    # error evaluation
    if args.gnss is not None:
        error_data = []
        interpolator = scipy.interpolate.interp1d(df_gnss["timestamp"].values, df_gnss[["x", "y", "status"]].T)
        for index, point in df_trajectory.iterrows():
            x_traj = point.x
            y_traj = point.y
            try:
                x_interp, y_interp, status_interp = interpolator(point.timestamp)
            except ValueError as e:
                print(F"Skipped interploation at timestamp = {point.timestamp} because an error is raised. ValueError: {e}")
                continue
            if status_threshold <= status_interp:
                error = np.linalg.norm([x_interp - x_traj, y_interp - y_traj])
                error_data.append([point.timestamp, point.x, point.y, error])
        error_data = np.array(error_data)

        print(F"error stats: count={len(error_data)}, mean={np.mean(error_data[:, 3])}, min={np.min(error_data[:, 3])}, 95%-ile={np.percentile(error_data[:, 3], q=95)}, max={np.max(error_data[:, 3])}")

        # plot error time series
        plt.scatter(error_data[:, 0], error_data[:, 3])
        plt.xlabel("timestamp")
        plt.ylabel("error [m]")
        plt.show()

        # plot error histogram
        plt.hist(error_data[:, 3], bins=1000, cumulative=True, density=True, histtype="step")
        plt.xlim(0)
        plt.ylim([0, 1])
        plt.xlabel("error [m]")
        plt.ylabel("cumularive distribution")
        plt.show()

        # plot error distribution
        plt.scatter(error_data[:, 1], error_data[:, 2], c=error_data[:, 3])
        plt.colorbar(label="error [m]")
        plt.gca().set_aspect('equal')
        plt.show()


if __name__ == "__main__":
    main()
