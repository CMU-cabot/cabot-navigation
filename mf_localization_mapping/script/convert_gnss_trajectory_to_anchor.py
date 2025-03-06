#!/usr/bin/env python
# -*- coding: utf-8 -*

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

import argparse
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from mf_localization import geoutil # use mf_localization/src/geoutil.py


def rt2d(X, Y):
    """
    estimate rotation and translation from 2D point corespondances
    input: X(n_samples, 2), Y(n_samples, 2)
    """
    Xmean = np.mean(X, axis=0)
    Ymean = np.mean(Y, axis=0)

    Xdev = X - Xmean
    Ydev = Y - Ymean

    H = np.dot(np.transpose(Xdev), Ydev)
    U, S, Vt = np.linalg.svd(H)

    # rotation
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0.0:  # check reflection
        Vt[-1, :] = -Vt[-1, :]
        S[-1] = -S[-1]
    R = np.dot(Vt.T, U.T)

    # translation
    t = - np.dot(R, Xmean.T) + Ymean.T
    return R, t


def main():
    parser = argparse.ArgumentParser("Tool to convert gnss and trajectory data to anchor")
    parser.add_argument("-g", "--gnss", required=True, help="gnss csv file")
    parser.add_argument("-t", "--trajectory", required=True, help="trajectory csv file")
    parser.add_argument("-s", "--status_threshold", default=2, type=int)
    parser.add_argument("-c", "--cov_threshold", default=0.01, type=float)

    args = parser.parse_args()
    gnss_file = args.gnss
    trajectory_file = args.trajectory
    navsat_status_threshold = args.status_threshold
    position_covariance_threshold = args.cov_threshold

    # data frame
    df_gnss = pd.read_csv(gnss_file)
    df_trajectory = pd.read_csv(trajectory_file)

    # find the most accurate point and use it as a global to local conversion anchor
    best_index = np.argmin(df_gnss.position_covariance_0.values)
    best_point = df_gnss.loc[best_index]
    gnss_anchor = geoutil.Anchor(lat=best_point.latitude, lng=best_point.longitude, rotate=0.0)
    print(f"gnss_anchor={gnss_anchor}, index={best_index}, cov0={best_point.position_covariance_0}")

    # prepare array to store data
    X = []  # robot local coordinate
    Y = []  # gnss local coordinate
    status_array = []
    cov_array = []

    # iterate for trajectory nodes because the time intervals are larger than gnss data
    for index, point in df_trajectory.iterrows():
        timestamp = point.timestamp
        target_index = (df_gnss["timestamp"] - timestamp).abs().idxmin()
        gnss_point = df_gnss.loc[target_index]

        # convert to local coordinate
        latlng = geoutil.Latlng(lat=gnss_point.latitude, lng=gnss_point.longitude)
        xy = geoutil.global2local(latlng, gnss_anchor)

        X.append([point.x, point.y])
        Y.append([xy.x, xy.y])
        status_array.append(gnss_point.status)
        cov_array.append(gnss_point.position_covariance_0)

    X = np.array(X)
    Y = np.array(Y)
    status_array = np.array(status_array)
    cov_array = np.array(cov_array)

    # select reliable points
    indices_accurate = np.where((navsat_status_threshold <= status_array) * (cov_array <= position_covariance_threshold))
    X2 = X[indices_accurate]
    Y2 = Y[indices_accurate]

    # calculate Rotation and translation
    R, t = rt2d(X2, Y2)

    # convert rotation matrix to theta
    theta = math.atan2(R[1, 0], R[0, 0])

    # plot
    Xproj = np.dot(R, X.T).T + t
    plt.scatter(Xproj[:, 0], Xproj[:, 1], label="X aligned", s=0.1)
    plt.scatter(Y[:, 0], Y[:, 1], label="Y", s=np.sqrt(cov_array))
    plt.scatter(t[0], t[1], label="X origin")
    plt.gca().set_aspect("equal")
    plt.legend()
    plt.show()

    X2proj = np.dot(R, X2.T).T + t
    projection_errors = np.sqrt((X2proj[:, 0] - Y2[:, 0])**2 + (X2proj[:, 1] - Y2[:, 1])**2)
    plt.scatter(Y[:, 0], Y[:, 1], label="Y", s=np.sqrt(cov_array))
    plt.scatter(Y2[:, 0], Y2[:, 1], label="projection error", s=projection_errors)
    plt.gca().set_aspect("equal")
    plt.legend()
    plt.show()

    plt.plot(projection_errors)
    plt.show()

    # convert the origin of X to lat, lng
    # y0 = np.dot(R, x0.T) + t = t (because x0=[0,0])
    latlng_origin = geoutil.local2global(geoutil.Point(x=t[0], y=t[1]), gnss_anchor)
    rotate = -np.degrees(theta)

    print("anchor: lat="+str(latlng_origin.lat)+", lng="+str(latlng_origin.lng)+", rotate="+str(rotate))
    return


if __name__ == "__main__":
    main()
