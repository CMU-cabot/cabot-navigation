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
import argparse
import logging
import os
import numpy as np
import matplotlib.pyplot as plt

from sklearn.neighbors import KNeighborsRegressor, NearestNeighbors

from mf_localization.wireless_utils import extract_samples


def convert_samples_XY(samples):
    # enumerate unique transmitter IDs
    idset = set()
    for s in samples:
        data = s["data"]
        beacons = data["beacons"]
        for b in beacons:
            tid = b["id"].lower()
            idset.add(tid)

    # convert ID set to sorted array
    keys = np.array(sorted(list(idset)))

    # convert samples data to numpy array
    X = np.zeros((len(samples), 4))
    Y = -100 * np.ones((len(samples), len(keys)))

    key_to_index = {}
    for i, key in enumerate(keys):
        key_to_index[key] = i

    for i, s in enumerate(samples):
        info = s["information"]
        rssi_gain = info.get("rssi_gain", 0.0)
        if "floor_num" in info:
            # for samples data converted on the Map Server
            floor = info["floor_num"]
        elif "floor" in info:
            floor = info["floor"]
        else:
            floor = 0.0

        X[i] = np.array([info["x"], info["y"], info["z"], floor])

        data = s["data"]
        beacons = data["beacons"]

        for b in beacons:
            key = b["id"].lower()
            rssi = float(b["rssi"])
            rssi = rssi - rssi_gain
            if -100 < rssi < -1:
                index = key_to_index[key]
                Y[i, index] = rssi

    return X, Y, keys, key_to_index


def create_wireless_rss_localizer(localizer_class, logger=None, n_neighbors=1, max_rssi_threshold=-100, min_beacons=3, rssi_offset=0.0, n_strongest=10):
    if localizer_class in globals():
        class_floor_localizer = globals()[localizer_class]
    else:
        error_str = "unknown floor localizer class (floor_localizer: "+str(
            localizer_class)+")"
        raise RuntimeError(error_str)

    if issubclass(class_floor_localizer, RSSLocalizer):
        localizer = class_floor_localizer(
                        logger=logger,
                        n_neighbors=n_neighbors, max_rssi_threshold=max_rssi_threshold, min_beacons=min_beacons, rssi_offset=rssi_offset, n_strongest=n_strongest)
        return localizer
    else:
        error_str = "unknown floor localizer class (floor_localizer: "+str(
            localizer_class)+")"
        raise RuntimeError(error_str)


class RSSLocalizer:
    pass


class SimpleRSSLocalizer(RSSLocalizer):

    def __init__(self, logger=None,
                 n_neighbors=1, max_rssi_threshold=-100, min_beacons=3, rssi_offset=0.0,
                 n_strongest=10):
        if logger is None:
            self._logger = logging.getLogger(__name__)
        else:
            self._logger = logger
        self._n_neighbors = n_neighbors
        self._max_rssi_threshold = max_rssi_threshold
        self._min_beacons = min_beacons
        self._rssi_offset = rssi_offset
        self._n_strongest = n_strongest

        self._logger.debug(f"{self.__class__.__name__}")
        self._logger.debug(f"    n_neighbors = {self._n_neighbors}")
        self._logger.debug(f"    max_rssi_threshold = {self._max_rssi_threshold}")
        self._logger.debug(f"    min_beacons = {self._min_beacons}")
        self._logger.debug(f"    rssi_offset = {self._rssi_offset}")
        self._logger.debug(f"    n_strongest = {self._n_strongest}")

    def fit(self, samples):
        self._samples = samples

        X, Y, keys, key_to_index = convert_samples_XY(samples)

        self._X = X
        self._Y = Y
        self._keys = keys
        self._key_to_index = key_to_index

    def predict(self, beacons):

        # create an RSS vector
        keys = self._keys
        y = -100 * np.ones((1, len(keys)))

        max_rssi = -100
        c_match = 0
        indices_active = []

        # filter beacons
        beacons_filtered = [b for b in beacons if b["id"].lower() in keys and -100 < b["rssi"] <= -1]  # known and valid rssi beacons
        beacons_sorted = sorted(beacons_filtered, key=lambda b: b["rssi"], reverse=True)  # [strongest, 2nd-strongest, ...]
        beacons_strongest = beacons_sorted[: self._n_strongest]

        for b in beacons_strongest:
            key = b["id"].lower()
            rssi = float(b["rssi"])
            if -100 < rssi < -1:  # check if raw rssi is in the range
                rssi = rssi - self._rssi_offset  # apply rssi offset

                if key in keys:
                    index = self._key_to_index[key]
                    y[0, index] = rssi
                    c_match += 1
                    max_rssi = np.max([max_rssi, rssi])
                    indices_active.append(index)

        # return if the input does not match the stored data
        if c_match == 0:
            return None  # undetermined

        # unreliable
        if c_match < self._min_beacons:
            return None
        elif max_rssi <= self._max_rssi_threshold:
            return None

        # create a knn regressor by using visible transmitter IDs
        indices_active = np.array(indices_active)
        knnr = KNeighborsRegressor(n_neighbors=self._n_neighbors)
        Y_active = self._Y[:, indices_active]
        knnr.fit(Y_active, self._X)
        _, neigh_ind = knnr.kneighbors(y[:, indices_active])
        x = np.mean(self._X[neigh_ind], axis=1)

        self._logger.debug(f"y_query={y[:, indices_active]}")
        self._logger.debug(f"Y_knn={Y_active[neigh_ind]}")
        self._logger.debug(f"x_knn={self._X[neigh_ind]}")

        return x

    def find_closest(self, x):
        if x is None:
            return None

        knnr = KNeighborsRegressor(n_neighbors=self._n_neighbors)  # noqa
        X = self._X  # noqa

        nn = NearestNeighbors(n_neighbors=1)
        nn.fit(self._X)
        ret = nn.kneighbors(x, return_distance=False)

        return self._X[ret][0]


class SimpleFloorLocalizer(RSSLocalizer):

    def __init__(self, logger=None,
                 n_neighbors=1, max_rssi_threshold=-100, min_beacons=3, rssi_offset=0.0,
                 n_strongest=10):
        if logger is None:
            self._logger = logging.getLogger(__name__)
        else:
            self._logger = logger
        self._n_neighbors = n_neighbors
        self._max_rssi_threshold = max_rssi_threshold
        self._min_beacons = min_beacons
        self._rssi_offset = rssi_offset
        self._n_strongest = n_strongest

        self._logger.debug(f"{self.__class__.__name__}")
        self._logger.debug(f"    n_neighbors = {self._n_neighbors}")
        self._logger.debug(f"    max_rssi_threshold = {self._max_rssi_threshold}")
        self._logger.debug(f"    min_beacons = {self._min_beacons}")
        self._logger.debug(f"    rssi_offset = {self._rssi_offset}")
        self._logger.debug(f"    n_strongest = {self._n_strongest}")

    def fit(self, samples):
        self._samples = samples

        X, Y, keys, key_to_index = convert_samples_XY(samples)

        self._X = X
        self._Y = Y
        self._keys = keys
        self._key_to_index = key_to_index

        # calculate rep position for each beacon
        top_n = self._n_neighbors
        index_to_rep_position = []
        for index, key in enumerate(keys):
            indices_largest = np.argsort(Y[:, index])[-top_n:]  # top n
            rep_position = np.mean(X[indices_largest, :], axis=0)
            index_to_rep_position.append(rep_position)
            self._logger.debug(f"key={key},rep_position={rep_position}")
            self._logger.debug(f"    Y={sorted(Y[:, index])[-top_n:]}")
            self._logger.debug(f"    X={X[indices_largest, :]}")
        index_to_rep_position = np.array(index_to_rep_position)

        self._index_to_rep_position = index_to_rep_position

    def predict(self, beacons):
        # input check
        keys = self._keys

        max_rssi = -100
        c_match = 0

        # filter beacons
        beacons_filtered = [b for b in beacons if b["id"].lower() in keys and -100 < b["rssi"] <= -1]  # known and valid rssi beacons

        for b in beacons_filtered:
            key = b["id"].lower()
            rssi = float(b["rssi"])
            rssi = rssi - self._rssi_offset  # apply rssi offset
            c_match += 1
            max_rssi = np.max([max_rssi, rssi])

        # return if the input does not match the stored data
        if c_match == 0:
            return None  # undetermined

        # unreliable
        if c_match < self._min_beacons:
            return None
        elif max_rssi <= self._max_rssi_threshold:
            return None

        # calculate weighted mean of rep positions
        key_list = []
        rssi_list = []
        rep_positions = []
        weights = []

        beacons_strongest = sorted(beacons_filtered, key=lambda b: -b["rssi"])

        count = 0
        n_max_strongest = self._n_strongest
        for b in beacons_strongest:
            key = b["id"].lower()

            if key not in keys:
                continue

            key_list.append(key)
            idx = self._key_to_index[key]
            rep_position = self._index_to_rep_position[idx]
            rep_positions.append(rep_position)

            weight = 1.0/np.power(10, b["rssi"]/-20.0)
            rssi_list.append(b["rssi"])
            weights.append(weight)

            count += 1
            if count >= n_max_strongest:
                break

        rep_positions = np.array(rep_positions)
        weights = np.array(weights)
        weights = weights/np.sum(weights)  # normalize weight

        self._logger.debug(f"keys={key_list}\nrssi_list={rssi_list}\nrep_positions={rep_positions}\nweights={weights}")

        loc = np.array([np.dot(weights, rep_positions)])

        return loc

    def find_closest(self, x):
        if x is None:
            return None

        knnr = KNeighborsRegressor(n_neighbors=self._n_neighbors)  # noqa
        X = self._X  # noqa

        nn = NearestNeighbors(n_neighbors=1)
        nn.fit(self._X)
        ret = nn.kneighbors(x, return_distance=False)

        return self._X[ret][0]


def main(samples_file, queries_file, localizer_class):
    logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG if 'DEBUG' in os.environ else logging.INFO)

    with open(samples_file) as f:
        samples = json.load(f)

    beacons_samples = extract_samples(samples, key="iBeacon")
    wifi_samples = extract_samples(samples, key="WiFi")

    logger.info(f"localizer = {localizer_class}")
    logger.info("#beacons="+str(len(beacons_samples)))
    logger.info("#wifi="+str(len(wifi_samples)))

    wifi_localizer = create_wireless_rss_localizer(localizer_class, logger, n_neighbors=3)
    wifi_localizer.fit(wifi_samples)

    ble_localizer = create_wireless_rss_localizer(localizer_class, logger, n_neighbors=3)
    ble_localizer.fit(beacons_samples)

    if queries_file is not None:
        with open(queries_file) as f:
            q_samples = json.load(f)

        X_w = []
        X_b = []
        Y_w = []
        Y_b = []
        ble_errors = []
        wifi_errors = []
        for q in q_samples:
            q_info = q["information"]
            loc_gt = np.array([q_info["x"], q_info["y"], q_info["z"], 0.0])

            q_beacons = q["data"]["beacons"]

            res = wifi_localizer.predict(q_beacons)
            if res is not None:
                loc_est = res[0]
                X_w.append(list(loc_gt))
                Y_w.append(list(loc_est))
                wifi_error = np.linalg.norm(loc_gt - loc_est)
                wifi_errors.append(wifi_error)

            res = ble_localizer.predict(q_beacons)
            if res is not None:
                loc_est = res[0]
                X_b.append(list(loc_gt))
                Y_b.append(list(loc_est))
                ble_error = np.linalg.norm(loc_gt - loc_est)
                ble_errors.append(ble_error)

        X_w = np.array(X_w)
        Y_w = np.array(Y_w)
        X_b = np.array(X_b)
        Y_b = np.array(Y_b)

        print("mean(wifi_errors)="+str(np.mean(wifi_errors)))
        print("mean(ble_errors)="+str(np.mean(ble_errors)))
        plt.hist(ble_errors, bins=500, normed=True,
                 cumulative=True, histtype="step", label="BLE")
        plt.hist(wifi_errors, bins=500, normed=True,
                 cumulative=True, histtype="step", label="wifi")
        plt.ylim([0, 1])
        plt.legend()
        plt.show()

        plt.scatter(X_b[:, 0], X_b[:, 1], label="BLE_gt", color="k")
        plt.scatter(Y_b[:, 0], Y_b[:, 1], label="BLE", color="b")
        plt.gca().set_aspect('equal')
        plt.legend()
        plt.show()

        plt.scatter(X_w[:, 0], X_w[:, 1], label="WiFi_gt", color="k")
        plt.scatter(Y_w[:, 0], Y_w[:, 1], label="WiFi_est", color="b")
        plt.gca().set_aspect('equal')
        plt.legend()
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--samples", required=True)
    parser.add_argument("-q", "--queries", default=None)
    parser.add_argument("--localizer", default="SimpleRSSLocalizer")
    args = parser.parse_args()

    samples_file = args.samples
    queries_file = args.queries
    localizer_class = args.localizer

    main(samples_file, queries_file, localizer_class)
