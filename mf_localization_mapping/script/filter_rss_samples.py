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

import json
import argparse
import copy
import re


def filter_samples(samples, beacon_pattern=r".", wifi_pattern=r"."):
    samples_new = []
    count = 0
    count_new = 0
    for s in samples:
        s2 = copy.copy(s)
        s2_beacons = []
        for e in s["data"]["beacons"]:
            count += 1
            if e["type"] == "iBeacon":
                if re.match(beacon_pattern, e["id"]):
                    s2_beacons.append(e)
                    count_new += 1
            elif e["type"] == "WiFi":
                if re.match(wifi_pattern, e["id"]):
                    s2_beacons.append(e)
                    count_new += 1
        s2["data"]["beacons"] = s2_beacons
        samples_new.append(s2)
    return samples_new, count, count_new


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True)
    parser.add_argument("-o", "--output")
    parser.add_argument("--wifi_pattern", default=r".")
    parser.add_argument("--beacon_pattern", default=r".")
    parser.add_argument("-v", "--verbose", action="store_true", default=False)
    args = parser.parse_args()

    input_file = args.input
    output_file = args.output
    beacon_pattern = args.beacon_pattern
    wifi_pattern = args.wifi_pattern

    with open(input_file) as f:
        samples = json.load(f)

    samples_new, count, count_new = filter_samples(samples, beacon_pattern=beacon_pattern, wifi_pattern=wifi_pattern)

    if args.verbose:
        print(f"#samples: {len(samples)} -> {len(samples_new)}")
        print(f"#samples.beacons {count} -> {count_new}")

    if output_file is not None:
        with open(output_file, "w") as f:
            json.dump(samples_new, f)
