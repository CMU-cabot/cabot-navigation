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
import csv
import re


def main():
    parser = argparse.ArgumentParser("Extract fixed_frame_pose_data from output of cartographer_pbstream info command for debugging purpose")
    parser.add_argument("-i", "--input", required=True, help="input file")
    parser.add_argument("-o", "--output", default=None, help="output csv file")
    args = parser.parse_args()

    # input file is stderr output of cartographer_pbstream info command
    # example command to write stderr output to a file
    # $ PATH_TO_CARTOGRAPHER_BIN/cartographer_pbstream info YOUR_PBSTREAM_FILE.pbstream --all_debug_strings 2> YOUR_PBSTREAM_INFO.txt
    with open(args.input) as f:
        string = f.read()

    # extract fixed_from_pose_data by pattern matching
    # expected pattern
    #
    # Serialized data: fixed_frame_pose_data {
    #     fixed_frame_pose_data {
    #         timestamp: timestamp
    #         pose {
    #             translation {
    #                 x: x
    #                 y: y
    #                 z: z
    #             }
    #             rotation {
    #                 w: w
    #             }
    #         }
    #     }
    # }
    pattern = r'Serialized data: fixed_frame_pose_data {\s*fixed_frame_pose_data {\s*timestamp: (\d+)\s*pose {\s*translation {\s*x: ([\d\-.]+)\s*y: ([\d\-.]+)\s*z: ([\d\-.]+)\s*}\s*rotation {\s*w: ([\d\-.]+)'
    matches = re.findall(pattern, string, re.DOTALL)

    # save the extracted fixed_frame_pose data to a file
    data = []
    for match in matches:
        timestamp, x, y, z, w = match
        print(f"timestamp: {timestamp}, x: {x}, y: {y}, z: {z}, w: {w}")
        data.append([timestamp, x, y, z, w])

    # save to a csv file
    output = args.output
    if output is not None and output != "":
        with open(output, "w") as f:
            writer = csv.writer(f, lineterminator="\n")
            header = ["timestamp", "x", "y", "z", "w"]
            writer.writerow(header)
            writer.writerows(data)


if __name__ == "__main__":
    main()
