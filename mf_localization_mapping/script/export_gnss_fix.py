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
from pathlib import Path
from rosbags.highlevel import AnyReader


def main():
    parser = argparse.ArgumentParser("Read ROS bag and output fix messages to a csv file")
    parser.add_argument("-i", "--input", required=True, help="ROS bag directory")  # bag
    parser.add_argument("-o", "--output", default=None, help="output csv file path")  # csv
    parser.add_argument("--fix_topic", default="/ublox/fix")
    args = parser.parse_args()

    bagpath = Path(args.input)

    data = []
    # read bag
    with AnyReader([bagpath]) as reader:
        connections = [x for x in reader.connections if x.topic == args.fix_topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # sensor_msgs/msg/NavSatFix
            msg = reader.deserialize(rawdata, connection.msgtype)

            t_sec = timestamp * 1.0e-9  # nanoseconds -> seconds
            header_stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
            frame_id = msg.header.frame_id
            status = msg.status.status
            service = msg.status.service
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
            position_covariance = msg.position_covariance
            position_covariance_type = msg.position_covariance_type

            data.append([t_sec, header_stamp, frame_id, status, service, latitude, longitude, altitude, position_covariance_type, *position_covariance])

    # save to a file
    output = args.output
    if output is not None and output != "":
        header = ["timestamp", "header_stamp", "frame_id", "status", "service", "latitude", "longitude", "altitude", "position_covariance_type"]
        header.extend(["position_covariance_" + str(i) for i in range(9)])

        with open(output, "w") as f:
            writer = csv.writer(f, lineterminator="\n")
            writer.writerow(header)
            writer.writerows(data)


if __name__ == "__main__":
    main()
