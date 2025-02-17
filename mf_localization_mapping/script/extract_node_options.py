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
import re
import numpy


def main():
    parser = argparse.ArgumentParser("Extract options from cartographer_node log")
    parser.add_argument("-i", "--input_log", required=True, help="input log file")
    parser.add_argument("-o", "--output", default=None, help="output yaml file")
    args = parser.parse_args()

    # input file is cartographer node log
    with open(args.input_log) as f:
        string = f.read()

    pattern = r'.*Using NavSatFix.*lat = ([0-9.]+).*long = ([0-9.]+).*use_enu_local_frame = ([0-1]).*use_spherical_mercator = ([0-1]).*'
    matches = re.findall(pattern, string)
    if len(matches) > 0:
        lat, long, use_enu_local_frame, use_spherical_mercator = matches[0]
    else:
        lat, long, use_enu_local_frame, use_spherical_mercator = numpy.nan, numpy.nan, "false", "false"

    # replace text
    use_enu_local_frame = "true" if use_enu_local_frame in [1, "1"] else "false"
    use_spherical_mercator = "true" if use_spherical_mercator in [1, "1"] else "false"

    lines = [
                F"options.nav_sat_predefined_enu_frame_latitude: {lat}",
                F"options.nav_sat_predefined_enu_frame_longitude: {long}",
                F"options.nav_sat_use_enu_local_frame: {use_enu_local_frame}",
                F"options.nav_sat_use_spherical_mercator: {use_spherical_mercator}",
            ]

    print("\n".join(lines))

    # save to a yaml file
    output = args.output
    if output is not None and output != "":
        with open(output, "w") as f:
            f.write("\n".join(lines))


if __name__ == "__main__":
    main()
