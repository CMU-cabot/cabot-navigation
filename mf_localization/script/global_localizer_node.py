#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright (c) 2025  IBM Corporation
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

import signal
import sys
import yaml

import rclpy
from rclpy.node import Node

from mf_localization.global_localizer import init_localizer
from mf_localization.global_localizer import GlobalLocalizerNode
from mf_localization.global_localizer import GLOBAL_LOCALIZER_CONFIG_KEY


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)


def main():
    rclpy.init()
    node = Node("global_localizer")
    logger = node.get_logger()

    node.declare_parameter('map_config_file', "")
    map_config_file = node.get_parameter('map_config_file').value
    if map_config_file == "":
        logger.error("map_config_file is not specified.")
        return

    map_config = None
    with open(map_config_file) as stream:
        map_config = yaml.safe_load(stream)

    # log
    logger.info(F"map_config.keys={map_config.keys()}")

    global_localizer_config = map_config.get(GLOBAL_LOCALIZER_CONFIG_KEY)
    if global_localizer_config is None:
        logger.info("global_localizer config does not exist in map_config.")
        return

    localizer = init_localizer(node, map_config)

    logger.info(F"created {type(localizer).__name__}")

    _ = GlobalLocalizerNode(node, map_config, localizer)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
