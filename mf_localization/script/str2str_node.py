#!/usr/bin/env python3
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

import sys
import signal
import subprocess
import rclpy


class Str2Str:
    def __init__(self, node, host, port, mountpoint, authentificate, username, password, serial_port, serial_baud):
        self._node = node
        self._host = host
        self._port = port
        self._mountpoint = mountpoint
        self._authentificate = authentificate
        self._username = username
        self._password = password
        self._serial_port = serial_port
        self._serial_baud = serial_baud

        self._logger = self._node.get_logger()

        self._process = None

    def run(self):
        # str in
        str_in = "ntrip://"
        if self._authentificate:
            str_in += self._username + ":" + self._password + "@"
        str_in += self._host
        if self._port is not None:
            str_in += ":" + self._port
        if self._mountpoint is not None:
            str_in += "/" + self._mountpoint

        # str out
        str_out = "serial://" + self._serial_port + ":" + self._serial_baud

        # command
        com = ["str2str", "-in", str_in, "-out", str_out]

        try:
            self._process = subprocess.Popen(com, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except Exception as e:
            self._logger.error(f'Error executing binary: {str(e)}')

    def poll(self):
        output = self._process.stdout.readline().strip()
        rc = self._process.poll()
        return output, rc


def main():
    rclpy.init()
    node = rclpy.create_node("str2str_node")

    # str in
    host = node.declare_parameter("host", "").value
    port = str(node.declare_parameter("port", 2101).value)
    mountpoint = node.declare_parameter("mountpoint", "").value
    authentificate = node.declare_parameter("authentificate", False).value
    username = node.declare_parameter("username", "").value
    password = node.declare_parameter("password", "").value

    # str out
    serial_port = node.declare_parameter("serial_port", "ttyUBLOX").value
    serial_baud = str(node.declare_parameter("serial_baud", 230400).value)

    logger = node.get_logger()

    str2str = Str2Str(node, host, port, mountpoint, authentificate, username, password, serial_port, serial_baud)
    str2str.run()

    while rclpy.ok():
        output, rc = str2str.poll()

        if output == '' and rc is not None:
            break

        if output is not None:
            logger.info(output)

        rclpy.spin_once(node, timeout_sec=0)  # do not wait because no callback is registered for now


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)


if __name__ == "__main__":
    main()
