#!/usr/bin/env python3

# Copyright (c) 2025  Carnegie Mellon University
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

import requests
import sys
import argparse
import os

from cabot_ui import datautil
from cabot_ui import geoutil
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy import Parameter
import subprocess
import glob
import rclpy.node

# Parse command line arguments
parser = argparse.ArgumentParser(description="Fetch and display tour data")
parser.add_argument("-d", "--debug", action="store_true", help="Show configurable variables")
parser.add_argument("-l", "--lang", type=str, default="en", help="Language code for i18n text")
parser.add_argument("-t", "--tour_id", type=str, help="Specify a tour_id to show")
args = parser.parse_args()


def get_anchor_file():
    cabot_site = os.environ.get("CABOT_SITE", "")
    if not cabot_site:
        print("CABOT_SITE environment variable not set.")
        sys.exit(1)
    sitedir = get_package_share_directory(cabot_site)
    if not sitedir:
        print("Site directory not found.")
        sys.exit(1)
    config_path = os.path.join(sitedir, "config", "config.sh")
    try:
        map_value = subprocess.check_output(
            ["bash", "-c", f"sitedir='{sitedir}'; gezebo=0; source {config_path} && echo $map"],
            universal_newlines=True,
        ).strip()
    except subprocess.CalledProcessError:
        map_value = ""
    if not map_value:
        print(f"Please check config/config.sh in site package ({sitedir}) to set map and world")
        sys.exit(1)
    return map_value


class TourTestNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("tour_test_node")
        self.anchor = None
        self.datautil = None

    def set_anchor(self, anchor):
        self.anchor = anchor
        self.datautil = datautil.getInstance(self)
        self.datautil.set_anchor(self.anchor)
        self.datautil.init_by_server()
        self.datautil.set_anchor(self.anchor)
        print("Done")

if __name__ == "__main__":
    rclpy.init(args=[
        "--ros-args",
        "-p",
        "map_server_host:=localhost:9090/map",
    ])
    node = TourTestNode()
    CaBotRclpyUtil.initialize(node)
    anchor = geoutil.get_anchor(get_anchor_file())

    if anchor is None:
        print("Anchor not found. Please specify a valid anchor file.")
        sys.exit(1)

    node.set_anchor(anchor)

    try:
        rclpy.spin(node)
    except:
        pass
    node.destroy_node()



"""
MAP_SERVICE_HOST = "http://localhost:9090/map"
TOUR_DATA = "cabot/tourdata.json"
API_CONFIG = "api/config"
ROUTE_SEARCH = "routesearch"
START_ACTION = "action=start"

config_url = f"{MAP_SERVICE_HOST}/{API_CONFIG}"
print(f"Fetching config from {config_url}") if args.debug else None
response = requests.get(config_url)
if response.status_code != 200:
    print(f"Failed to fetch config: {response.status_code} - {response.text}")
    sys.exit(1)
config_data = response.json()
print(config_data) if args.debug else None
initial_location = config_data.get("INITIAL_LOCATION", {})

lang = args.lang
user = "tour-test-script"
lat = float(initial_location.get("lat", 0))
lng = float(initial_location.get("lng", 0))
dist = int(config_data.get("MAX_RADIUS", 0))

start_url = f"{MAP_SERVICE_HOST}/{ROUTE_SEARCH}?{START_ACTION}&{lat=}&{lng=}&{dist=}&user={user}&lang={lang}"
print(f"Starting route search with URL: {start_url}") if args.debug else None
response = requests.get(start_url)
if response.status_code != 200:
    print(f"Failed to start route search: {response.status_code} - {response.text}")
    sys.exit(1)
landmarks = response.json()
print(landmarks) if args.debug else None

start_url = f"{MAP_SERVICE_HOST}/{ROUTE_SEARCH}?{START_ACTION}&{lat=}&{lng=}&{dist=}&user={user}&lang={lang}"
print(f"Starting route search with URL: {start_url}") if args.debug else None
response = requests.get(start_url)
if response.status_code != 200:
    print(f"Failed to start route search: {response.status_code} - {response.text}")
    sys.exit(1)
landmarks = response.json()
landmarks = landmarks.get('landmarks', [])
print(len(landmarks)) if args.debug else None
for landmark in landmarks:
    for key, value in landmark.items():
        print(f"{key}: {value}") if args.debug else None
    break


tour_data_url = f"{MAP_SERVICE_HOST}/{TOUR_DATA}"
print(f"Fetching tour data from {tour_data_url}") if args.debug else None
response = requests.get(tour_data_url)
if response.status_code != 200:
    print(f"Failed to fetch tour data: {response.status_code} - {response.text}")
    sys.exit(1)

tour_data = response.json()
tour_list = tour_data.get("tours", [])


def get_i18n_text(data, key, lang=args.lang):
    lang_key = f"{key}-{lang}"
    if lang_key in data:
        return data[lang_key]
    if key in data:
        return data[key]

    return f"__{key}__"


for tour in tour_list:
    tour_id = tour.get("tour_id")
    if args.tour_id and tour_id != args.tour_id:
        continue
    tour_name = get_i18n_text(tour, "title")
    print(f"Tour ID: {tour_id}, Tour Name: {tour_name}")
    destinations = tour.get("destinations", [])

if not destinations:
    sys.exit(0)

for destination in destinations:
    print(f"Destination: {destination}") if args.debug else None
    dest_id = destination.get("dest_id")
    dest_name = get_i18n_text(destination, "name")
    dest_desc = get_i18n_text(destination, "description")

    print(f"Destination ID: {dest_id}, Name: {dest_name}, Description: {dest_desc}")"
"""
