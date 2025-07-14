#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
import json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="input MapData.geojson file")
    parser.add_argument("-o", "--output", help="output geojson file")
    parser.add_argument("--floors", nargs="*", type=int, help="a list of floors of which lat and lng values are adjusted")
    parser.add_argument("--d_lat", type=float, default=0.0, help="latitude difference. lat_new = lat + d_lat")
    parser.add_argument("--d_lng", type=float, default=0.0, help="longitude difference. lng_new = lng + d_lng")

    args = parser.parse_args()

    filename = args.input
    filename_out = args.output
    floors = args.floors
    d_lat = args.d_lat
    d_lng = args.d_lng

    if filename == filename_out:
        raise RuntimeError("input file must be different from output file.")

    with open(filename) as f:
        mapdata = json.load(f)

    features = mapdata["features"]

    feature_db = {}
    for feature in features:
        _id = feature["_id"]
        feature_db[_id] = feature

    new_features = []

    for feature in features:
        # print(feature)

        if feature["type"] == "Feature":
            geometry = feature["geometry"]

            if geometry["type"] == "Point" \
                    or geometry["type"] == "LineString" \
                    or geometry["type"] == "Polygon":
                pass  # do nothing
            else:
                raise RuntimeError("unknown geometry type")

            _id = feature["_id"]

            if _id.startswith("EDITOR_node"):
                print("node")
                """
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [
                            lng_0,
                            lat_0
                        ]
                    },
                    "properties": {
                        "lat": lat,
                        "lon": lon,
                        "floor": floor,
                        "link1_id": "EDITOR_link_",
                        "link2_id": "EDITOR_link_",
                        "link3_id": "EDITOR_link_",
                        "node_id": "EDITOR_node_"
                    },
                    "_id": "EDITOR_node_"
                }
                """
                coordinates = geometry["coordinates"]
                properties = feature["properties"]

                lat = coordinates[1]
                lng = coordinates[0]
                floor_ = properties["floor"]
                if floor_ in floors:
                    lat = lat + d_lat
                    lng = lng + d_lng

                    coordinates[0] = lng
                    coordinates[1] = lat

                    feature["geometry"]["coordinates"][0] = lng
                    feature["geometry"]["coordinates"][1] = lat

                    feature["properties"]["lat"] = lat
                    feature["properties"]["lon"] = lng

                    print(feature)
            elif _id.startswith("EDITOR_link"):
                print("link")
                """
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [
                            [
                                lng_0,
                                lat_0
                            ],
                            [
                                lng_1,
                                lat_1
                            ]
                        ]
                    },
                    "properties": {
                        "start_id": "EDITOR_node_0",
                        "end_id": "EDITOR_node_1",
                        "link_id": "EDITOR_link_ID",
                        ......
                    },
                    "_id": "EDITOR_link_ID"
                }
                or 
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [
                            lng_0,
                            lat_0
                        ]
                    },
                    "properties": {
                        "start_id": "EDITOR_node_0",
                        "end_id": "EDITOR_node_1",
                        "link_id": "EDITOR_link_ID",
                        ......
                    },
                    "_id": "EDITOR_link_ID"
                }
                """

                # check floor
                start_id = feature["properties"]["start_id"]
                end_id = feature["properties"]["end_id"]

                try:
                    start_floor = feature_db[start_id]["properties"]["floor"]
                except KeyError as e:
                    print(f"{e}")
                    start_floor = None

                try:
                    end_floor = feature_db[end_id]["properties"]["floor"]
                except KeyError as e:
                    print(f"{e}")
                    end_floor = None

                if geometry["type"] == "LineString":  # standard link
                    if start_floor in floors:
                        feature["geometry"]["coordinates"][0][0] += d_lng
                        feature["geometry"]["coordinates"][0][1] += d_lat
                    if end_floor in floors:
                        feature["geometry"]["coordinates"][1][0] += d_lng
                        feature["geometry"]["coordinates"][1][1] += d_lat

                    print(feature)
                elif geometry["type"] == "Point":  # vertical link (e.g. elevator)
                    if start_floor in floors or end_floor in floors:
                        print(feature)
                        feature["geometry"]["coordinates"][0] += d_lng
                        feature["geometry"]["coordinates"][1] += d_lat

            elif _id.startswith("EDITOR_facil") or _id.startswith("EDITOR_poi"):
                print("facil or poi")
                """
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [
                            lng_0,
                            lat_0
                        ]
                    },
                    "properties": {
                        "ent1_node": "EDITOR_node_ENT1",
                        "ent1_lat": ent1_lat,
                        "ent1_lon": ent1_lng,
                        "ent1_fl": ent1_floor,
                        "ent1_n": "",
                        "facil_id": "EDITOR_facil_ID",
                        "lat": lat_0,
                        "lon": lng_0,
                        "name_ja": name_ja,
                        "name_en": name_en,
                        ......
                    },
                    "_id": "EDITOR_facil_ID"
                }
                """
                """
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [
                            lng_0,
                            lat_0
                        ]
                    },
                    "properties": {
                        "lat": lat_0,
                        "lon": lng_0,
                        "hulop_major_category": hulop_major_category,
                        "hulop_sub_category": hulop_sub_category,
                        "hulop_height": hulop_height,
                        "hulop_content": "...",
                        "facil_id": "EDITOR_facil_ID",
                        ......
                    },
                    "_id": "EDITOR_facil_ID"
                }
                """

                if "ent1_node" in feature["properties"]:
                    start_floor = feature_db[feature["properties"]["ent1_node"]]["properties"]["floor"]
                    if start_floor in floors:
                        feature["geometry"]["coordinates"][0] += d_lng
                        feature["geometry"]["coordinates"][1] += d_lat

                        for i in range(10):
                            if "ent"+str(i)+"_node" in feature["properties"]:
                                feature["properties"]["ent"+str(i)+"_lat"] += d_lat
                                feature["properties"]["ent"+str(i)+"_lon"] += d_lng

                elif "hulop_major_category" in feature["properties"]:
                    floor = feature["properties"]["hulop_height"]
                    if floor in floors:
                        feature["geometry"]["coordinates"][0] += d_lng
                        feature["geometry"]["coordinates"][1] += d_lat
                        feature["properties"]["lon"] += d_lng
                        feature["properties"]["lat"] += d_lat
                else:
                    print("feature="+str(feature))
                    print("unknown facil node")
                    # raise RuntimeError("unknown facil node")
            elif _id.startswith("EDITOR_area"):               
                print("area")
                pass  # do nothing
            else:
                raise RuntimeError("unknow feature ID prefix. feature="+str(feature))
            # print(feature["_id"])
        else:
            raise RuntimeError("unknow  type")

        new_features.append(feature)

    mapdata["features"] = new_features

    if filename_out is not None:
        with open(filename_out, "w") as f:
            json.dump(mapdata, f, indent="\t", ensure_ascii=False)


if __name__ == "__main__":
    main()