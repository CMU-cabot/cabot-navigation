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
import os
import subprocess
import sys


def _as_set(values):
    if values is None:
        return None
    values = [v for v in values if v is not None]
    if len(values) == 0:
        return None
    return set(values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="input MapData.geojson file")
    parser.add_argument("-o", "--output", help="output geojson file (default: <output_dir>/<input_basename>.shifted.geojson)")
    parser.add_argument("--output_dir", default="/tmp", help="default output directory when --output is not specified")
    parser.add_argument("--floors", nargs="*", type=int, help="a list of floors of which lat and lng values are adjusted")
    parser.add_argument("--all_floors", action="store_true", help="adjust all floors (use with care)")
    parser.add_argument(
        "--in_out",
        nargs="*",
        type=int,
        help="only adjust features connected to nodes with these in_out values (e.g., indoor/outdoor)",
    )
    parser.add_argument(
        "--allow_unlinked_features",
        action="store_true",
        help="when --in_out is set, also adjust POIs without entrance node references based on floor only",
    )
    parser.add_argument("--d_lat", type=float, default=0.0, help="latitude difference. lat_new = lat + d_lat")
    parser.add_argument("--d_lng", type=float, default=0.0, help="longitude difference. lng_new = lng + d_lng")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--write_diff", action="store_true", help="also write a diff GeoJSON for visualization")
    parser.add_argument("--diff_output_dir", default=None, help="output directory for diff GeoJSON (default: --output_dir)")

    args = parser.parse_args()

    filename = args.input
    filename_out = args.output
    output_dir = args.output_dir
    floors = args.floors
    all_floors = args.all_floors
    in_out_set = _as_set(args.in_out)
    d_lat = args.d_lat
    d_lng = args.d_lng
    verbose = args.verbose

    if filename_out is None:
        base = os.path.basename(filename)
        if base.lower().endswith(".geojson"):
            base = os.path.splitext(base)[0]
        filename_out = os.path.join(output_dir, base + ".shifted.geojson")

    if filename == filename_out:
        raise RuntimeError("input file must be different from output file.")

    if floors is not None and len(floors) == 0:
        floors = None

    if floors is None and not all_floors and in_out_set is None:
        raise RuntimeError("Specify --floors (recommended) or --in_out or --all_floors.")

    floors_set = None if all_floors or floors is None else set(floors)

    def floor_ok(floor_value):
        if floors_set is None:
            return True
        return floor_value in floors_set

    with open(filename) as f:
        mapdata = json.load(f)

    features = mapdata["features"]

    feature_db = {}
    for feature in features:
        _id = feature["_id"]
        feature_db[_id] = feature

    new_features = []

    def node_ok(node_feature):
        if node_feature is None:
            return False
        props = node_feature.get("properties", {})
        floor_value = props.get("floor")
        if floor_value is None:
            return False
        if not floor_ok(floor_value):
            return False
        if in_out_set is not None and props.get("in_out") not in in_out_set:
            return False
        return True

    def node_id_ok(node_id):
        return node_ok(feature_db.get(node_id))

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
                if verbose:
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
                if node_ok(feature):
                    lat = lat + d_lat
                    lng = lng + d_lng

                    coordinates[0] = lng
                    coordinates[1] = lat

                    feature["geometry"]["coordinates"][0] = lng
                    feature["geometry"]["coordinates"][1] = lat

                    feature["properties"]["lat"] = lat
                    feature["properties"]["lon"] = lng

                    if verbose:
                        print(feature)
            elif _id.startswith("EDITOR_link"):
                if verbose:
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

                start_ok = node_id_ok(start_id)
                end_ok = node_id_ok(end_id)

                if geometry["type"] == "LineString":  # standard link
                    if start_ok:
                        feature["geometry"]["coordinates"][0][0] += d_lng
                        feature["geometry"]["coordinates"][0][1] += d_lat
                    if end_ok:
                        feature["geometry"]["coordinates"][1][0] += d_lng
                        feature["geometry"]["coordinates"][1][1] += d_lat

                    if verbose:
                        print(feature)
                elif geometry["type"] == "Point":  # vertical link (e.g. elevator)
                    if start_ok or end_ok:
                        if verbose:
                            print(feature)
                        feature["geometry"]["coordinates"][0] += d_lng
                        feature["geometry"]["coordinates"][1] += d_lat

            elif _id.startswith("EDITOR_facil") or _id.startswith("EDITOR_poi"):
                if verbose:
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
                    ent_nodes = []
                    for i in range(10):
                        node_key = "ent"+str(i)+"_node"
                        if node_key in feature["properties"]:
                            ent_nodes.append(feature["properties"][node_key])

                    should_shift = any(node_id_ok(node_id) for node_id in ent_nodes)
                    if should_shift:
                        feature["geometry"]["coordinates"][0] += d_lng
                        feature["geometry"]["coordinates"][1] += d_lat

                        if "lon" in feature["properties"]:
                            feature["properties"]["lon"] += d_lng
                        if "lat" in feature["properties"]:
                            feature["properties"]["lat"] += d_lat

                        for i in range(10):
                            node_key = "ent"+str(i)+"_node"
                            if node_key not in feature["properties"]:
                                continue
                            if node_id_ok(feature["properties"][node_key]):
                                lat_key = "ent"+str(i)+"_lat"
                                lon_key = "ent"+str(i)+"_lon"
                                if lat_key in feature["properties"]:
                                    feature["properties"][lat_key] += d_lat
                                if lon_key in feature["properties"]:
                                    feature["properties"][lon_key] += d_lng

                elif "hulop_major_category" in feature["properties"]:
                    if in_out_set is not None and not args.allow_unlinked_features:
                        # Cannot determine indoor/outdoor without node references; skip by default.
                        new_features.append(feature)
                        continue
                    floor = feature["properties"]["hulop_height"]
                    if floor_ok(floor):
                        feature["geometry"]["coordinates"][0] += d_lng
                        feature["geometry"]["coordinates"][1] += d_lat
                        feature["properties"]["lon"] += d_lng
                        feature["properties"]["lat"] += d_lat
                else:
                    if verbose:
                        print("feature="+str(feature))
                        print("unknown facil node")
                    # raise RuntimeError("unknown facil node")
            elif _id.startswith("EDITOR_area"):               
                if verbose:
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

    if args.write_diff:
        diff_output_dir = args.diff_output_dir or output_dir
        out_base = os.path.basename(filename_out)
        if out_base.endswith(".shifted.geojson"):
            diff_name = out_base[: -len(".shifted.geojson")] + ".diff.geojson"
        elif out_base.endswith(".geojson"):
            diff_name = out_base[: -len(".geojson")] + ".diff.geojson"
        else:
            diff_name = out_base + ".diff.geojson"
        diff_out = os.path.join(diff_output_dir, diff_name)

        diff_script = os.path.join(os.path.dirname(__file__), "diff_mapdata_geojson.py")
        cmd = [
            sys.executable,
            diff_script,
            "--orig",
            filename,
            "--shifted",
            filename_out,
            "-o",
            diff_out,
            "--delta_lines",
            "--summary",
        ]
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
