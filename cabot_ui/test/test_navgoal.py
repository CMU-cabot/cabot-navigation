#!/usr/bin/env python3

# Copyright (c) 2024  Carnegie Mellon University
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
"""Test navgoal module"""

import json
import unittest

import rclpy
import rclpy.node
from cabot_ui import geojson, geoutil, navgoal
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class TestNavgoal(unittest.TestCase):
    """Test class"""
    def setUp(self):
        pass

    def setUpClass():
        rclpy.init()
        TestNavgoal.node = rclpy.node.Node("test_node")
        CaBotRclpyUtil.initialize(TestNavgoal.node)

    def _prepare_data(self):
        import os
        dir_path = os.path.dirname(os.path.realpath(__file__))

        node_map = open(dir_path+"/data/node_map1.json")
        _ = geojson.Object.marshal_dict(json.load(node_map))
        features = open(dir_path+"/data/features1.json")
        _ = geojson.Object.marshal_list(json.load(features))

        route = open(dir_path+"/data/route2.json")
        self.route = geojson.Object.marshal_list(json.load(route))

        self.anc = geoutil.Anchor(lat=40.443259,
                                  lng=-79.945874,
                                  rotate=15.1)

        for obj in geojson.Object.get_all_objects():
            obj.reset()
            obj.update_anchor(self.anc)

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()
        return super().tearDown()

    def get_logger(self):
        return self.node.get_logger()

    def global_map_name(self):
        return "map"

    def test_doorgoal(self):
        self._prepare_data()
        door = geojson.Object.get_object_by_id("EDITOR_facil_1547762317242")
        link = geojson.Object.get_object_by_id("EDITOR_link_1553174320746")
        goal = navgoal.DoorGoal(self, link, self.anc, door)

        self.assertIsNotNone(goal)

        targetX = int(goal.x*12)
        for x in range(0, targetX):
            pose = geoutil.Pose(x=x/10.0, y=0.0, r=0.0)
            goal.check(pose)

        self.assertEqual(goal._is_completed, True)
