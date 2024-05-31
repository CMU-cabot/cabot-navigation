# Copyright (c) 2024  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import ros
import math
from pedestrian.check_polygon_collision import isCircleCollidingWithPolygon

count = 0


def onUpdate(**args):
    global count
    name = args['name'] if 'name' in args else "no name"
    if count == 0:
        ros.info(f"obstacle {name} is spawned")
    count = 1
    robot = args['robot'] if 'robot' in args else None

    if robot:
        circle_center_point = {'x':robot['x'],robot['y']}
        robot_radius = 3.0
        polygon_vertex_points = [\
            {'x':args['x']-0.5,'y':args['y']-0.5}, \
            {'x':args['x']+0.5,'y':args['y']-0.5}, \
            {'x':args['x']+0.5,'y':args['y']+0.5}, \
            {'x':args['x']-0.5,'y':args['y']+0.5}]
        is_collision_detected = isCircleCollidingWithPolygon(circle_center_point,robot_radius,polygon_vertex_points)

        rx = robot['x']
        ry = robot['y']
        x = args['x']
        y = args['y']
        dx = rx - x
        dy = ry - y
        dist = math.sqrt(dx * dx + dy * dy)
        ros.info(f"obstacle collision {rx=}, {ry=}, {x=}, {y=}, {dx=}, {dy=}, {dist=}")

        if is_collision_detected:
            ros.collision(name, dist)

    return args
