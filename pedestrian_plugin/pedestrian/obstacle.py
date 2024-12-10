# ******************************************************************************
#  Copyright (c) 2024  Carnegie Mellon University and Miraikan
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# ******************************************************************************

import ros
import math
from pedestrian.check_polygon_collision import isCircleCollidingWithPolygon

count = 0


def calc2DPointRotZ(x, y, yaw):
    x_rotated = x*math.cos(yaw) - y*math.sin(yaw)
    y_rotated = x*math.sin(yaw) + y*math.cos(yaw)
    return {'x': x_rotated, 'y': y_rotated}


def onUpdate(**args):
    global count
    name = args['name'] if 'name' in args else "no name"
    if count == 0:
        ros.info(f"obstacle {name} is spawned")
    count = 1
    robot = args['robot'] if 'robot' in args else None

    if robot:
        rx = robot['x']
        ry = robot['y']
        x = args['x']
        y = args['y']
        obstacle_width = args['obstacle_width']
        obstacle_height = args['obstacle_height']
        obstacle_yaw = args['yaw']
        circle_center_point = {'x': rx-x, 'y': ry-y}
        robot_radius = args['robot_radius']
        polygon_vertex_points = [
            calc2DPointRotZ(-obstacle_width/2., -obstacle_height/2., obstacle_yaw),
            calc2DPointRotZ(+obstacle_width/2., -obstacle_height/2., obstacle_yaw),
            calc2DPointRotZ(+obstacle_width/2., +obstacle_height/2., obstacle_yaw),
            calc2DPointRotZ(-obstacle_width/2., +obstacle_height/2., obstacle_yaw)
            ]
        is_collision_detected = isCircleCollidingWithPolygon(circle_center_point, robot_radius, polygon_vertex_points)

        dx = rx - x
        dy = ry - y
        dist = math.sqrt(dx * dx + dy * dy)
        ros.info(f"[{is_collision_detected=}]: {rx=}, {ry=}, {x=}, {y=}, {robot_radius=}, {obstacle_width=}, {obstacle_height=}, {obstacle_yaw=}")
        ros.info(f"                         : {obstacle_width=}, {obstacle_height=}")
        for i in range(len(polygon_vertex_points)):
            ros.info(f"[{is_collision_detected=}]: p{i}: ({polygon_vertex_points[i]['x']},{polygon_vertex_points[i]['y']})")

        if is_collision_detected:
            ros.collision(name, dist)

    return args
