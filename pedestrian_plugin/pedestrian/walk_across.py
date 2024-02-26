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
from pedestrian import state

stopped = False
count = 0
px = None
py = None


def onUpdate(**args):
    global stopped
    global count, px, py

    # parameters
    name = args['name']
    vel = args.get('velocity', 1.0)  # [m/s]
    collision_threshold = args.get('collision_threshold', 0.5)  # [m]
    decel_distance = args.get('decel_distance', 0.0)  # [m]
    pause_distance = args.get('pause_distance', 0.0)  # [m]
    stop_distance = args.get('stop_distance', 0.0)  # [m]
    radius = args.get('radius', 0.4)  # [m]
    args['radius'] = radius

    # variables
    x = args['x']
    y = args['y']
    yaw = args['yaw']
    dt = args['dt']

    # wait until robot pose is available
    if 'robot' not in args:
        return args

    rx = args['robot']['x']
    ry = args['robot']['y']

    # check collision
    dx = rx - x
    dy = ry - y
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < collision_threshold:
        ros.info(f"collision {name} {dist}")
        ros.collision(name, dist)

    # apply decel & pause_distance (temporary slow down)
    if dist < pause_distance:
        vel = 0.0
    elif dist < decel_distance:
        vel = (dist - pause_distance) / (decel_distance - pause_distance) * vel

    # apply stop_distance (permanent stop)
    if dist <= stop_distance:
        stopped = True
    if stopped:
        vel = 0.0

    # wait until the robot starts to move
    if px is not None and py is not None:
        dx = rx - px
        dy = ry - py
        dist = math.sqrt(dx*dx+dy*dy)
        if dist > 0.01 or count > 0:
            count = 10
        else:
            vel = 0
        count -= 1
    px = rx
    py = ry

    # update actor variables
    args['x'] += vel * dt * math.cos(yaw)
    args['y'] += vel * dt * math.sin(yaw)
    args['yaw'] = yaw

    # save state
    state.state[name] = args

    # ros.info(f"{args} {px} {py} {dx}")
    return args
