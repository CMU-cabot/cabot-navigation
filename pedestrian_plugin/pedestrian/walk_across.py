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
initial_time = None
count = 0
px = None
py = None
progress = 0  # variable to notify the progress of movement in each repeat cycle


def onUpdate(**args):
    global stopped
    global initial_time
    global count, px, py
    global progress

    # parameters
    name = args['name']
    init_x = args['init_x']
    init_y = args['init_y']
    goal_x = args.get('goal_x', None)
    goal_y = args.get('goal_y', None)
    vel = args.get('velocity', 1.0)  # [m/s]
    start_after = args.get('start_after', 0.0)  # [sec]
    collision_threshold = args.get('collision_threshold', 0.5)  # [m]
    decel_distance = args.get('decel_distance', 0.0)  # [m]
    pause_distance = args.get('pause_distance', 0.0)  # [m]
    stop_distance = args.get('stop_distance', 0.0)  # [m]
    repeat = args.get('repeat', None)  # If defined, repeat after the goal arrival.
    goal_radius = args.get('goal_raduis', 0.5)
    radius = args.get('radius', 0.4)  # [m] person radius used in the pedestrian plugin
    args['radius'] = radius  # module -> plugin

    # plugin variables
    x = args['x']
    y = args['y']
    yaw = args['yaw']
    dt = args['dt']
    t = args['time']

    # wait until robot pose is available
    if 'robot' not in args:
        return args

    if initial_time is None:
        initial_time = t

    # plugin variables
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
            initial_time = t  # reset this variable to enable start_after after the robot starts to move
        count -= 1
    px = rx
    py = ry

    # update actor variables
    if t - initial_time >= start_after:
        args['x'] += vel * dt * math.cos(yaw)
        args['y'] += vel * dt * math.sin(yaw)
        args['yaw'] = yaw
        progress += 1
        args['progress'] = progress

    # apply repeat
    if repeat is not None and goal_x is not None and goal_y is not None:
        dist_goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
        if dist_goal <= goal_radius:
            args['x'] = init_x
            args['y'] = init_y
            args['yaw'] = yaw  # keep the same direction
            args['progress'] = 0  # notify the start of repeat to the plugin

    # save state
    state.state[name] = args

    # ros.info(f"{args} {px} {py} {dx}")
    return args
