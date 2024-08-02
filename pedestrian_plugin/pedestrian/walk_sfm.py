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
import pysocialforce
import pysocialforce.scene
import numpy as np
from pedestrian import state
from pathlib import Path

# global state
sim = pysocialforce.Simulator(
    np.array([[]]),
    config_file=Path(__file__).resolve().parent.joinpath("sfm.toml"))
indicies = {}  # map from actor name to simulation state index
pRobot = None  # previous robot state

# debug
count = 0
initialized = False


def onUpdate(**args):
    global count, pRobot, initialized
    if not initialized:
        state.state.clear()
        initialized = True

    # parameter
    collision_threshold = args['collision_threshold'] if 'collision_threshold' in args else 0.5
    n_actors = args.get("n_actors", 10)
    goal_x = args.get('goal_x', None)
    goal_y = args.get('goal_y', None)
    min_x = args.get('min_x', -np.inf)
    max_x = args.get('max_x', np.inf)
    min_y = args.get('min_y', -np.inf)
    max_y = args.get('max_y', np.inf)
    radius = args.get('radius', 0.4)  # [m]

    args['goal_x'] = goal_x
    args['goal_y'] = goal_y
    args['radius'] = radius

    name = args['name']
    dt = args['dt']
    robot = args['robot'] if 'robot' in args else None

    if robot:
        rx = robot['x']
        ry = robot['y']
        x = args['x']
        y = args['y']
        dx = rx - x
        dy = ry - y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < collision_threshold:
            ros.collision(name, dist)

    # here will be executed for all actors
    # but wait until all actors state is available
    if len(state.state.keys()) == n_actors:
        # build state array for simulation
        temp = []
        index = 0
        for key in sorted(state.state.keys()):
            st = state.state[key]
            x = st['x']
            y = st['y']
            yaw = st['yaw']
            vel = args['velocity']
            vx = vel*math.cos(yaw)
            vy = vel*math.sin(yaw)
            gx = x + math.cos(yaw) if st['goal_x'] is None else st['goal_x']
            gy = y + math.sin(yaw) if st['goal_y'] is None else st['goal_y']
            temp.append([x, y, vx, vy, gx, gy])
            indicies[key] = index
            index += 1
        # add robot state
        if pRobot:
            rx = robot['x']
            ry = robot['y']
            drx = rx - pRobot['x']
            dry = ry - pRobot['y']
            rv = math.sqrt(drx*drx + dry*dry) / dt
            ryaw = robot['yaw']
            rvx = rv * math.cos(ryaw)
            rvy = rv * math.sin(ryaw)
            grx = robot['x'] + math.cos(yaw)
            gry = robot['y'] + math.sin(yaw)
            temp.append([rx, ry, rvx, rvy, grx, gry])

        # here needs to be hacked a bit
        sim.peds = pysocialforce.scene.PedState(np.array(temp), None, sim.scene_config)
        sim.forces = sim.make_forces(sim.config)
        sim.step(1)

        # debug outpu
        if count == 0:
            ros.info(f"{robot}")
            ros.info(f"{sim.config}")
            ros.info(f"{sim.peds.state}")
            count = 1

        # clear the state to ensure the simulation is executed only once in a cycle
        state.state.clear()
        pRobot = robot

    if name in indicies:
        index = indicies[name]
        st = sim.peds.state[index]
        x = st[0]
        y = st[1]
        vx = st[2]
        vy = st[3]
        v = math.sqrt(vx*vx+vy*vy)
        yaw = math.atan2(vy, vx)

        # bound in (min_x, min_y) x (max_x, max_y) region
        if max_x < x and abs(yaw) < math.pi/2:
            yaw = yaw+math.pi
        if x < min_x and math.pi/2 < abs(yaw):
            yaw = yaw-math.pi

        if max_y < y and 0 < yaw:
            yaw = -yaw
        if y < min_y and yaw < 0:
            yaw = -yaw

        if math.pi < yaw:
            yaw = yaw - math.pi*2
        if yaw < -math.pi:
            yaw = yaw + math.pi*2

        args['x'] = x
        args['y'] = y
        args['yaw'] = yaw
        args['velocity'] = v
        del indicies[name]

    # store the current state
    state.state[name] = args
    return args
