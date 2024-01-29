import sys
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
count=0
initialized=False

def onUpdate(**args):
    global count, pRobot, initialized
    if not initialized:
        state.state.clear()
        initialized = True

    dt = args['dt']
    name = args['name']
    robot = args['robot'] if 'robot' in args else None

    # here will be executed for all actors
    # but wait until all actors state is available
    if len(state.state.keys()) == 10:
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
            gx = x + math.cos(yaw)
            gy = y + math.sin(yaw)
            temp.append([x, y, vx, vy, gx, gy])
            indicies[key] = index
            index+=1
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
            count=1

        # clear the state to ensure the simulation is executed only once in a cycle
        state.state.clear()
        pRobot = robot

    if name in indicies:
        index=indicies[name]
        st = sim.peds.state[index]
        x = st[0]
        y = st[1]
        vx = st[2]
        vy = st[3]
        v = math.sqrt(vx*vx+vy*vy)
        yaw = math.atan2(vy, vx)

        # bound in (-5, -5) (5, 5) region
        bound=5
        if bound < x and abs(yaw) < math.pi/2:
            yaw = yaw+math.pi
        if x < -bound and math.pi/2 < abs(yaw) :
            yaw = yaw-math.pi

        if bound < y and 0 < yaw:
            yaw = -yaw
        if y < -bound and yaw < 0:
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
