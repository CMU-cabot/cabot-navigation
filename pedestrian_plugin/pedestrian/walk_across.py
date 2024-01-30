import sys
import ros
import math
from pedestrian import state

count=0
px=None
py=None

def onUpdate(**args):
    global count, px, py

    name = args['name']
    vel = args['velocity'] if 'velocity' in args else 1.0
    dt = args['dt']
    yaw = args['yaw']
    threshold = args['collision_threshold'] if 'collision_threshold' in args else 0.5

    rx = args['robot']['x']
    ry = args['robot']['y']
    x = args['x']
    y = args['y']
    dx = rx - x
    dy = ry - y
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < threshold:
        ros.info(f"collision {name} {dist}")
        ros.collision(name, dist)

    dx = 0
    if px is not None and py is not None:
        dx = rx - px
        dy = ry - py
        dist = math.sqrt(dx*dx+dy*dy)
        if dist > 0.01 or count > 0:
            count = 100
        else:
            vel = 0
        count-=1
    px = rx
    py = ry

    args['x'] += vel * dt * math.cos(yaw)
    args['y'] += vel * dt * math.sin(yaw)
    args['yaw'] = yaw
    name = args['name']
    state.state[name] = args

    # ros.info(f"{args} {px} {py} {dx}")
    return args
