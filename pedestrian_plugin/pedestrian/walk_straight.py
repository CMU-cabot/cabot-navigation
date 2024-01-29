import sys
import ros
import math
from pedestrian import state

def onUpdate(**args):
    vel = args['velocity'] if 'velocity' in args else 1.0
    dt = args['dt']
    yaw = args['yaw']

    #if 'robot' not in args:
    #    return args

    #rx = args['robot']['x']
    #ry = args['robot']['y']
    x = args['x']
    y = args['y']

    #dx = rx - x
    #dy = ry - y

    #dist = math.sqrt(dx*dx+dy*dy)
    #vel = min(max(0, dist-1.5), vel)

    args['x'] += vel * dt * math.cos(yaw)
    args['y'] += vel * dt * math.sin(yaw)
    args['yaw'] = yaw
    name = args['name']
    state.state[name] = args

    #ros.info(f"{args}")

    return args
