import sys
import ros
import math

def onUpdate(**args):
    vel = args['velocity'] if 'velocity' in args else 1.0
    dt = args['dt']
    yaw = args['yaw'] - dt/2

    rx = args['robot']['x']
    ry = args['robot']['y']
    x = args['x']
    y = args['y']

    dx = rx - x
    dy = ry - y

    if math.sqrt(dx*dx+dy*dy) < 2:
        vel = 0

    args['x'] += vel * dt * math.cos(yaw)
    args['y'] += vel * dt * math.sin(yaw)
    args['yaw'] = yaw

    #ros.info(f"{args}")

    return args
