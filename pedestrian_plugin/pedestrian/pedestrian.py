import sys
import ros
import math

actors={}

count = 0
def onUpdate(**args):
    global count, actors
    count +=1
    vel = args['velocity'] if 'velocity' in args else 1.0
    dt = args['dt']
    yaw = args['yaw'] + dt

    args['x'] += vel * dt * math.cos(yaw)
    args['y'] += vel * dt * math.sin(yaw)
    args['yaw'] = yaw

    name = args["name"]
    if name:
        if name not in actors:
            actors[name] = {}
        for k, v in args.items():
            actors[name][k] = v
    if count < 100:
        ros.info(f"{name}: {args}")

    return args
