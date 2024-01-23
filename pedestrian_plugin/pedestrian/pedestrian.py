import sys
import ros
import math

count = 0
def onUpdate(**args):
    global count
    count +=1
    if count < 100:
        ros.info(f"{count} {args}")
    vel = args['velocity']
    dt = args['dt']
    yaw = args['yaw'] + dt

    args['x'] += vel * dt * math.cos(yaw)
    args['y'] += vel * dt * math.sin(yaw)
    args['yaw'] = yaw
    return args
