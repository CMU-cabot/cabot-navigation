import sys
import ros

count = 0
def onUpdate(*args):
    global count
    count +=1
    r = (0.001*count, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001*count)
    ros.info(f"{str(args)}")
    return r