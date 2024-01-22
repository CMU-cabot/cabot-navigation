import sys
import ros

def onUpdate(*args):
    ros.info(str(args))
    return None
