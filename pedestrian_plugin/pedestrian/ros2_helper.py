import rclpy
import rclpy.executors
import traceback
import threading
import sys

try:
    rclpy.init()
    shared_executor = rclpy.executors.SingleThreadedExecutor()
    def loop():
        shared_executor.spin()
    thread = threading.Thread(target=loop)
    thread.start()
except:
    print(traceback.format_exc(), file=sys.stderr)

names = {}
def add_node(name, node):
    shared_executor.add_node(node)
    names[name] = True

def exists(name):
    return name in names
