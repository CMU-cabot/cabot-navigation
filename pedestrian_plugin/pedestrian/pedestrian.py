import sys

def onUpdate(*args):
    try:
        import rclpy.logging
        rclpy.logging.get_logger("pedestrian").info(str(args))
    except:
        import backtrace
        backtrace.print_exc()
        return "Error"
    return "Hello World"
