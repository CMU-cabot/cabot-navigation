#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2024  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
###############################################################################

import importlib
import inspect
import sys
import math
import time
import traceback
import uuid
import yaml
import logging

from optparse import OptionParser
import rclpy
import rclpy.node
from rosidl_runtime_py import set_message_fields
from tf_transformations import quaternion_from_euler

from cabot_common.util import callee_name

from geometry_msgs.msg import PoseWithCovarianceStamped
from mf_localization_msgs.srv import RestartLocalization
from gazebo_msgs.srv import SetEntityState

from pedestrian.manager import PedestrianManager


def import_class(input_str):
    # Split the input string and form module and class strings
    module_str, class_str = input_str.rsplit('/', 1)
    module_str = module_str.replace('/', '.')
    # Import the module dynamically
    module = importlib.import_module(module_str)
    return getattr(module, class_str)


# global
node = None
manager = None
logger = None


# decorator of test actions
def wait_test(timeout=60):
    def outer_wrap(function):
        def wrap(*args, **kwargs):
            tester = args[0]

            t = kwargs['seconds'] if 'seconds' in kwargs else timeout
            t = kwargs['timeout'] if 'timeout' in kwargs else t
            action_name = function.__name__
            case = {'target': tester.test_func_name, 'action': action_name, 'done': False, 'success': False, 'error': None}
            test_action = {'uuid': str(uuid.uuid4())}

            # logger.debug(f"calling {function} {case} {test_action} - {args} {kwargs}")
            args = args + (case,)
            test_action.update(kwargs)
            function(*args, test_action)
            start = time.time()
            while not case['done'] and time.time() - start < t:
                rclpy.spin_once(node, timeout_sec=1)
            if not case['done']:
                case['success'] = False
                case['error'] = "Timeout"
                # logger.error("Timeout")
                # continue other test
            else:
                case['success'] = True
            tester.register_action_result(case['target'], case)
        return wrap
    return outer_wrap


class Tester:
    def __init__(self, node):
        self.node = node
        self.done = False
        self.alive = True
        self.config = {}
        self.subscriptions = {}
        self.futures = {}
        self.timers = {}
        self.actor_count = 0
        self.restart_localization_client = self.node.create_client(RestartLocalization, '/restart_localization')
        self.initialpose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.set_entity_state_client = self.node.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.test_func_name = None
        self.result = {}

    def test(self, module, specific_test, wait_ready=False):
        functions = [func for func in dir(module) if inspect.isfunction(getattr(module, func))]

        # prepare the test
        self.default_config()
        for func in ['config', 'checks', 'wait_ready']:
            if not wait_ready and func == 'wait_ready':
                functions.remove(func)
                continue
            if func in functions:
                logger.debug(f"Calling {func}")
                self.test_func_name = func
                getattr(module, func)(self)
                functions.remove(func)

        if specific_test and specific_test in functions:
            logger.debug(f"Testing {specific_test}")
            self.test_func_name = specific_test
            getattr(module, specific_test)(self)
        else:
            for func in sorted(functions):
                if func.startswith("_"):
                    continue
                logger.debug(f"Testing {func}")
                self.test_func_name = func
                getattr(module, func)(self)

        logger.debug("Done all test")

        for key in sorted(self.result.keys()):
            tfResult = self.result[key]
            success = True
            for aResult in tfResult:
                success = success and aResult['success']
            if success:
                logger.info(f"{key}: Success")
            else:
                logger.error(f"{key}: Failure")
            for aResult in tfResult:
                success = aResult['success']
                action = aResult['action']
                if success:
                    logger.info(f" - {action}: Success")
                else:
                    logger.error(f" - {action}: Failure")
                    logger.error(f"{aResult['error']}")
            logger.info("--------------------------")

        sys.exit(0)

    def register_action_result(self, target_function_name, case):
        if target_function_name not in self.result:
            self.result[target_function_name] = []
        self.result[target_function_name].append(case)

    def default_config(self):
        self.config = {
            'init_x': 0.0,
            'init_y': 0.0,
            'init_z': 0.0,
            'init_a': 0.0,
        }

    def info(self, text):
        logger.info(text)

    @wait_test()
    def init_manager(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
        manager.init(callback=done_callback)

    @wait_test(1)
    def check_topic_error(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']
        uuid = test_action['uuid']

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    logger.error(f"check_topic_error: condition ({condition}) matched\n{msg}")
                    case['success'] = False
                    case['error'] = f"condition {condition} matched\n{msg}"
                    sub = self.subscriptions[uuid]
                    self.node.destroy_subscription(sub)
            except:
                logger.error(traceback.format_exc())

        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.subscriptions[uuid] = sub
        case['done'] = True

    @wait_test()
    def wait_ready(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        uuid = test_action['uuid']
        topic = '/cabot/activity_log'
        topic_type = import_class('cabot_msgs/msg/Log')
        condition = "msg.category=='cabot/interface' and msg.text=='status' and msg.memo=='ready'"

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    case['done'] = True
                    sub = self.subscriptions[uuid]
                    self.node.destroy_subscription(sub)
            except:
                logger.error(traceback.format_exc())
        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.subscriptions[uuid] = sub

    @wait_test()
    def reset_position(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        uuid = test_action['uuid']
        topic = '/localize_status'
        topic_type = import_class('mf_localization_msgs/msg/MFLocalizeStatus')
        condition = "msg.status==msg.TRACKING"

        # change gazebo model position
        request = SetEntityState.Request()
        request.state.name = 'mobile_base'
        init_x = test_action['x'] if 'x' in test_action else self.config['init_x']
        init_y = test_action['y'] if 'y' in test_action else self.config['init_y']
        init_z = test_action['z'] if 'z' in test_action else self.config['init_z']
        init_a = test_action['a'] if 'a' in test_action else self.config['init_a']
        init_yaw = init_a / 180.0 * math.pi
        request.state.pose.position.x = float(init_x)
        request.state.pose.position.y = float(init_y)
        request.state.pose.position.z = float(init_z)
        q = quaternion_from_euler(0, 0, init_yaw)
        request.state.pose.orientation.x = q[0]
        request.state.pose.orientation.y = q[1]
        request.state.pose.orientation.z = q[2]
        request.state.pose.orientation.w = q[3]
        future = self.set_entity_state_client.call_async(request)
        self.futures[uuid] = future

        def done_callback(future):
            # request to restart localization
            request = RestartLocalization.Request()
            self.restart_localization_client.call_async(request)
            self.futures[uuid] = future

            def done_callback2(future):
                # check localize status to be tracking
                def topic_callback(msg):
                    try:
                        context = {'msg': msg}
                        exec(f"result=({condition})", context)
                        if context['result']:
                            case['done'] = True
                            sub = self.subscriptions[uuid]
                            self.node.destroy_subscription(sub)
                            time.sleep(2)
                    except:
                        logger.error(traceback.format_exc())
                sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
                self.subscriptions[uuid] = sub

                time.sleep(1)
                # publish initialpose for localization hint
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = "map"
                pose.pose.pose.position.x = float(init_x)
                pose.pose.pose.position.y = float(init_y)
                pose.pose.pose.position.z = float(init_z)
                pose.pose.pose.orientation.x = q[0]
                pose.pose.pose.orientation.y = q[1]
                pose.pose.pose.orientation.z = q[2]
                pose.pose.pose.orientation.w = q[3]
                self.initialpose_pub.publish(pose)

            future.add_done_callback(done_callback2)
        future.add_done_callback(done_callback)

    @wait_test()
    def setup_actors(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
        manager.update(
            actors=test_action['actors'],
            callback=done_callback)

    @wait_test()
    def wait_topic(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']
        uuid = test_action['uuid']

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    case['done'] = True
                    sub = self.subscriptions[uuid]
                    self.node.destroy_subscription(sub)
            except:
                logger.error(traceback.format_exc())
        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.subscriptions[uuid] = sub

    @wait_test()
    def pub_topic(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        message = test_action['message']

        msg = topic_type()
        data = yaml.safe_load(message)
        set_message_fields(msg, data)

        pub = self.node.create_publisher(topic_type, topic, 10)
        pub.publish(msg)
        self.node.destroy_publisher(pub)
        case['done'] = True

    @wait_test()
    def wait(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        seconds = test_action['seconds']
        uuid = test_action['uuid']

        def timer_callback():
            case['done'] = True
            timer = self.timers[uuid]
            timer.cancel()
            self.node.destroy_timer(timer)

        timer = self.node.create_timer(seconds, timer_callback)
        self.timers[uuid] = timer

    def terminate(self, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        sys.exit(0)


class LogColors:
    DEBUG = '\033[94m'  # Blue
    INFO = '\033[92m'   # Green
    WARNING = '\033[93m' # Yellow
    ERROR = '\033[91m'   # Red
    CRITICAL = '\033[1;91m'  # Bold Red
    RESET = '\033[0m'   # Reset

# Custom formatter
class ColorFormatter(logging.Formatter):
    format = "%(asctime)s.%(msecs)03d %(levelname)s: %(message)s"# (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: LogColors.DEBUG + format + LogColors.RESET,
        logging.INFO: LogColors.INFO + format + LogColors.RESET,
        logging.WARNING: LogColors.WARNING + format + LogColors.RESET,
        logging.ERROR: LogColors.ERROR + format + LogColors.RESET,
        logging.CRITICAL: LogColors.CRITICAL + format + LogColors.RESET
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


def main():
    global node, manager, logger
    parser = OptionParser(usage="""
    Example
    {0} -m <module name>    # run test module
    {0} -f <func name>      # run only func
    """.format(sys.argv[0]))

    parser.add_option('-m', '--module', type=str, help='test module name')
    parser.add_option('-f', '--func', type=str, help='test func name')
    parser.add_option('-w', '--wait-ready', action='store_true', help='wait ready')
    parser.add_option('-d', '--debug', action='store_true', help='debug print')

    (options, args) = parser.parse_args()

    if not options.module:
        parser.print_help()
        sys.exit(0)

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG if options.debug else logging.INFO)
    handler = logging.StreamHandler()
    handler.setFormatter(ColorFormatter())
    logger.addHandler(handler)

    rclpy.init()
    node = rclpy.node.Node("test_node")
    manager = PedestrianManager(node)

    tester = Tester(node)
    mod = importlib.import_module(options.module)
    tester.test(mod, options.func, wait_ready=options.wait_ready)


if __name__ == "__main__":
    main()
