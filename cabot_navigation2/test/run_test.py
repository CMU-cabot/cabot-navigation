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
import re

from optparse import OptionParser
import rclpy
import rclpy.node
from rosidl_runtime_py import set_message_fields
from tf_transformations import quaternion_from_euler

from cabot_common.util import callee_name

from geometry_msgs.msg import PoseWithCovarianceStamped
from mf_localization_msgs.srv import StartLocalization, StopLocalization, MFSetInt
from gazebo_msgs.srv import SetEntityState

from pedestrian.manager import PedestrianManager
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity

# cabot_navigation2/test
from evaluator import Evaluator


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
            t = kwargs['timeout'] if 'timeout' in kwargs else t+5  # make sure not timeout if wait seconds is specified
            action_name = kwargs['action_name'] if 'action_name' in kwargs else function.__name__
            case = {'target': tester.test_func_name, 'action': action_name, 'done': False, 'success': False, 'error': None}
            test_action = {'uuid': str(uuid.uuid4())}

            logger.debug(f"calling {function} {case} {test_action} - {args} {kwargs}")
            args = args + (case,)
            test_action.update(kwargs)
            result = function(*args, test_action)
            start = time.time()

            while not case['done'] and time.time() - start < t:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not case['done']:
                case['success'] = False
                case['error'] = f"Timeout ({t} seconds)"
                # logger.error("Timeout")
                # continue other test
            else:
                case['success'] = True
            logger.debug(f"finish: {case}")
            tester.register_action_result(case['target'], case)
            return result
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
        self.stop_localization_client = self.node.create_client(StopLocalization, '/stop_localization')
        self.start_localization_client = self.node.create_client(StartLocalization, '/start_localization')
        self.set_current_floor_client = self.node.create_client(MFSetInt, '/set_current_floor')
        self.initialpose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.set_entity_state_client = self.node.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.test_func_name = None
        self.result = {}
        # evaluation
        self.evaluator = None

    def set_evaluator(self, evaluator):
        self.evaluator = evaluator

    def test(self, module, test_pat, wait_ready=False):
        functions = [func for func in dir(module) if inspect.isfunction(getattr(module, func))]

        # prepare the test
        self.default_config()
        for func in ['config', 'checks', 'wait_ready']:
            if not wait_ready and func == 'wait_ready':
                functions.remove(func)
                continue
            if func in functions:
                logger.info(f"Calling {func}")
                self.test_func_name = func
                getattr(module, func)(self)
                functions.remove(func)

        if wait_ready:
            success = self.print_result(self.result, func)
            if not success:
                sys.exit(1)

        for func in sorted(functions):
            if func.startswith("_"):
                continue
            if test_pat and not test_pat.match(func):
                continue
            logger.info(f"Testing {func}")
            self.test_func_name = func
            getattr(module, func)(self)
            self.stop_evaluation()  # automatically stop metric evaluation
            success = self.print_result(self.result, func)
            self.register_action_result(func, self.result)
            self.cancel_subscription(func)

        logger.info("Done all test")

        allSuccess = True
        for key in sorted(self.result.keys()):
            allSuccess = allSuccess and success

        if allSuccess:
            sys.exit(0)
        else:
            sys.exit(1)

    def print_result(self, result, key):
        tfResult = result[key]
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
        return success

    def register_action_result(self, target_function_name, case):
        if target_function_name not in self.result:
            self.result[target_function_name] = []
        self.result[target_function_name].append(case)

    def add_subscription(self, case, sub):
        if 'target' in case:
            target = case['target']
        else:
            raise RuntimeError(f"no target in {case}")
        if 'action' in case:
            action = case['action']
        else:
            raise RuntimeError(f"no action in {case}")
        if target not in self.subscriptions:
            self.subscriptions[target] = {}
        self.subscriptions[target][action] = sub

    def cancel_subscription(self, case):
        if isinstance(case, str):
            target = case
            action = None
        else:
            if 'target' in case:
                target = case['target']
            else:
                raise RuntimeError(f"no target in {case}")
            if 'action' in case:
                action = case['action']
            else:
                raise RuntimeError(f"no action in {case}")
        if target in self.subscriptions:
            for key, sub in self.subscriptions[target].items():
                if action is None or key == action:
                    self.node.destroy_subscription(sub)
            if action:
                if action in self.subscriptions[target]:
                    del self.subscriptions[target][action]
            else:
                del self.subscriptions[target]

    def default_config(self):
        self.config = {
            'init_x': 0.0,
            'init_y': 0.0,
            'init_z': 0.0,
            'init_a': 0.0,
            'init_floor': 0
        }

    def info(self, text):
        logger.info(text)

    # evaluation
    def set_evaluation_parameters(self, **kwargs):
        """
        Set parameters used for computing metrics.

        Parameters are defined as EvaluationParameter dataclass in evaluator module

        Parameters
        ----------
        metrics: Optional[list] = []
            List of metric functions to be computed. The callable functions are defined in evaluation_metrics.py

        robot_radius: Optional[float] = None
            The robot radius used to detect collisions in the metric computation.
            If not defined, the default value (0.45) defined in the pedestrian plugin is used.

        """
        self.evaluator.set_evaluation_parameters(**kwargs)

    def start_evaluation(self):
        """
        Start computing the metrics.

        This method should be called when ready to start the navigation
        """
        self.evaluator.start()

    def stop_evaluation(self):
        """
        Stop comuting the metrics.

        It is usually not necessary to call this method because it is automatically called when the test ends.
        This method can be used when the user intentionally stops the metric computation
        """
        self.evaluator.stop()

    # shorthand functions
    def check_collision(self, **kwargs):
        return self.check_topic_error(**dict(
            dict(
                action_name='check_collision',
                topic="/collision",
                topic_type="pedestrian_plugin_msgs/msg/Collision",
                condition="True"
            ),
            **kwargs)
        )

    def goto_node(self, node_id, **kwargs):
        self.pub_topic(**dict(
            dict(
                action_name=f'goto_node({node_id})',
                topic='/cabot/event',
                topic_type='std_msgs/msg/String',
                message=f"data: 'navigation;destination;{node_id}'"
            ),
            **kwargs)
        )

    def floor_change(self, diff, **kwargs):
        self.call_service(**dict(
            dict(
                action_name=f'floor_chage({diff})',
                service='/floor_change',
                service_type='mf_localization_msgs.srv/FloorChange',
                request=f"diff: {diff}"
            ),
            **kwargs)
        )

    def cancel_navigation(self, **kwargs):
        self.pub_topic(**dict(
            dict(
                action_name='cancel_navigation',
                topic='/cabot/event',
                topic_type='std_msgs/msg/String',
                message="data: 'navigation;cancel'"
            ),
            **kwargs)
        )

    def button_down(self, button, **kwargs):
        self.pub_topic(**dict(
            dict(
                action_name=f'button_down({button})',
                topic='/cabot/event',
                topic_type='std_msgs/msg/String',
                message=f"data: 'button_down_{button}'"
            ),
            **kwargs)
        )

    def wait_goal(self, goalName, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name=f'wait_goal({goalName})',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition=f"msg.category=='cabot/navigation' and msg.text=='goal_completed' and msg.memo=='{goalName}'",
                timeout=60
            ),
            **kwargs)
        )

    def wait_navigation_completed(self, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name='wait_navigation_completed',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition="msg.category=='cabot/navigation' and msg.text=='completed'",
                timeout=60
            ),
            **kwargs)
        )

    def wait_navigation_arrived(self, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name='wait_navigation_arrived',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition="msg.category=='cabot/navigation' and msg.text=='navigation' and msg.memo=='arrived'",
                timeout=60
            ),
            **kwargs)
        )

    def wait_turn_towards(self, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name='wait_turn_towards',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition="msg.category=='cabot/navigation' and msg.text=='turn_towards'",
                timeout=60
            ),
            **kwargs)
        )

    def check_navigation_arrived(self, **kwargs):
        self.check_topic(**dict(
            dict(
                action_name='check_navigation_arrived',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition="msg.category=='cabot/navigation' and msg.text=='navigation' and msg.memo=='arrived'",
                timeout=60
            ),
            **kwargs)
        )

    def check_turn_towards(self, **kwargs):
        self.check_topic(**dict(
            dict(
                action_name='check_turn_towards',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition="msg.category=='cabot/navigation' and msg.text=='turn_towards'",
                timeout=60
            ),
            **kwargs)
        )

    def wait_for(self, seconds, **kwargs):
        self.wait(**dict(
            dict(
                action_name=f'wait_for({seconds})',
                seconds=seconds
            ),
            **kwargs)
        )

    def wait_ready(self, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name='wait_ready',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition="msg.category=='cabot/interface' and msg.text=='status' and msg.memo=='ready'",
                timeout=60
            ),
            **kwargs)
        )

    def wait_localization_started(self, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name='wait_localization_started',
                topic='/localize_status',
                topic_type='mf_localization_msgs/msg/MFLocalizeStatus',
                condition='msg.status==1',
                timeout=60
            ),
            **kwargs)
        )

    @wait_test()
    def clean_door(self, case, test_action):
        uuid = test_action['uuid']

        def done_callback(future):
            case['done'] = True
        future = ElevatorDoorSimulator.instance().clean(callback=done_callback)
        if not future:
            return
        self.futures[uuid] = future

    @wait_test()
    def spawn_door(self, case, test_action):
        uuid = test_action['uuid']
        self.futures[uuid] = ElevatorDoorSimulator.instance().spawn_door(**test_action)

        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
        self.futures[uuid].add_done_callback(done_callback)

    @wait_test()
    def delete_door(self, case, test_action):
        uuid = test_action['uuid']
        self.futures[uuid] = ElevatorDoorSimulator.instance().delete_door(**test_action)

        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
        self.futures[uuid].add_done_callback(done_callback)

    # actual task needs to be waited
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

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    logger.error(f"check_topic_error: condition ({condition}) matched\n{msg}")
                    case['success'] = False
                    case['error'] = f"condition {condition} matched\n{msg}"
                    self.cancel_subscription(case)
            except:  # noqa: #722
                logger.error(traceback.format_exc())

        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.add_subscription(case, sub)
        case['done'] = True

        def cancel_func():
            self.cancel_subscription(case)
        return cancel_func

    @wait_test()
    def check_topic(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    logger.debug(f"success {condition}")
                    case['success'] = True
                    self.cancel_subscription(case)
            except:  # noqa: #722
                logger.error(traceback.format_exc())

        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.add_subscription(case, sub)
        case['done'] = True
        case['success'] = False

    @wait_test()
    def reset_position(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        uuid = test_action['uuid']
        topic = '/localize_status'
        topic_type = import_class('mf_localization_msgs/msg/MFLocalizeStatus')
        condition = "msg.status==msg.TRACKING"

        # use true pose as initial pose guess or not
        use_initialpose = test_action.get("use_initialpose", True)

        # request to stop localization
        request = StopLocalization.Request()
        self.futures[uuid] = self.stop_localization_client.call_async(request)

        def done_stop_localization_callback(future):
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
            self.futures[uuid] = self.set_entity_state_client.call_async(request)

            def done_set_entity_state_callback(future):
                # define callback to check localize status to be tracking
                def topic_callback(msg):
                    try:
                        context = {'msg': msg}
                        exec(f"result=({condition})", context)
                        if context['result']:
                            case['done'] = True
                            self.cancel_subscription(case)
                            time.sleep(2)
                    except:  # noqa: #722
                        logger.error(traceback.format_exc())

                if use_initialpose:
                    # request to set current floor
                    request = MFSetInt.Request()
                    init_floor = test_action['floor'] if 'floor' in test_action else self.config['init_floor']
                    request.data = int(init_floor)
                    self.futures[uuid] = self.set_current_floor_client.call_async(request)

                    def done_set_current_floor_callback(future):
                        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
                        self.add_subscription(case, sub)

                        time.sleep(1)
                        # publish initialpose to start localization with initial pose guess
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

                    self.futures[uuid].add_done_callback(done_set_current_floor_callback)

                else:
                    # request to start localization
                    request = StartLocalization.Request()
                    self.futures[uuid] = self.start_localization_client.call_async(request)

                    def done_start_localization_callback(future):
                        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
                        self.add_subscription(case, sub)

                    self.futures[uuid].add_done_callback(done_start_localization_callback)

            self.futures[uuid].add_done_callback(done_set_entity_state_callback)

        self.futures[uuid].add_done_callback(done_stop_localization_callback)

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

        def topic_callback(msg):
            try:
                context = {'msg': msg}
                exec(f"result=({condition})", context)
                if context['result']:
                    case['done'] = True
                    self.cancel_subscription(case)
            except:  # noqa: #722
                logger.error(traceback.format_exc())
        sub = self.node.create_subscription(topic_type, topic, topic_callback, 10)
        self.add_subscription(case, sub)

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
    def call_service(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        service = test_action['service']
        service_type_str = test_action['service_type']
        service_type = import_class(service_type_str)
        request = test_action['request']
        request_type = service_type.Request
        uuid = test_action['uuid']

        req = request_type()
        data = yaml.safe_load(request)
        set_message_fields(req, data)

        srv = self.node.create_client(service_type, service)
        self.futures[uuid] = srv.call_async(req)

        def done_callback(future):
            case['done'] = True

        self.futures[uuid].add_done_callback(done_callback)

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


class ElevatorDoorSimulator:
    _instance = None

    @classmethod
    def instance(cls):
        if not ElevatorDoorSimulator._instance:
            ElevatorDoorSimulator._instance = ElevatorDoorSimulator()
        return ElevatorDoorSimulator._instance

    def __init__(self):
        self.spawn_entity_client = node.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = node.create_client(DeleteEntity, '/delete_entity')
        self.remaining = []

    def clean(self, callback):
        if self.remaining:
            future = self.delete_door(name=self.ramaining[0])

            def done_callback(future):
                self.clean(callback)
            future.add_done_callback(done_callback)
        else:
            callback("Done")

    def delete_door(self, **kwargs):
        name = kwargs['name']
        request = DeleteEntity.Request()
        request.name = name
        future = self.delete_entity_client.call_async(request)

        def callback(future):
            self.remaining.remove(name)
        future.add_done_callback(callback)
        return future

    def spawn_door(self, **kwargs):
        name = kwargs['name']
        x = kwargs['x']
        y = kwargs['y']
        z = kwargs['z']
        yaw = kwargs['yaw']

        door_xml = f"""
<?xml version="1.0" ?>
<sdf version="1.6">
    <model name="{name}">
        <static>true</static>
        <link name="{name}-link">
            <pose>{x} {y} {z+1} 0 {math.pi/2} {yaw}</pose>
            <visual name="{name}-visual">
                <geometry>
                    <box>
                        <size>2 2 0.01</size>
                    </box>
                </geometry>
            </visual>
            <collision name="{name}-collision">
                <geometry>
                    <box>
                        <size>2 2 0.01</size>
                    </box>
                </geometry>
            </collision>
        </link>
    </model>
</sdf>
"""
        logging.debug(door_xml)
        request = SpawnEntity.Request()
        request.name = name
        request.xml = door_xml
        request.reference_frame = "world"
        future = self.spawn_entity_client.call_async(request)

        def callback(future):
            self.remaining.append(name)
        future.add_done_callback(callback)
        return future


class LogColors:
    DEBUG = '\033[94m'       # Blue
    INFO = '\033[92m'        # Green
    WARNING = '\033[93m'     # Yellow
    ERROR = '\033[91m'       # Red
    CRITICAL = '\033[1;91m'  # Bold Red
    RESET = '\033[0m'        # Reset


# Custom formatter
class ColorFormatter(logging.Formatter):
    format = "%(asctime)s.%(msecs)03d %(levelname)s: %(message)s"

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


class ROS2LogHandler(logging.Handler):
    """A logging handler that forwards Python logging messages to ROS2 logging."""

    def __init__(self, node: rclpy.node.Node):
        super().__init__()
        self.node = node

    def emit(self, record):
        """Override emit to forward the log message to ROS2 logging."""
        msg = self.format(record)
        level = record.levelno
        if level >= logging.CRITICAL:
            self.node.get_logger().fatal(msg)
        elif level >= logging.ERROR:
            self.node.get_logger().error(msg)
        elif level >= logging.WARNING:
            self.node.get_logger().warn(msg)
        elif level >= logging.INFO:
            self.node.get_logger().info(msg)
        else:  # DEBUG and NOTSET
            self.node.get_logger().debug(msg)


def main():
    global node, manager, logger
    parser = OptionParser(usage="""
    Example
    {0} -m <module name>    # run test module
    {0} -f <func name pat>  # run only func that mataches the pattern
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

    ros2Handler = ROS2LogHandler(node)
    logger.addHandler(ros2Handler)

    evaluator = Evaluator(node)
    evaluator.set_logger(logger)

    tester = Tester(node)
    tester.set_evaluator(evaluator)
    mod = importlib.import_module(options.module)
    func_pat = None
    if options.func:
        try:
            func_pat = re.compile(options.func)
            logger.info(f"test func = {options.func}")
        except re.error as error:
            logger.error(error)
            return
    tester.test(mod, func_pat, wait_ready=options.wait_ready)


if __name__ == "__main__":
    main()
