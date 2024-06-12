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
import pkgutil
import inspect
import sys
import math
import numpy
import time
import traceback
import uuid
import yaml
import logging
import re
from dataclasses import dataclass, fields
from optparse import OptionParser
import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosidl_runtime_py import set_message_fields
from tf_transformations import quaternion_from_euler

from cabot_common.util import callee_name
from people_msgs.msg import People, Person
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter, ParameterType
from mf_localization_msgs.srv import StartLocalization, StopLocalization, MFSetInt
from gazebo_msgs.srv import SetEntityState
from rcl_interfaces.srv import SetParameters

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
            case = {'target': tester.test_func_name, 'action': action_name, 'done': False, 'success': None, 'error': None}
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
                if case['success'] is not None:
                    if not case['success']:
                        logger.error(F"{case}")
                    else:
                        logger.debug(F"{case}")
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
        self.set_people_detection_params_client = self.node.create_client(
            SetParameters,
            '/gazebo/set_parameters'
        )
        self.test_func_name = None
        self.result = {}
        # evaluation
        self.evaluator = None

        # while not self.set_people_detection_params_client.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('service not available, waiting again...')

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

        allSuccess = True
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
            allSuccess = allSuccess and success

        logger.info("Done all test")

        if allSuccess:
            sys.exit(0)
        else:
            sys.exit(1)

    def print_result(self, result, key):
        tfResult = result[key]
        success = True
        for aResult in tfResult:
            if aResult['success'] is None:
                aResult['success'] = False
            success = success and aResult['success']
        if success:
            logger.info(f"{key}: Success")
        else:
            logger.error(f"{key}: Failure")
        for aResult in tfResult:
            success2 = aResult['success']
            action = aResult['action']
            if success2:
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

    # people detection
    def set_people_detection_range(self, **kwargs):
        params = []

        # occlusion_angle_range parameter
        if 'occlusion_ray_range' in kwargs:
            param = Parameter()
            param.name = 'pedestrian_plugin.occlusion_ray_range'
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.double_value = kwargs['occlusion_ray_range']
            params.append(param)

        # min_range parameter
        if 'min_range' in kwargs:
            param = Parameter()
            param.name = 'pedestrian_plugin.min_range'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = kwargs['min_range']
            params.append(param)

        # max_range parameter
        if 'max_range' in kwargs:
            param = Parameter()
            param.name = 'pedestrian_plugin.max_range'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = kwargs['max_range']
            params.append(param)

        # min_angle parameter
        if 'min_angle' in kwargs:
            param = Parameter()
            param.name = 'pedestrian_plugin.min_angle'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = kwargs['min_angle']
            params.append(param)

        # max_angle parameter
        if 'max_angle' in kwargs:
            param = Parameter()
            param.name = 'pedestrian_plugin.max_angle'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = kwargs['max_angle']
            params.append(param)

        # divider_distance_m parameter
        if 'divider_distance_m' in kwargs:
            param = Parameter()
            param.name = 'pedestrian_plugin.divider_distance_m'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = kwargs['divider_distance_m']
            params.append(param)

        # divider_angle_deg parameter
        if 'divider_angle_deg' in kwargs:
            param = Parameter()
            param.name = 'pedestrian_plugin.divider_angle_deg'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = kwargs['divider_angle_deg']
            params.append(param)

        request = SetParameters.Request()
        request.parameters = params

        logger.info("Set pedestrian_plugin params")

        future = self.set_people_detection_params_client.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            logger.info(f'Service call succeeded: {future.result()}')
        else:
            logger.error(f'Service call failed: {future.exception()}')

    # shorthand functions
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

    def check_position(self, **kwargs):
        x = kwargs['x'] if 'x' in kwargs else 0
        y = kwargs['y'] if 'y' in kwargs else 0
        tolerance = kwargs['tolerance'] if 'tolerance' in kwargs else 0.5
        floor = kwargs['floor'] if 'floor' in kwargs else 0
        self.wait_topic(**dict(
            dict(
                action_name=f'check_position ({x}, {y})[f={floor}] < {tolerance}',
                topic_type="cabot_msgs/msg/PoseLog",
                topic="/cabot/pose_log",
                condition=F"math.sqrt((msg.pose.position.x - {x})**2 + (msg.pose.position.y - {y})**2) < {tolerance} and msg.floor == {floor}",
                once=True
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

    def wait_for(self, seconds, **kwargs):
        self.wait(**dict(
            dict(
                action_name=f'wait_for({seconds})',
                seconds=seconds
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

    def wait_localization_started(self, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name='wait_localization_started',
                topic='/localize_status',
                topic_type='mf_localization_msgs/msg/MFLocalizeStatus',
                qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                condition='msg.status==1 or msg.status==2',
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

    # actual task needs to be waited
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
            case['success'] = True

        self.futures[uuid].add_done_callback(done_callback)

    @wait_test()
    def check_topic(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']

        def topic_callback(msg):
            try:
                context = {'msg': msg, 'math': math}
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
        case['success'] = None

    @wait_test(1)
    def check_topic_error(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']

        def topic_callback(msg):
            try:
                context = {'msg': msg, 'math': math}
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
        case['success'] = True

        def cancel_func():
            logger.debug(F"cancel {case}")
            self.cancel_subscription(case)
        return cancel_func

    @wait_test()
    def clean_door(self, case, test_action):
        uuid = test_action['uuid']

        def done_callback(future):
            case['done'] = True
            case['success'] = True
        future = ObstacleManager.instance().clean(callback=done_callback)
        if not future:
            return
        self.futures[uuid] = future

    @wait_test()
    def delete_actor(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")

        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
            case['success'] = True
        manager.delete(
            name=test_action['name'],
            callback=done_callback)

    @wait_test()
    def delete_door(self, case, test_action):
        uuid = test_action['uuid']
        self.futures[uuid] = ObstacleManager.instance().delete_door(**test_action)

        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
            case['success'] = True
        self.futures[uuid].add_done_callback(done_callback)

    @wait_test()
    def init_manager(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")

        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
            case['success'] = True
        manager.init(callback=done_callback)

    @wait_test()
    def pub_topic(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        message = test_action['message']
        qos = test_action['qos'] if 'qos' in test_action else QoSProfile(depth=10, durability=DurabilityPolicy.SYSTEM_DEFAULT)

        msg = topic_type()
        data = yaml.safe_load(message)
        set_message_fields(msg, data)

        pub = self.node.create_publisher(topic_type, topic, qos)
        pub.publish(msg)
        self.node.destroy_publisher(pub)
        case['done'] = True
        case['success'] = True

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
            origin_x = test_action['origin_x'] if 'origin_x' in test_action else self.config['origin_x'] if 'origin_x' in self.config else 0
            origin_y = test_action['origin_y'] if 'origin_y' in test_action else self.config['origin_y'] if 'origin_y' in self.config else 0
            init_x = test_action['x'] if 'x' in test_action else self.config['init_x']
            init_y = test_action['y'] if 'y' in test_action else self.config['init_y']
            init_z = test_action['z'] if 'z' in test_action else self.config['init_z']
            init_a = test_action['a'] if 'a' in test_action else self.config['init_a']
            init_yaw = init_a / 180.0 * math.pi
            request.state.pose.position.x = float(init_x + origin_x)
            request.state.pose.position.y = float(init_y + origin_y)
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
                        context = {'msg': msg, 'math': math}
                        exec(f"result=({condition})", context)
                        if context['result']:
                            case['done'] = True
                            case['success'] = True
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
            case['success'] = True
        manager.update(
            actors=test_action['actors'],
            callback=done_callback)

    @wait_test()
    def spawn_door(self, case, test_action):
        uuid = test_action['uuid']
        self.futures[uuid] = ObstacleManager.instance().spawn_door(**test_action)

        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
            case['success'] = True
        self.futures[uuid].add_done_callback(done_callback)

    @wait_test()
    def spawn_obstacle(self, case, test_action):
        uuid = test_action['uuid']
        self.futures[uuid] = ObstacleManager.instance().spawn_obstacle(**test_action)

        def done_callback(future):
            logger.debug(future.result())
            case['done'] = True
            case['success'] = True
        self.futures[uuid].add_done_callback(done_callback)

    @wait_test()
    def wait_topic(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        topic = test_action['topic']
        topic_type = test_action['topic_type']
        topic_type = import_class(topic_type)
        condition = test_action['condition']
        once = test_action['once'] if 'once' in test_action else False
        qos = test_action['qos'] if 'qos' in test_action else QoSProfile(depth=10, durability=DurabilityPolicy.SYSTEM_DEFAULT)

        def topic_callback(msg):
            try:
                context = {'msg': msg, 'math': math}
                exec(f"result=({condition})", context)
                if context['result']:
                    case['done'] = True
                    case['success'] = True
                    self.cancel_subscription(case)
                elif once:
                    case['done'] = True
                    case['success'] = False
                    case['msg'] = msg
                    self.cancel_subscription(case)
            except:  # noqa: #722
                logger.error(traceback.format_exc())
        sub = self.node.create_subscription(topic_type, topic, topic_callback, qos)
        self.add_subscription(case, sub)

    @wait_test()
    def wait(self, case, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        seconds = test_action['seconds']
        uuid = test_action['uuid']

        def timer_callback():
            case['done'] = True
            case['success'] = True
            timer = self.timers[uuid]
            timer.cancel()
            self.node.destroy_timer(timer)

        timer = self.node.create_timer(seconds, timer_callback)
        self.timers[uuid] = timer

    def terminate(self, test_action):
        logger.debug(f"{callee_name()} {test_action}")
        sys.exit(0)


@dataclass
class Door:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    width: float
    height: float
    depth: float

    @staticmethod
    def from_dict(**kwargs):
        valid_fields = {field.name for field in fields(Door)}
        filtered_kwargs = {k: v for k, v in kwargs.items() if k in valid_fields}
        return Door(**filtered_kwargs)


class ObstacleManager:
    _instance = None

    @classmethod
    def instance(cls):
        if not ObstacleManager._instance:
            ObstacleManager._instance = ObstacleManager()
        return ObstacleManager._instance

    def __init__(self):
        self.spawn_entity_client = node.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = node.create_client(DeleteEntity, '/delete_entity')
        self.plan_sub = node.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.obstacle_pub = node.create_publisher(People, '/obstacles', 10)
        self.timer = node.create_timer(0.2, self.timer_callback)
        self.remaining = []
        self.last_plan = None

    def plan_callback(self, msg):
        self.last_plan = msg

    def timer_callback(self):
        if not self.last_plan:
            return
        obstacle_point = None
        for pose in self.last_plan.poses:
            for obstacle in self.remaining:
                if self.is_point_in_rotated_rect(pose.pose.position, obstacle):
                    obstacle_point = pose.pose.position
                    break
        if obstacle_point:
            msg = Person()
            msg.name = obstacle.name
            msg.position.x = obstacle_point.x
            msg.position.y = obstacle_point.y
            msg.position.z = obstacle_point.z
            msg.reliability = 1.0
            msg.tags.append("stationary")
            pmsg = People()
            pmsg.people.append(msg)
            pmsg.header.stamp = node.get_clock().now().to_msg()
            pmsg.header.frame_id = "map_global"
            self.obstacle_pub.publish(pmsg)

    def is_point_in_rotated_rect(self, point, obstacle):
        margin = 0.45
        # Convert yaw to radians
        yaw = obstacle.yaw
        # Translate point to origin based on rect position
        translated_point_x = point.x - obstacle.x
        translated_point_y = point.y - obstacle.y
        # Rotate point around origin (0,0) in the opposite direction of the rectangle's rotation
        cos_yaw, sin_yaw = numpy.cos(-yaw), numpy.sin(-yaw)
        rotated_point_x = translated_point_x * cos_yaw - translated_point_y * sin_yaw
        rotated_point_y = translated_point_x * sin_yaw + translated_point_y * cos_yaw
        # Check if the rotated point is within the rectangle bounds
        return -margin-obstacle.width / 2 <= rotated_point_x <= obstacle.width / 2 + margin and \
               -margin-obstacle.height / 2 <= rotated_point_y <= obstacle.height / 2 + margin

    def clean(self, callback):
        self.last_path = None
        if self.remaining:
            future = self.delete_door(name=self.ramaining[0].name)

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
            self.remaining = [door for door in self.remaining if door.name != name]
        future.add_done_callback(callback)
        return future

    def spawn_door(self, **kwargs):
        return self.spawn_obstacle(**dict(
            dict(
                width=0.01,
                height=2.0,
                depth=2.0
            ),
            **kwargs)
        )

    def spawn_obstacle(self, **kwargs):
        door = Door.from_dict(**kwargs)
        name = door.name
        x = door.x
        y = door.y
        z = door.z
        yaw = door.yaw
        width = door.width
        height = door.height
        depth = door.depth

        door_xml = f"""
<?xml version="1.0" ?>
<sdf version="1.6">
    <model name="{name}">
        <static>true</static>
        <link name="{name}-link">
            <pose>{x} {y} {z+depth/2.0} 0 0 {yaw}</pose>
            <visual name="{name}-visual">
                <geometry>
                    <box>
                        <size>{width} {height} {depth}</size>
                    </box>
                </geometry>
            </visual>
            <collision name="{name}-collision">
                <geometry>
                    <box>
                        <size>{width} {height} {depth}</size>
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
            self.remaining.append(door)
            logger.debug(F"spawn result = {future.result()}, {door}, {len(self.remaining)}")
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
    parser = OptionParser()

    parser.add_option('-m', '--module', type=str, help='test module name')
    parser.add_option('-d', '--debug', action='store_true', help='debug print')
    parser.add_option('-f', '--func', type=str, help='test func name')
    parser.add_option('-L', '--list-modules', action='store_true', help='list test modules')
    parser.add_option('-l', '--list-functions', action='store_true', help='list test function')
    parser.add_option('-w', '--wait-ready', action='store_true', help='wait ready')

    (options, args) = parser.parse_args()

    if not options.module:
        parser.print_help()
        sys.exit(1)

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG if options.debug else logging.INFO)
    handler = logging.StreamHandler()
    handler.setFormatter(ColorFormatter())
    logger.addHandler(handler)

    if options.list_modules:
        module = __import__(options.module)
        modules = [name for _, name, _ in pkgutil.iter_modules(module.__path__)]
        for m in modules:
            logger.info(m)
        sys.exit(0)

    if options.list_functions:
        module = importlib.import_module(options.module)
        functions = [func for func in dir(module) if inspect.isfunction(getattr(module, func))]
        for f in functions:
            logger.info(f)
        sys.exit(0)

    rclpy.init()
    node = rclpy.node.Node("test_node")
    manager = PedestrianManager(node)
    ObstacleManager.instance()

    ros2Handler = ROS2LogHandler(node)
    logger.addHandler(ros2Handler)

    evaluator = Evaluator(node)
    evaluator.set_logger(logger)

    tester = Tester(node)
    tester.set_evaluator(evaluator)
    try:
        mod = importlib.import_module(options.module)
    except ModuleNotFoundError:
        logger.error(f"{options.module} is not found.")
        sys.exit(1)
    func_pat = None
    if options.func:
        try:
            func_pat = re.compile(options.func)
            logger.info(f"test func = {options.func}")
        except re.error as error:
            logger.error(error)
            return
    tester.test(mod, func_pat, wait_ready=options.wait_ready)


def exit_hook(status_code):
    logger.info(F"Exiting the program. {status_code}")
    try:
        if node:
            node.destroy_node()
            rclpy.shutdown()
    except:  # noqa: 722
        logger.info(traceback.format_exc())
    original_exit(status_code)


original_exit = sys.exit
sys.exit = exit_hook  # Set the exit hook


if __name__ == "__main__":
    main()
