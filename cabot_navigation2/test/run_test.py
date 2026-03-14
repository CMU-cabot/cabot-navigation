#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2024  Carnegie Mellon University and Miraikan
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

import os
import importlib
import pkgutil
import inspect
import csv
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
from pedestrian_plugin_msgs.msg import Agents
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterType
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
            case = {'target': tester.test_func_name, 'action': action_name, 'done': False, 'success': None, 'error': None}
            test_action = {'uuid': str(uuid.uuid4())}

            logger.debug(f"calling {function} {case} {test_action} - {args} {kwargs}")
            args = args + (case,)
            test_action.update(kwargs)
            result = function(*args, test_action)
            start = time.time()

            while not case['done'] and time.time() - start < t:
                rclpy.spin_once(node, timeout_sec=0.1)
                # Early exit if collision or error was detected by a check_topic_error subscription
                if tester.abort_current_test and not case['done']:
                    case['done'] = True
                    if case['success'] is None:
                        case['success'] = False
                        case['error'] = 'Aborted: collision/error detected'
                    break

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
    def __init__(self, node, output_dir, test_module_name):
        self.node = node
        self.output_dir = output_dir
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
        self.test_module_name = test_module_name
        self.result = {}
        self.test_summary = {}
        self.evaluator_summary = {}
        self.condition_list = []
        # evaluation
        self.evaluator = None
        # collision/error abort flag
        self.abort_current_test = False
        # abort_enabled is False until start_evaluation() is called (i.e. after navigation destination
        # is published). This prevents leftover queued messages on /collision_person from
        # triggering an abort during reset_position / setup_actors.
        self.abort_enabled = False
        self._nav_cancel_pub = None  # lazy-initialized publisher for navigation cancel

    def set_evaluator(self, evaluator):
        self.evaluator = evaluator

    def _send_navigation_cancel(self):
        """Immediately publish a navigation cancel event to stop the robot."""
        from std_msgs.msg import String as StringMsg
        if self._nav_cancel_pub is None:
            self._nav_cancel_pub = self.node.create_publisher(StringMsg, '/cabot/event', 10)
        msg = StringMsg()
        msg.data = 'navigation;cancel'
        self._nav_cancel_pub.publish(msg)
        logger.info("Sent navigation cancel due to collision/error detection")

    def add_metric_condition(self, condition):
        self.condition_list.append(condition)

    def check_conditions(self, results, func):
        if func not in self.result:
            self.result[func] = []

        for condition in self.condition_list:
            matching_result = next((result for result in results if result["name"] == condition["name"]), None)

            condition_result = {
                "action": f"check_{condition['name']}",
                "condition": f"{condition['condition']}"
            }

            if matching_result:
                value = matching_result["value"]
                condition_result["value"] = value

                if eval(condition["condition"]):
                    condition_result["success"] = True
                    condition_result["error"] = False
                else:
                    condition_result["success"] = False
                    condition_result["error"] = True
            else:
                condition_result["success"] = False
                condition_result["error"] = True

            self.result[func].append(condition_result)

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
            self.abort_current_test = False  # reset abort flag for each test case
            self.abort_enabled = False        # disable abort until start_evaluation() is called
            self.test_func_name = func
            getattr(module, func)(self)

            # If test was aborted due to collision, ensure navigation is cancelled and
            # allow time for the cancel command to be processed before resetting for
            # the next test case (prevents robot moving while actors are being set up).
            if self.abort_current_test:
                logger.warning(f"Test {func} was aborted due to collision/error. Waiting for navigation to stop...")
                self._send_navigation_cancel()  # send once more to be safe
                # Drain the ROS event loop briefly so the cancel message is delivered
                cancel_wait_start = time.time()
                while time.time() - cancel_wait_start < 3.0:
                    rclpy.spin_once(node, timeout_sec=0.1)

            self.stop_evaluation()  # automatically stop metric evaluation

            self._save_trajectory(func)

            try:
                evaluation_results = self.evaluator.get_evaluation_results()
                self.evaluator_summary[func] = evaluation_results
            except IndexError as e:
                logger.warning(f"Failed to calculate metrics for {func} due to empty data (IndexError): {e}")
                evaluation_results = []
                self.evaluator_summary[func] = []
            except Exception as e:
                logger.error(f"Failed to calculate metrics for {func}: {e}")
                evaluation_results = []
                self.evaluator_summary[func] = []

            self.check_conditions(evaluation_results, func)
            self.condition_list = []

            success = self.print_result(self.result, func)
            self.register_action_result(func, self.result)
            self.cancel_subscription(func)
            allSuccess = allSuccess and success
            
            if func not in self.test_summary:
                self.test_summary[func] = {'success': 0, 'failure': 0}
            if success:
                self.test_summary[func]['success'] += 1
            else:
                self.test_summary[func]['failure'] += 1

        self.output_test_summary()

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
                if 'condition' in aResult:
                    logger.info(f" - {action} ({aResult['condition']}): Success")
                else:
                    logger.info(f" - {action}: Success")
            else:
                if 'condition' in aResult:
                    logger.error(f" - {action} ({aResult['condition']}): Failure")
                else:
                    logger.error(f" - {action}: Failure")
                logger.error(f"{aResult['error']}")
        logger.info("--------------------------")
        return success

    def output_test_summary(self):
        test_summary_path = os.path.join(self.output_dir, 'test_summary.csv')
        test_evaluation_path = os.path.join(self.output_dir, 'test_evaluation_results.csv')

        # Read existing summary if available
        existing_summary = {}
        if os.path.exists(test_summary_path):
            with open(test_summary_path, mode='r', newline='') as file:
                reader = csv.reader(file)
                header = next(reader, None)
                if header:
                    for row in reader:
                        if len(row) >= 4:
                            # Skip Total row if it exists
                            if row[0] == "Total":
                                continue
                            
                            # Key: (module_name, case_name)
                            # CSV format: Test module name, Test case name, Success, Failure, Rate
                            key = (row[0], row[1])
                            existing_summary[key] = {
                                'success': int(row[2]),
                                'failure': int(row[3])
                            }

        # Update existing summary with current results
        for test_name, counts in self.test_summary.items():
            key = (self.test_module_name, test_name)
            if key not in existing_summary:
                existing_summary[key] = {'success': 0, 'failure': 0}
            
            existing_summary[key]['success'] += counts['success']
            existing_summary[key]['failure'] += counts['failure']

        # Calculate total statistics
        total_success = 0
        total_failure = 0
        for key, counts in existing_summary.items():
            total_success += counts['success']
            total_failure += counts['failure']
        
        total_count = total_success + total_failure
        total_rate = total_success / total_count if total_count > 0 else 0.0

        # Write back to file
        with open(test_summary_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Test module name", "Test case name", "Number of success", "Number of failure", "Success rate"])
            # Sort by keys for consistent output
            for key in sorted(existing_summary.keys()):
                module_name, case_name = key
                counts = existing_summary[key]
                success_count = counts['success']
                fail_count = counts['failure']
                current_total = success_count + fail_count
                success_rate = success_count / current_total if current_total > 0 else 0.0
                writer.writerow([module_name, case_name, success_count, fail_count, "{:.2f}".format(success_rate)])
            
            # Write Total row
            writer.writerow(["Total", "All cases", total_success, total_failure, "{:.2f}".format(total_rate)])

        # For evaluation results, we append to keep history if user wants history, 
        # or overwrite if they want fresh results for this run?
        # User asked for cumulative summary. Usually evaluation results are large.
        # But for consistency, maybe we should append?
        # The previous code overwrote it. Let's keep it overwrite for now OR do append?
        # If I change summary to be cumulative, evaluation results should probably correspond to the latest run or all runs?
        # If I append, the file grows indefinitely.
        # However, without appending, we lose the details of previous runs that contribute to the summary.
        # Let's assume user wants cumulative stats in summary, but maybe latest details in evaluation?
        # Actually, let's look at the original code again. It overwrote.
        # If I change to append, I must ensure the header is handled.
        
        file_exists = os.path.isfile(test_evaluation_path)
        with open(test_evaluation_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists or os.path.getsize(test_evaluation_path) == 0:
                writer.writerow(["Test module name", "Test case name", "evaluator", "value"])
            for test_name, results in self.evaluator_summary.items():
                for result in results:
                    writer.writerow([self.test_module_name, test_name, result["name"], result["value"]])

    def _save_trajectory(self, func_name):
        """Save the trajectory of the robot and all actors for the given test case to a CSV file."""
        trajectory_data = self.evaluator.get_trajectory_data()
        if not trajectory_data:
            logger.info(f"No trajectory data to save for {func_name}")
            return
        trajectory_path = os.path.join(self.output_dir, f'trajectory_{func_name}.csv')
        fieldnames = [
            'timestamp_sec', 'elapsed_sec', 'entity_name', 'entity_type',
            'pos_x', 'pos_y', 'pos_z', 'yaw',
            'quat_x', 'quat_y', 'quat_z', 'quat_w',
        ]
        with open(trajectory_path, mode='w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(trajectory_data)
        logger.info(f"Trajectory saved to {trajectory_path} ({len(trajectory_data)} rows)")

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
        # Allow abort from this point onward (navigation has started, so collisions are real)
        self.abort_enabled = True

    def stop_evaluation(self):
        """
        Stop comuting the metrics.

        It is usually not necessary to call this method because it is automatically called when the test ends.
        This method can be used when the user intentionally stops the metric computation
        """
        self.evaluator.stop()

    # people detection
    def set_people_detection_range(self, **kwargs):
        param_list = []

        for name in ['min_range', 'max_range', 'min_angle', 'max_angle', 'occlusion_radius', 'divider_distance_m', 'divider_angle_deg']:
            if name not in kwargs:
                continue

            param = {
                'name': f'pedestrian_plugin.{name}',
                'value': {
                    'type': ParameterType.PARAMETER_DOUBLE,
                    'double_value': kwargs[name]
                }
            }
            param_list.append(param)

        request_yaml = yaml.dump({'parameters': param_list})

        self.call_service(**dict(
            dict(
                action_name='set_people_detection_range',
                service='/gazebo/set_parameters',
                service_type='rcl_interfaces.srv/SetParameters',
                request=request_yaml,
                wait_for_service=True
            ),
            **kwargs)
        )

    def send_navigation_event(self, event):
        self.pub_topic(**dict(
            dict(
                action_name=f'send_navigation_event({event})',
                topic='/cabot/event',
                topic_type='std_msgs/msg/String',
                message=f"data: 'navigation_{event}'"
            ),
        ))

    """
    # do no use raw button events because there are some different button mappings
    # shorthand functions
    def button_up(self, button, **kwargs):
        self.pub_topic(**dict(
            dict(
                action_name=f'button_up({button})',
                topic='/cabot/event',
                topic_type='std_msgs/msg/String',
                message=f"data: 'button_up_{button}'"
            ),
            **kwargs)
        )
        return

    def button_down(self, button, hold=0, **kwargs):
        if hold:
            self.pub_topic(**dict(
                dict(
                    action_name=f'holddown_({button}_{hold})',
                    topic='/cabot/event',
                    topic_type='std_msgs/msg/String',
                    message=f"data: 'holddown_{button}_{hold}'"
                ),
                **kwargs)
            )
            return

        if button == 1:
            self.pub_topic(**dict(
                dict(
                    action_name=f'button_down({button})',
                    topic='/cabot/event',
                    topic_type='std_msgs/msg/String',
                    message="data: 'navigation_speedup'"
                ),
                **kwargs)
            )
        elif button == 2:
            self.pub_topic(**dict(
                dict(
                    action_name=f'button_down({button})',
                    topic='/cabot/event',
                    topic_type='std_msgs/msg/String',
                    message="data: 'navigation_speeddown'"
                ),
                **kwargs)
            )
        elif button == 3:
            self.pub_topic(**dict(
                dict(
                    action_name=f'button_down({button})',
                    topic='/cabot/event',
                    topic_type='std_msgs/msg/String',
                    message="data: 'navigation_pause'"
                ),
                **kwargs)
            )
        elif button == 4:
            self.pub_topic(**dict(
                dict(
                    action_name=f'button_down({button})',
                    topic='/cabot/event',
                    topic_type='std_msgs/msg/String',
                    message="data: 'navigation_resume'"
                ),
                **kwargs)
            )
    """

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
                topic="/collision_person",
                topic_type="pedestrian_plugin_msgs/msg/Collision",
                condition="True"
            ),
            **kwargs)
        )

    def check_collision_obstacle(self, **kwargs):
        return self.check_topic_error(**dict(
            dict(
                action_name='check_collision_obstacle',
                topic="/collision_obstacle",
                topic_type="pedestrian_plugin_msgs/msg/ObstacleCollision",
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

    def set_speed(self, speed, **kwargs):
        self.pub_topic(**dict(
            dict(
                action_name='setting speed',
                topic='/cabot/user_speed',
                topic_type='std_msgs/msg/Float32',
                qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                message=f"data: {speed}",
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

    def wait_mode_changed(self, modeName, **kwargs):
        self.wait_topic(**dict(
            dict(
                action_name=f'wait_mode_changed({modeName})',
                topic='/cabot/activity_log',
                topic_type='cabot_msgs/msg/Log',
                condition=f"msg.category=='cabot/navigation' and msg.text=='change_mode' and msg.memo=='{modeName}'",
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
        callback = test_action['callback'] if 'callback' in test_action else None
        uuid = test_action['uuid']
        # optional parameters for wait_for_service
        wait_for_service = test_action.get('wait_for_service', False)
        wait_for_service_timeout = test_action.get('wait_for_service_timeout', 1.0)

        req = request_type()
        data = yaml.safe_load(request)
        set_message_fields(req, data)

        srv = self.node.create_client(service_type, service)

        # wait for service if requested
        if wait_for_service:
            if not srv.service_is_ready():
                logger.debug(f"Waiting for service {service}...")
                if srv.wait_for_service(wait_for_service_timeout):
                    logger.debug(f"Finished waiting for service {service}")
                else:
                    logger.error(f"Timeout waiting for service {service}")
                    # return as fail
                    case['done'] = True
                    case['success'] = False
                    return

        self.futures[uuid] = srv.call_async(req)

        def done_callback(future):
            case['done'] = True
            case['success'] = True
            if callback:
                callback(future.result())

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

        def cancel_func():
            logger.debug(F"cancel {case}")
            self.cancel_subscription(case)
        return cancel_func

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
                    if self.abort_enabled:
                        # Abort the current test immediately and cancel navigation.
                        # Only when abort_enabled is True (i.e. after start_evaluation()),
                        # so that leftover queued messages during reset_position/setup_actors
                        # do not trigger a false abort.
                        self.abort_current_test = True
                        self._send_navigation_cancel()
                    else:
                        logger.warning("check_topic_error matched but abort is not yet enabled "
                                       "(navigation has not started). Ignoring abort, recording failure.")
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
    def clean_obstacle(self, case, test_action):
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
    def delete_obstacle(self, case, test_action):
        uuid = test_action['uuid']
        self.futures[uuid] = ObstacleManager.instance().delete_obstacle(**test_action)

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
            if future is not None:
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

    @wait_test()
    def assert_true(self, case, test_action):
        assert_condition = test_action['condition']
        case['done'] = True
        case['success'] = assert_condition

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
        self.node = node
        self.spawn_entity_client = node.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = node.create_client(DeleteEntity, '/delete_entity')
        self.plan_sub = node.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.obstacle_pub = node.create_publisher(People, '/obstacles', 10)
        self.timer = node.create_timer(0.2, self.timer_callback)
        self.remaining = []
        self.last_plan = None
        self.obstacle_states_sub = node.create_subscription(Agents, '/obstacle_states', self.obstacle_states_callback, 10)

    def plan_callback(self, msg):
        self.last_plan = msg

    def obstacle_states_callback(self, msg):
        if len(self.remaining) < len(msg.agents):
            remaining_names = [rem.name for rem in self.remaining]
            # agent_names = [agent.name for agent in msg.agents]
            for agent in msg.agents:
                if agent.name not in remaining_names:
                    obstacle = Door.from_dict(**{
                        "name": agent.name,
                        "x": agent.position.position.x,
                        "y": agent.position.position.y,
                        "z": agent.position.position.z,
                        "yaw": agent.yaw,
                        "width": 0,  # Agent.msg does not provide this parameter
                        "height": 0,  # Agent.msg does not provide this parameter
                        "depth": 0  # Agent.msg does not provide this parameter
                        })
                    self.remaining.append(obstacle)

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
            future = self.delete_door(name=self.remaining[0].name)
            self.remaining.pop(0)

            def done_callback(future):
                self.clean(callback)
            future.add_done_callback(done_callback)
        else:
            callback("Done")

    def delete_door(self, **kwargs):
        return self.delete_obstacle(**kwargs)

    def delete_obstacle(self, **kwargs):
        name = kwargs['name']
        request = DeleteEntity.Request()
        request.name = name
        future = self.delete_entity_client.call_async(request)

        def callback(future):
            self.remaining = [door for door in self.remaining if door.name != name]
            logger.debug(F"delete result = {future.result()}, {name}, {len(self.remaining)}")
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
        rclpy.spin_once(node, timeout_sec=1)  # wait until topic /obstacle_states ready
        door = Door.from_dict(**kwargs)
        if kwargs['name'] in [rem.name for rem in self.remaining]:
            # add suffix '_NUMBER' if the name already exists
            obstacle_suffix_num = 1
            while kwargs['name']+f"_{obstacle_suffix_num}" \
                    in [rem.name for rem in self.remaining]:
                obstacle_suffix_num += 1
            name = door.name + f"_{obstacle_suffix_num}"
        else:
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
        <pose>{x} {y} {z+depth/2.0} 0 0 {yaw}</pose>
        <link name="{name}-link">
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
        <plugin name="pedestrian_plugin_{name}" filename="libobstacle_plugin.so">
            <module>pedestrian.obstacle</module>
            <robot>mobile_base</robot>
        </plugin>
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
    parser.add_option('-o', '--output-dir', type=str, help='directory where the summary will be output')

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

    tester = Tester(node, options.output_dir, options.module)
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