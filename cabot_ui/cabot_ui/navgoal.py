# Copyright (c) 2022, 2024  Carnegie Mellon University and Miraikan
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from typing import List
import math
import inspect
import numpy
import threading
import time
import traceback
import yaml

from cabot_ui import geoutil, geojson
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

import tf_transformations
import nav_msgs.msg
import geometry_msgs.msg

from action_msgs.msg import GoalStatus

from cabot_common import util


class GoalInterface(object):
    def activity_log(self, category="", text="", memo=""):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def enter_goal(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def exit_goal(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def naviget_to_pose(self, goal_pose, bt_xml, done_cb):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def naviget_through_poses(self, goal_poses, bt_xml, done_cb):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def set_pause_control(self, flag):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def turn_towards(self, orientation, callback, clockwise=0):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def goto_floor(self, floor, callback):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_call_elevator(self, pos):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def publish_path(self, global_path, convert=True):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def global_map_name(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_pass_door(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def door_passed(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_follow_behind(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_return_position(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def change_parameters(self, params):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def request_parameters(self, params):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")


def make_goals(delegate, groute, anchor, yaw=None):
    """
    This function makes one or multiple action goals towards the destination to deal with
    manual door, elevators, queue, narrow path, and so on.

    :param delegate: delegate instance for Goal
    :type  delegate: GoalInterface
    :param groute: route obtained from MapService in global coordinate
    :type  groute: [geojson.Object]
    :param anchor: anchor to convert the global coordinate to local coordinate
    :type  anchor: geoutil.Anchor

    :returns: a list of Goal
    :rtype: [navgoal.Goal]
    """
    CaBotRclpyUtil.info(F"--make_goals-{len(groute)}--------------------")
    CaBotRclpyUtil.debug(F"{groute}")

    if len(groute) == 0:
        CaBotRclpyUtil.error("groute length should not be 0")
        return []
    if len(groute) == 1:
        # if route is only a node
        return [NavGoal(delegate, groute, anchor, is_last=True)]

    goals = []
    route_objs = []
    index = 1
    is_last = False

    while index < len(groute):
        link_or_node = groute[index]
        index += 1
        if not isinstance(link_or_node, geojson.RouteLink):
            continue
        # This is a hulistic rule to deal with the last leaf link towards doorway in corridor
        # It needs to check if the last link has perpendicular turn
        # if the link is a leaf of the graph and short
        link = link_or_node
        if link.target_node.is_leaf and link.length < 3.0:
            continue
        route_objs.append(link)
        is_last = (index == len(groute) - 1)

        # Handle Door POIs
        #
        #      o======== (Door) ========o
        #  x-----1.0m----(Door)----1.0m-----x
        #
        doors = list(filter(lambda x: isinstance(x, geojson.DoorPOI) and not x.is_auto, link.pois))
        if doors:
            if len(doors) > 1:
                # will not support multiple doors in a link
                CaBotRclpyUtil.warn("don't put multiple door pois into a link")

            # set previous goal 1.0 meter behind the door
            if len(route_objs) >= 2:
                goals.append(NavGoal(delegate, route_objs[:-1], anchor, target_poi=doors[0], set_back=(1.0, 0.0)))
            # ask user to pass the door
            goals.append(DoorGoal(delegate, link, anchor, doors[0]))
            route_objs = []

        # Handle Queue Target and Queue Wait
        # TBD
        queue_targets = list(filter(lambda x: isinstance(x, geojson.QueueTargetPOI), link.pois))
        if queue_targets:
            if len(queue_targets) > 1:
                # will not support
                CaBotRclpyUtil.error("don't put multiple queue target pois into a link")
                return None
            else:
                # find link with queue enter node from whole route
                queue_enter_link_idx = None
                queue_enter_link_source_node_id = None
                for idx_temp_r, temp_r in enumerate(groute):
                    if isinstance(temp_r, geojson.RouteLink) and temp_r.target_node._id == queue_targets[0].enter_node._id:
                        queue_enter_link_idx = idx_temp_r
                        queue_enter_link_source_node_id = temp_r.source_node._id
                        break
                if queue_enter_link_idx is None:
                    CaBotRclpyUtil.error("link whose targeet node is queue enter node is not found")
                    return None

                # find the linke before the link connected with queue enter node
                before_queue_enter_link_idx = None
                for idx_temp_r, temp_r in enumerate(groute):
                    if isinstance(temp_r, geojson.RouteLink) and temp_r.target_node._id == queue_enter_link_source_node_id:
                        before_queue_enter_link_idx = idx_temp_r
                        break
                if before_queue_enter_link_idx is not None:
                    # until before the link connected with queue enter node, navigate in normal mode
                    befoer_queue_enter_groute = groute[:before_queue_enter_link_idx+1]
                    befoer_queue_enter_groute.append(groute[before_queue_enter_link_idx].target_node)
                    goals.append(NavGoal(delegate, befoer_queue_enter_groute, anchor))

                    # from the link connected with queue enter node, navigate in queue mode to prevent shortcutting path to queue enter node
                    goals.append(QueueNavFirstGoal(delegate, [groute[queue_enter_link_idx].source_node,
                                                              groute[queue_enter_link_idx],
                                                              groute[queue_enter_link_idx].target_node], anchor))
                else:
                    # the linke before the link connected with queue enter node is not found, start navigate in queue mode because queue enter node is close
                    queue_enter_groute = groute[:queue_enter_link_idx+1]
                    queue_enter_groute.append(groute[queue_enter_link_idx].target_node)
                    goals.append(QueueNavFirstGoal(delegate, queue_enter_groute, anchor))

                # select route from queue enter node to queue target node
                queue_target_groute = groute[queue_enter_link_idx+1:]
                queue_target_groute.insert(0, groute[queue_enter_link_idx].target_node)
                # find queue wait POI that is not copied in queue route yet
                queue_link_idx_queue_wait_dict = {}
                for idx_temp_r, temp_r in enumerate(queue_target_groute):
                    if isinstance(temp_r, geojson.RouteLink):
                        queue_waits = list(filter(lambda x: isinstance(x, geojson.QueueWaitPOI) and not x.is_copied, temp_r.pois))
                        if queue_waits:
                            if len(queue_waits) > 1:
                                CaBotRclpyUtil.error("don't put multiple queue wait pois into a queue route")
                                return None
                            else:
                                queue_link_idx_queue_wait_dict[idx_temp_r] = queue_waits

                for idx_temp_r, temp_r in enumerate(queue_target_groute):
                    if isinstance(temp_r, geojson.RouteLink) and idx_temp_r in queue_link_idx_queue_wait_dict:
                        queue_waits = queue_link_idx_queue_wait_dict[idx_temp_r]
                        # if POI that is not copied is found, copy queue wait POI along the queue route
                        if queue_waits is not None and len(queue_waits) == 1:
                            queue_waits[0].register_link(queue_target_groute[idx_temp_r])

                            temp_queue_wait = queue_waits[0]
                            temp_queue_wait_interval = queue_waits[0].interval
                            while True:
                                temp_queue_wait_position = numpy.array([temp_queue_wait.local_geometry.x, temp_queue_wait.local_geometry.y])
                                temp_r_start_position = numpy.array([queue_target_groute[idx_temp_r].source_node.local_geometry.x,
                                                                     queue_target_groute[idx_temp_r].source_node.local_geometry.y])
                                dist_poi_r_start = numpy.linalg.norm(temp_queue_wait_position-temp_r_start_position)
                                if dist_poi_r_start > temp_queue_wait_interval:
                                    temp_r_end_pose = geoutil.Pose.pose_from_points(queue_target_groute[idx_temp_r].target_node.local_geometry,
                                                                                    queue_target_groute[idx_temp_r].source_node.local_geometry, backward=True)
                                    temp_queue_wait_x = temp_queue_wait.local_geometry.x - math.cos(temp_r_end_pose.r) * temp_queue_wait_interval
                                    temp_queue_wait_y = temp_queue_wait.local_geometry.y - math.sin(temp_r_end_pose.r) * temp_queue_wait_interval

                                    temp_queue_wait = queue_waits[0].copy_to_link(queue_target_groute[idx_temp_r], temp_queue_wait_x, temp_queue_wait_y)
                                else:
                                    break
                # create multiple goals to queue target node
                goals.extend(make_queue_goals(delegate, queue_target_groute, anchor))

                # navigate to queue exit node
                queue_exit_groute = delegate._datautil.get_route(link.target_node._id, queue_targets[0].exit_node._id)
                if len(queue_exit_groute) == 0:
                    CaBotRclpyUtil.error("route to queue exit node is not found")
                    return None
                # create one goal to queue exit node
                goals.append(QueueNavLastGoal(delegate, queue_exit_groute, anchor))

                route_objs = []

        if link.is_elevator:
            continue

        if link.target_node.is_elevator:
            # find cab POIs
            # elevator cab POIs are associated with non elevator links
            src_cabs = list(filter(lambda x: isinstance(x, geojson.ElevatorCabPOI), route_objs[-1].pois))
            CaBotRclpyUtil.info(F"src_cabs {str(src_cabs)}")
            if len(src_cabs) == 0:
                CaBotRclpyUtil.error("require elevator cab POI")
                CaBotRclpyUtil.error(route_objs[-1])
                return None
            else:
                if len(src_cabs) > 1:
                    # TODO
                    CaBotRclpyUtil.warn("multiple cabs are not supported yet, use the first")

                if len(route_objs) > 1:
                    # navigat to in front of the elevator cab (3.0 meters from the cab as default)
                    goals.append(NavGoal(delegate, route_objs, anchor, target_poi=src_cabs[0], set_back=src_cabs[0].set_back))
                # turn towards door
                goals.append(ElevatorWaitGoal(delegate, src_cabs[0]))
                # wait cab and get on the cab
                goals.append(ElevatorInGoal(delegate, src_cabs[0], is_last=is_last))
                route_objs = []

        if link.source_node.is_elevator:
            dest_cabs = list(filter(lambda x: isinstance(x, geojson.ElevatorCabPOI), link.pois))
            if len(dest_cabs) > 1:
                CaBotRclpyUtil.warn("multiple cabs are not supported yet, use the first one")

            # turn towards exit door (assumes cab POI has angle facing towards opening door)
            goals.append(ElevatorTurnGoal(delegate, dest_cabs[0]))
            # CaBotRclPyUtil.info(goals[-1])
            # wait until the door open
            goals.append(ElevatorFloorGoal(delegate, dest_cabs[0]))
            # CaBotRclPyUtil.info(goals[-1])
            # forward 3.0 meters (as default) without global mapping support
            goals.append(ElevatorOutGoal(delegate, dest_cabs[0], set_forward=dest_cabs[0].set_forward, is_last=is_last))
            # CaBotRclPyUtil.info(goals[-1])
            route_objs = []

        # TODO: escalator

    if len(route_objs) > 0:
        goals.append(NavGoal(delegate, route_objs, anchor, is_last=True))

    if yaw is not None:
        goal_node = groute[-1]  # should be Node
        goals.append(TurnGoal(delegate, goal_node, anchor, yaw))

    CaBotRclpyUtil.debug(F"goals: {goals}")
    return goals


def create_ros_path(navcog_route, anchor, global_map_name, target_poi=None, set_back=[0.0, 0.0]):
    """convert a NavCog path to ROS path"""
    mode = geojson.NavigationMode.Standard
    # convert route to points
    points = []
    if len(navcog_route) == 0:
        CaBotRclpyUtil.error("create_ros_path, navcog_route length should not be 0")
        return points
    CaBotRclpyUtil.debug(F"create_ros_path, {str(navcog_route)}")
    last_index = len(navcog_route) - 1

    def convert(g, a=anchor):
        return geoutil.global2local(g, a)

    for (index, item) in enumerate(navcog_route):
        if index == 0 and isinstance(item.geometry, geojson.LineString):
            if item.navigation_mode != geojson.NavigationMode.Standard:
                # if the first item is link, add the source node
                points.append(convert(item.source_node.geometry))
            else:
                sp = convert(item.source_node.geometry)
                ep = convert(item.target_node.geometry)
                points.append(sp.interpolate(ep, 0.5))
        elif index == last_index:
            if isinstance(item.geometry, geojson.Point):
                # if navcog_route only has one Node
                if len(navcog_route) == 1:
                    points.append(convert(item.geometry))
                # if last item is Point (Node), it would be same as the previous link target node
                continue

        if hasattr(item, "navigation_mode"):
            mode = item.navigation_mode

        if isinstance(item.geometry, geojson.Point):
            points.append(convert(item.geometry))
        elif isinstance(item.geometry, geojson.LineString):
            points.append(convert(item.target_node.geometry))
        else:
            CaBotRclpyUtil.debug("geometry is not point or linestring {item.geometry}")
        CaBotRclpyUtil.debug(F"{index}: {str(points)}")

    # make a path from points
    path = nav_msgs.msg.Path()
    path.header.frame_id = global_map_name
    path.poses = []
    quaternion = None
    prev_orientation = geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    for i in range(0, len(points)):
        start = points[i]
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = global_map_name
        pose.pose = geometry_msgs.msg.Pose()
        pose.pose.position.x = start.x
        pose.pose.position.y = start.y
        pose.pose.position.z = 0.0
        if i+1 < len(points):
            end = points[i+1]
            direction = math.atan2(end.y - start.y, end.x - start.x)
            quaternion = tf_transformations.quaternion_from_euler(0, 0, direction)

            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path.poses.append(pose)
            prev_orientation = pose.pose.orientation
        else:
            pose.pose.orientation = prev_orientation
            path.poses.append(pose)

    # if target_poi is specified, the last_pose will be the target_poi
    if target_poi:
        last_pose = target_poi.to_pose_msg()
        # if set_back is specified too, move the last_pose based on the last link direction
        if set_back[0] > 0 or set_back[1] > 0:
            i = 1
            last_obj = navcog_route[-1]
            while isinstance(last_obj, geojson.Node) and i < len(navcog_route):
                i += 1
                last_obj = navcog_route[-i]
            if not isinstance(last_obj, geojson.RouteLink):
                raise RuntimeError("There should be at least one link towards target POI,"
                                   F"if it is provided with set_back (={set_back}) \n===POI===\n{str(target_poi)}\n=========")
            backward = geoutil.Pose.pose_from_points(last_obj.source_node.local_geometry,
                                                     last_obj.target_node.local_geometry, backward=True)
            CaBotRclpyUtil.info(F"set_back {backward.r:.4} * ({str(set_back)})")
            last_pose.position.x += math.cos(backward.r) * set_back[0] - math.sin(backward.r) * set_back[1]
            last_pose.position.y += math.sin(backward.r) * set_back[0] + math.cos(backward.r) * set_back[1]
        if target_poi is not None:
            path.poses[-1].pose.position.x = last_pose.position.x
            path.poses[-1].pose.position.y = last_pose.position.y

    CaBotRclpyUtil.debug(F"path {path}")
    return (path, path.poses[-1] if len(path.poses) > 0 else None, mode)


def estimate_next_goal(goals, current_pose, current_floor):
    CaBotRclpyUtil.info(F"estimate_next_goal is called: len(goals)={len(goals)}, {current_pose}, {current_floor}")
    for i in range(len(goals), 0, -1):
        CaBotRclpyUtil.info(F"checking goal[{i-1}]")
        goal = goals[i-1]
        if goal.completed(pose=current_pose, floor=current_floor):
            continue
        if goal.match(pose=current_pose, floor=current_floor):
            return (goal, i-1)
    return (None, 0)  # might be reached the goal


class Goal(geoutil.TargetPlace):
    GOAL_XY_THRETHOLD = 0.5
    GOAL_ANGLE_THRETHOLD_IN_DEGREE = 15
    default_params = {}

    def __init__(self, delegate, **kwargs):
        super(Goal, self).__init__(**kwargs)
        if delegate is None:
            raise RuntimeError("please provide proper delegate object")
        self.delegate = delegate
        self._logger = delegate.get_logger()
        self.is_last = kwargs['is_last'] if 'is_last' in kwargs else False
        self._ready_to_execute = False
        self._is_completed = False
        self._is_canceled = False
        self._current_statement = None
        self.global_map_name = self.delegate.global_map_name()
        self._handles = []
        self._saved_params = Goal.default_params
        self._is_exiting_goal = False

    def reset(self):
        self._is_completed = False
        self._is_canceled = False
        self._is_exiting_goal = False

    def enter(self):
        """
        Goal.enter() is called when the goal is activated
        This tries to save parameters and then change parameters, following Goal._enter() call
        Subclass may override _enter() function to implement its process
        """
        if self._is_canceled:
            CaBotRclpyUtil.info(f"{self} enter called, but already cancelled")
            return
        self.delegate.enter_goal(self)

        def change_callback():
            self._enter()
        self._change_params(self.nav_params(), change_callback)

    def _enter(self):
        pass

    def _save_params(self, callback):
        CaBotRclpyUtil.info(F"{util.callee_name()} is called")

        def done_request_parameters_callback(result):
            CaBotRclpyUtil.info("done_request_parameters_callback is called")
            self._saved_params = result
            CaBotRclpyUtil.info(F"{self.__class__.__name__}._saved_params = {self._saved_params}")
            callback()
        if self._saved_params:
            done_request_parameters_callback(self._saved_params)
        else:
            if self.nav_params_keys():
                CaBotRclpyUtil.info(f"call request_parameters {self.nav_params_keys()}")
                self.delegate.request_parameters(self.nav_params_keys(), done_request_parameters_callback)
            else:
                callback()

    def _change_params(self, params, callback):
        def done_change_parameters_callback(params):
            def inner(result):
                CaBotRclpyUtil.info(f"done_change_parameters_callback is called {params}")
                # update saved_params by changed params
                for node, values in list(params.items()):
                    if node in self._saved_params:
                        for key, value in list(values.items()):
                            self._saved_params[node][key] = value
                CaBotRclpyUtil.info(f"{self._saved_params=}")
                callback()
            return inner
        if self._saved_params:
            for node, values in list(params.items()):
                if node in self._saved_params:
                    for key, value in list(values.items()):
                        if key in self._saved_params[node] and self._saved_params[node][key] == value:
                            del params[node][key]
                        if not params[node]:
                            del params[node]
        CaBotRclpyUtil.info(F"params to be changed: {params=}")
        if params:
            self.delegate.change_parameters(params, done_change_parameters_callback(params))
        else:
            callback()

    def nav_params_keys(self):
        return []

    def nav_params(self):
        return {}

    def check(self, current_pose):
        pass

    def update_goal(self, goal):
        pass

    def estimate_inner_goal(self, current_pose, current_floor):
        pass

    @property
    def is_completed(self):
        return self._is_completed

    @property
    def is_canceled(self):
        return self._is_canceled

    @property
    def is_exiting_goal(self):
        return self._is_exiting_goal

    def exit(self, callback):
        def done_change_parameters_callback():
            CaBotRclpyUtil.info(F"{self.__class__.__name__}.exit done_change_parameters_callback is called")
            callback()
        CaBotRclpyUtil.info(F"{self.__class__.__name__}.exit is called")
        CaBotRclpyUtil.info(F"saved_params = {self._saved_params}")
        self.delegate.exit_goal(self)
        if self._saved_params:
            self._is_exiting_goal = True
            self._change_params(Goal.default_params, done_change_parameters_callback)
        else:
            callback()

    @property
    def current_statement(self):
        return self._current_statement

    @property
    def is_social_navigation_enabled(self):
        return True

    def __str__(self):
        ret = F"{type(self)}, ({hex(id(self))})\n"
        for key in self.__dict__:
            value = getattr(self, key)
            ret += F"{key}: {value}\n"

        import inspect
        for method in inspect.getmembers(type(self), predicate=lambda o: isinstance(o, property)):
            key = method[0]
            value = method[1].__get__(self, type(self))
            ret += F"{key}: {value}\n"

        return ret

    def __repr__(self):
        return F"{super(Goal, self).__repr__()}"

    def goal_handle_callback(self, handle, cancel_callback=None):
        self._handles.append((handle, cancel_callback))
        if self._is_canceled:
            self.cancel()

    def cancel(self, callback=None):
        CaBotRclpyUtil.info(F"{self.__class__.__name__}.cancel is called")
        try:
            def done_change_parameters_callback(result):
                CaBotRclpyUtil.info(F"{self.__class__.__name__}.cancel done_change_parameters_callback is called")
                self._saved_params = None
                self._cancel(callback)
            if self._saved_params:
                self.delegate.change_parameters(self._saved_params, done_change_parameters_callback)
            else:
                self._cancel(callback)
        except:  # noqa: #722
            self._logger.error(traceback.format_exc())

    def _cancel(self, callback=None):
        self._is_canceled = True

        if len(self._handles) > 0:
            (handle, cancel_callback) = self._handles.pop(0)
            future = handle.cancel_goal_async()

            def done_callback(future):
                if future.cancelled():
                    self._logger.info("Future was cancelled.")
                elif future.done():
                    error = future.exception()
                    if error:
                        self._logger.error(f"Future resulted in error: {error}")
                    else:
                        self._logger.info(f"cancel future result = {future.result}")
                if cancel_callback:
                    cancel_callback()
                self.delegate._process_queue.append((self.cancel, callback))
            future.add_done_callback(done_callback)

            def timeout_watcher(future, timeout_duration):
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > timeout_duration:
                        self._logger.error("Timeout occurred while waiting for future to complete")
                        future.cancel()
                        return
                    time.sleep(0.1)

            timeout_tread = threading.Thread(target=timeout_watcher, args=(future, 5))
            timeout_tread.start()

            self._logger.info(f"sent cancel goal: {len(self._handles)} handles remaining")
        else:
            if callback:
                callback()
            self._logger.info("done cancel goal")

    def match(self, pose, floor):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")
        return False

    def completed(self, pose, floor):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")
        return False


class Nav2Params:
    @classmethod
    def get_parameters_for(cls, mode):
        if mode == geojson.NavigationMode.Standard:
            params = """
/planner_server:
    CaBot.path_adjusted_center: 0.0
    CaBot.path_adjusted_minimum_path_width: 0.5
    CaBot.path_width: 2.0
    CaBot.min_iteration_count: 500
    CaBot.max_iteration_count: 1000
    CaBot.ignore_people: False
/footprint_publisher:
    footprint_mode: 0
/controller_server:
    FollowPath.max_vel_x: 1.0
    FollowPath.sim_time: 1.7
    cabot_goal_checker.xy_goal_tolerance: 0.5
/global_costmap/global_costmap:
    inflation_layer.inflation_radius: 0.75
/local_costmap/local_costmap:
    inflation_layer.inflation_radius: 0.75
/cabot/lidar_speed_control_node:
    min_distance: 1.0
/cabot/low_lidar_speed_control_node:
    min_distance: 1.0
/cabot/people_speed_control_node:
    social_distance_x: 2.0
    social_distance_y: 0.5
"""
        if mode == geojson.NavigationMode.Narrow:
            params = """
/planner_server:
    CaBot.path_adjusted_center: 0.0
    CaBot.path_adjusted_minimum_path_width: 0.0
    CaBot.path_width: 0.0
    CaBot.min_iteration_count: 5
    CaBot.max_iteration_count: 10
    CaBot.ignore_people: True
/footprint_publisher:
    footprint_mode: 3
/controller_server:
    FollowPath.max_vel_x: 0.5
    FollowPath.sim_time: 0.5
    cabot_goal_checker.xy_goal_tolerance: 0.1
/global_costmap/global_costmap:
    inflation_layer.inflation_radius: 0.45
/local_costmap/local_costmap:
    inflation_layer.inflation_radius: 0.45
/cabot/lidar_speed_control_node:
    min_distance: 0.60
/cabot/low_lidar_speed_control_node:
    min_distance: 0.60
/cabot/people_speed_control_node:
    social_distance_x: 1.0
    social_distance_y: 0.50
/cabot/speed_control_node_touch_true:
    complete_stop: [false,false,true,false,true,false,true]
"""
        if mode == geojson.NavigationMode.Tight:
            params = """
/planner_server:
    CaBot.path_adjusted_center: 0.0
    CaBot.path_adjusted_minimum_path_width: 0.0
    CaBot.path_width: 0.0
    CaBot.min_iteration_count: 5
    CaBot.max_iteration_count: 10
    CaBot.ignore_people: True
/footprint_publisher:
    footprint_mode: 1
/controller_server:
    FollowPath.max_vel_x: 0.5
    FollowPath.sim_time: 0.5
    cabot_goal_checker.xy_goal_tolerance: 0.1
/global_costmap/global_costmap:
    inflation_layer.inflation_radius: 0.25
/local_costmap/local_costmap:
    inflation_layer.inflation_radius: 0.25
/cabot/lidar_speed_control_node:
    min_distance: 0.60
/cabot/low_lidar_speed_control_node:
    min_distance: 0.60
/cabot/people_speed_control_node:
    social_distance_x: 1.0
    social_distance_y: 0.50
/cabot/speed_control_node_touch_true:
    complete_stop: [false,false,true,false,true,false,true]
"""
        if mode == geojson.NavigationMode.Crosswalk:
            params = """
/planner_server:
    CaBot.path_adjusted_center: 0.0
    CaBot.path_adjusted_minimum_path_width: 0.0
    CaBot.path_width: 0.0
    CaBot.min_iteration_count: 5
    CaBot.max_iteration_count: 10
    CaBot.ignore_people: True
/footprint_publisher:
    footprint_mode: 3
/controller_server:
    FollowPath.max_vel_x: 1.0
    FollowPath.sim_time: 0.5
    cabot_goal_checker.xy_goal_tolerance: 0.5
/global_costmap/global_costmap:
    inflation_layer.inflation_radius: 0.45
/local_costmap/local_costmap:
    inflation_layer.inflation_radius: 0.45
/cabot/lidar_speed_control_node:
    min_distance: 0.60
/cabot/low_lidar_speed_control_node:
    min_distance: 0.60
/cabot/people_speed_control_node:
    social_distance_x: 1.0
    social_distance_y: 0.50
/cabot/speed_control_node_touch_true:
    complete_stop: [false,false,true,false,true,false,true]
"""
        # if simualtion, check if 'speed_control_node_touch_true' or 'speed_control_node_touch_false' is used
        if CaBotRclpyUtil.instance().use_sim_time:
            CaBotRclpyUtil.info("parameters simulation mode")
            # returns List[Tuple[node_name, namespace]]
            nodes = CaBotRclpyUtil.instance().node.get_node_names_and_namespaces()
            names = [name for name, ns in nodes]
            for name in names:
                CaBotRclpyUtil.info(f"{name}")
            if "speed_control_node_touch_false" in names:
                params = params.replace("/cabot/speed_control_node_touch_true", "/cabot/speed_control_node_touch_false")

        data = yaml.safe_load(params)
        return data

    @classmethod
    def aggregate_nested_keys(cls, dicts):
        """
        Aggregate all unique second-level keys for each first-level key across multiple two-level nested dictionaries.

        Parameters:
        dicts (list): List of dictionaries to process.

        Returns:
        dict: A dictionary where each key is a first-level key and each value is a list of unique second-level keys.
        """
        aggregated_keys = {}
        for d in dicts:
            for key, subdict in d.items():
                if key not in aggregated_keys:
                    aggregated_keys[key] = set()
                aggregated_keys[key].update(subdict.keys())
        # Convert sets to lists
        for key, value in aggregated_keys.items():
            aggregated_keys[key] = list(value)
        return aggregated_keys

    @classmethod
    def all_keys(cls):
        return cls.aggregate_nested_keys([cls.get_parameters_for(mode) for mode in geojson.NavigationMode])


if __name__ == "__main__":
    print(Nav2Params.all_keys())


class NavGoal(Goal):
    """
    only a node:  (R)     o
    only a link:  (R)     -----
    link and node:(R)     -----o-----
    link and node:(R)     -----o-----o
    """
    DEFAULT_BT_XML = "package://cabot_bt/behavior_trees/cabot_navigate.xml"
    NEIGHBOR_THRESHOLD = 1.0

    def __init__(self, delegate, navcog_route, anchor, target_poi=None, set_back=(0, 0), **kwargs):
        if navcog_route is None or len(navcog_route) == 0:
            raise RuntimeError("navcog_route should have one more object")
        if anchor is None:
            raise RuntimeError("anchor should be provided")

        # need init global_map_name for initialization
        self.global_map_name = delegate.global_map_name()
        self.navcog_route = navcog_route
        self.anchor = anchor

        # handle corner case (only a node is in the navcog_route)
        if len(navcog_route) == 1 and isinstance(navcog_route[0], geojson.Node):
            self.separated_route = [navcog_route]
            self.navcog_routes = [create_ros_path(navcog_route, self.anchor, self.global_map_name, target_poi=target_poi, set_back=set_back)]
        else:
            self.separated_route = self.separate_route(navcog_route)
            # self.separated_route = [list(group) for _, group in groupby(navcog_route, key=lambda x: x.navigation_mode)]
            self.navcog_routes = [
                create_ros_path(
                    route,
                    self.anchor,
                    self.global_map_name,
                    target_poi=target_poi if index == len(self.separated_route) - 1 else None,
                    set_back=set_back if index == len(self.separated_route) - 1 else None
                ) for index, route in enumerate(self.separated_route)
            ]

        last_pose = self.navcog_routes[-1][1]
        self.pois = self._extract_pois()
        self.gradient = self._extract_gradient()
        self.handle = None
        self.mode = None
        self.route_index = 0
        super(NavGoal, self).__init__(delegate, angle=180, floor=navcog_route[-1].floor, pose_msg=last_pose, **kwargs)

    def separate_route(self, route: List[geojson.RouteLink]) -> List[List[geojson.RouteLink]]:
        separated_routes = []
        current_group = [route[0]]
        for i in range(1, len(route)):
            current_link = route[i]
            previous_link = route[i - 1]
            orientation_diff = math.fabs(geoutil.diff_angle(current_link.pose.orientation, previous_link.pose.orientation))
            if current_link.navigation_mode != previous_link.navigation_mode or \
               (previous_link.navigation_mode != geojson.NavigationMode.Standard and orientation_diff > 80.0 / 180.0 * math.pi):
                separated_routes.append(current_group)
                current_group = [current_link]
            else:
                current_group.append(current_link)
        separated_routes.append(current_group)  # Add the last group
        return separated_routes

    def _extract_pois(self):
        """extract pois along the route"""
        temp = []
        for (_, item) in enumerate(self.navcog_route):
            if isinstance(item, geojson.RouteLink):
                CaBotRclpyUtil.debug(item._id)
                for poi in item.pois:
                    CaBotRclpyUtil.debug(["  ", type(poi), poi._id])
                temp.extend(item.pois)
        return temp

    def _extract_gradient(self):
        """extract gradient along the route"""
        temp = []
        for (_, item) in enumerate(self.navcog_route):
            if isinstance(item, geojson.RouteLink):
                if item.gradient in [geojson.Gradient.Up, geojson.Gradient.Down]:
                    temp.append(item)
        return temp

    @util.setInterval(5, times=1)
    def call_delay(self, func):
        func()

    def nav_params_keys(self):
        return Nav2Params.all_keys()

    def nav_params(self):
        new_mode = self.navcog_routes[self.route_index][2]
        self.mode = new_mode
        return Nav2Params.get_parameters_for(new_mode)

    def enter(self):
        CaBotRclpyUtil.info("NavGoal enter")
        new_mode = self.navcog_routes[self.route_index][2]
        CaBotRclpyUtil.info(F"NavGoal.check_mode new_mode={new_mode}")
        delay = False
        if self.mode != geojson.NavigationMode.Tight and new_mode == geojson.NavigationMode.Tight:
            self.delegate.please_follow_behind()
            delay = True
        if self.mode == geojson.NavigationMode.Tight and new_mode != geojson.NavigationMode.Tight:
            self.delegate.please_return_position()
            delay = True
        self.mode = new_mode
        if delay:
            self.call_delay(super(NavGoal, self).enter)
        else:
            super(NavGoal, self).enter()

    def _enter(self):
        CaBotRclpyUtil.info("NavGoal._enter is called")
        # publish navcog path
        path = self.navcog_routes[self.route_index][0]
        path.header.stamp = CaBotRclpyUtil.now().to_msg()
        self.delegate.publish_path(path)
        CaBotRclpyUtil.info("NavGoal publish path")

        # wanted a path (not only a pose) in planner plugin, but it is not possible
        # bt_navigator will path only a pair of consecutive poses in the path to the plugin
        # so we use navigate_to_pose and planner will listen the published path
        # self.delegate.navigate_through_poses(self.ros_path.poses[1:], NavGoal.DEFAULT_BT_XML, self.done_callback)
        self.delegate.navigate_to_pose(path.poses[-1], NavGoal.DEFAULT_BT_XML, self.goal_handle_callback, self.done_callback)

    def done_callback(self, future):
        if future:
            CaBotRclpyUtil.info(F"NavGoal completed result={future.result()}, {self.route_index}/{len(self.navcog_routes)}")
        else:
            CaBotRclpyUtil.info("Could not send goal")
        status = future.result().status if future is not None else None

        # TODO(daisuke): needs to change test case conditions
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.mode == geojson.NavigationMode.Narrow or self.mode == geojson.NavigationMode.Tight:
                self.delegate.activity_log("cabot/navigation", "goal_completed", "NarrowGoal")
            if self.mode == geojson.NavigationMode.Crosswalk:
                self.delegate.activity_log("cabot/navigation", "goal_completed", "CrosswalkGoal")
            if self.mode == geojson.NavigationMode.Standard:
                self.delegate.activity_log("cabot/navigation", "goal_completed", "NavGoal")

        if status == GoalStatus.STATUS_SUCCEEDED and self.route_index + 1 < len(self.navcog_routes):
            self.route_index += 1
            self.enter()
        else:
            if self.mode == geojson.NavigationMode.Tight:
                self.delegate.please_return_position()
            self._is_completed = (status == GoalStatus.STATUS_SUCCEEDED)
            self._is_canceled = (status != GoalStatus.STATUS_SUCCEEDED)

    def reset(self):
        self.mode = None
        super(NavGoal, self).reset()

    def update_goal(self, goal):
        CaBotRclpyUtil.info("Updated goal position")
        # self.delegate.send_goal(goal, self.done_callback)

    def estimate_inner_goal(self, current_pose, current_floor):
        CaBotRclpyUtil.info("NavGoal.estimate_inner_goal is called")
        try:
            g = geoutil.local2global(current_pose, self.anchor)
            min_dist = 100
            min_index = 0
            for index, navcog_route in enumerate(self.navcog_routes):
                p = navcog_route[1].pose.position
                if current_pose.distance_to(geoutil.Point(x=p.x, y=p.y)) < NavGoal.GOAL_XY_THRETHOLD:
                    continue
                route = self.separated_route[index]
                for node_or_link in route:
                    try:
                        if node_or_link.floor == current_floor:
                            dist = node_or_link.distance_to(g)
                            if dist < min_dist:
                                min_dist = dist
                                min_index = index
                    except:  # noqa: #722
                        CaBotRclpyUtil.error(F"{node_or_link}")
            self.route_index = min_index
        except:  # noqa: #722
            CaBotRclpyUtil.error(traceback.format_exc())
            self.route_index = 0

    def match(self, pose, floor):
        # work around, Link.distance_to is not implemented for local geometry
        g = geoutil.local2global(pose, self.anchor)
        for node_or_link in self.navcog_route:
            try:
                # CaBotRclpyUtil.info(F"NavGoal.match distance_to ({node_or_link._id}, ({pose.x}, {pose.y})) = {node_or_link.distance_to(g)}")
                if node_or_link.floor == floor and \
                   node_or_link.distance_to(g) < NavGoal.NEIGHBOR_THRESHOLD:
                    return True
            except:  # noqa: #722
                CaBotRclpyUtil.error(F"{node_or_link}")
        return False

    def completed(self, pose, floor):
        # CaBotRclpyUtil.info(F"NavGoal.completed distance_to ({pose}) = {self.distance_to(pose)}")
        return floor == self._floor and \
            self.distance_to(pose) < Goal.GOAL_XY_THRETHOLD


class TurnGoal(Goal):
    def __init__(self, delegate, goal_node, anchor, yaw, **kwargs):
        goal = geoutil.global2local(goal_node.geometry, anchor)
        self.yaw_target = (- yaw + 90 + anchor.rotate) / 180.0 * math.pi
        CaBotRclpyUtil.info(F"TurnGoal {goal} {yaw} {anchor.rotate} {self.yaw_target}")
        pose = geoutil.Pose(x=goal.x, y=goal.y, r=self.yaw_target)
        pose._angle = Goal.GOAL_ANGLE_THRETHOLD_IN_DEGREE
        pose._floor = goal_node.floor
        super(TurnGoal, self).__init__(delegate, target=pose, **kwargs)

    def _enter(self):
        CaBotRclpyUtil.info("call turn_towards")
        CaBotRclpyUtil.info(F"turn target {str(self.orientation)}")
        self.delegate.turn_towards(self.orientation, self.goal_handle_callback, self.done_callback, 0, 3.0)

    def done_callback(self, result):
        if result:
            CaBotRclpyUtil.info(F"TurnGoal completed {result=}")
            self._is_completed = True
            return
        if self._is_canceled:
            CaBotRclpyUtil.info("TurnGoal not completed but cancelled")
            return

    def match(self, pose, floor):
        # CaBotRclpyUtil.info(F"TurnGoal.match distance_to ({pose}) = {self.distance_to(pose)}")
        return floor == self._floor and \
            self.distance_to(pose) < Goal.GOAL_XY_THRETHOLD

    def completed(self, pose, floor):
        diff = geoutil.diff_angle(self.orientation, pose.orientation)
        # CaBotRclpyUtil.info(F"TurnGoal.completed distance_to ({pose}) = {self.distance_to(pose)}, angle diff {diff}")
        return floor == self._floor and \
            self.distance_to(pose) < Goal.GOAL_XY_THRETHOLD and \
            diff < 0.1


class DoorGoal(Goal):
    APPROACHED_THRESHOLD = 0.25

    def __init__(self, delegate, link, anchor, poi: geojson.POI):
        self._link = link
        self._anchor = anchor
        target = geoutil.TargetPlace(
            x=poi.local_geometry.x + math.cos(link.pose.r)*1.0,
            y=poi.local_geometry.y + math.sin(link.pose.r)*1.0,
            r=link.pose.r,
            angle=45,
            floor=link.floor,
        )
        super(DoorGoal, self).__init__(delegate, target=target)

    def enter(self):
        self.delegate.please_pass_door()
        self.delegate.set_pause_control(True)
        self.delegate.enter_goal(self)

    def check(self, current_pose):
        if self.is_approaching(current_pose):
            return
        elif self.is_approached(current_pose):
            return

        pose_to_door = geoutil.Pose.pose_from_points(current_pose, self)
        CaBotRclpyUtil.info(F"DoorGoal.check distance_to (({self.x}, {self.y}) ({current_pose.x}, {current_pose.y})) = {self.distance_to(current_pose)}"
                            F", {pose_to_door.r} <=> {self.r} - {geoutil.in_angle(pose_to_door, self, 90)}")
        if self.distance_to(current_pose) > 0 and geoutil.in_angle(pose_to_door, self, 90):
            self._is_completed = True

    def exit(self, callback):
        self.delegate.door_passed()
        self.delegate.set_pause_control(False)
        super(DoorGoal, self).exit(callback)

    def match(self, pose, floor):
        gpose = geoutil.local2global(pose, self._anchor)  # work around
        # CaBotRclpyUtil.info(F"DoorGoal.match distance_to ({self._link._id}, ({pose.x}, {pose.y})) = {self._link.distance_to(gpose)}")
        return floor == self._floor and \
            self._link.distance_to(gpose) < NavGoal.NEIGHBOR_THRESHOLD

    def completed(self, pose, floor):
        pose_to_door = geoutil.Pose.pose_from_points(pose, self)
        # CaBotRclpyUtil.info(F"DoorGoal.completed distance_to (({self.x}, {self.y}) ({pose.x}, {pose.y})) = {self.distance_to(pose)}, {pose_to_door.r} <=> {self.r}")
        return floor == self._floor and \
            self.distance_to(pose) < DoorGoal.APPROACHED_THRESHOLD and geoutil.in_angle(pose_to_door, self, 90)


class ElevatorGoal(Goal):
    # using odom frame
    ELEVATOR_SOCIAL_DISTANCE_X = 0.0
    ELEVATOR_SOCIAL_DISTANCE_Y = 0.0
    ELEVATOR_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_elevator.xml"
    LOCAL_ODOM_BT_XML = "package://cabot_bt/behavior_trees/navigate_w_local_odom.xml"
    MATCH_TOLLERANCE = 0.5

    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorGoal, self).__init__(delegate, target=cab_poi, **kwargs)
        self.cab_poi = cab_poi

        def dist(offset):
            return math.sqrt(offset[0] * offset[0] + offset[1] * offset[1])
        self.set_back_distance = dist(cab_poi.set_back)
        self.set_forward_distance = dist(cab_poi.set_forward)

    def nav_params_keys(self):
        return {
            "/cabot/people_speed_control_node": ["social_distance_x", "social_distance_y"],
            "/footprint_publisher": ["footprint_mode"],
        }

    def nav_params(self):
        return {
            "/cabot/people_speed_control_node": {
                "social_distance_x": ElevatorGoal.ELEVATOR_SOCIAL_DISTANCE_X,
                "social_distance_y": ElevatorGoal.ELEVATOR_SOCIAL_DISTANCE_Y
            },
            "/footprint_publisher": {
                "footprint_mode": 1
            }
        }


class ElevatorWaitGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi):
        super(ElevatorWaitGoal, self).__init__(delegate, cab_poi)

    def enter(self):
        CaBotRclpyUtil.info("ElevatorWaitGoal call turn_towards")
        CaBotRclpyUtil.info(F"current pose {str(self.delegate.current_pose)}")
        CaBotRclpyUtil.info(F"cab poi      {str(self.cab_poi.local_geometry)}")
        pose = geoutil.Pose.pose_from_points(self.delegate.current_pose, self.cab_poi.door_geometry)
        CaBotRclpyUtil.info(F"turn target {str(pose)}")
        self.delegate.turn_towards(pose.orientation, self.goal_handle_callback, self.done_callback)

    def done_callback(self, result):
        if result:
            CaBotRclpyUtil.info("ElevatorWaitGoal completed")
            pos = self.cab_poi.where_is_buttons(self.delegate.current_pose)
            self.delegate.please_call_elevator(pos)
            self._is_completed = result
            return
        if self._is_canceled:
            CaBotRclpyUtil.info("ElevatorWaitGoal not completed but cancelled")
            return
        pose = geoutil.Pose.pose_from_points(self.delegate.current_pose, self.cab_poi.door_geometry)
        CaBotRclpyUtil.info(F"turn target {str(pose)}")
        self.delegate.turn_towards(pose.orientation, self.goal_handle_callback, self.done_callback)

    def match(self, pose, floor):
        target_pose = geoutil.Pose.pose_from_points(pose, self.cab_poi.door_geometry)
        same_direction = abs(geoutil.diff_angle(target_pose.orientation, pose.orientation)) < 0.1
        CaBotRclpyUtil.info(F"target pose {str(target_pose)}, same_direction={same_direction}")
        return self.same_floor(floor) and \
            self.distance_to(pose) > ElevatorGoal.MATCH_TOLLERANCE and \
            self.distance_to(pose) < self.set_back_distance + ElevatorGoal.MATCH_TOLLERANCE and \
            not same_direction

    def completed(self, pose, floor):
        target_pose = geoutil.Pose.pose_from_points(self.delegate.current_pose, self.cab_poi.door_geometry)
        same_direction = abs(geoutil.diff_angle(target_pose.orientation, pose.orientation)) < 0.1
        return self.same_floor(floor) and same_direction and self.distance_to(pose) > 0.5


class ElevatorInGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorInGoal, self).__init__(delegate, cab_poi, **kwargs)

    # use default enter to set parameters
    def _enter(self):
        # use odom frame for navigation
        self.delegate.navigate_to_pose(self.to_pose_stamped_msg(frame_id=self.global_map_name), ElevatorGoal.ELEVATOR_BT_XML, self.goal_handle_callback, self.done_callback)

    def done_callback(self, future):
        CaBotRclpyUtil.info("ElevatorInGoal completed")
        status = future.result().status
        self._is_completed = (status == GoalStatus.STATUS_SUCCEEDED)
        self._is_canceled = (status != GoalStatus.STATUS_SUCCEEDED)

    def match(self, pose, floor):
        target_pose = geoutil.Pose.pose_from_points(pose, self.cab_poi.door_geometry)
        same_direction = abs(geoutil.diff_angle(target_pose.orientation, pose.orientation)) < 0.3
        CaBotRclpyUtil.info(f"ElevatorInGoal match: self.r={self.r}, pose.r={target_pose.r}, distance={self.distance_to(pose)}")
        return self.same_floor(floor) and \
            self.distance_to(pose) > ElevatorGoal.MATCH_TOLLERANCE and \
            self.distance_to(pose) < self.set_back_distance + ElevatorGoal.MATCH_TOLLERANCE and \
            (same_direction or self.distance_to(pose) < ElevatorGoal.MATCH_TOLLERANCE*3)

    def completed(self, pose, floor):
        return self.same_floor(floor) and self.distance_to(pose) <= ElevatorGoal.MATCH_TOLLERANCE


class ElevatorTurnGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorTurnGoal, self).__init__(delegate, cab_poi, **kwargs)

    def _enter(self):
        CaBotRclpyUtil.info("call turn_towards")
        pose = geoutil.Pose(x=self.cab_poi.x, y=self.cab_poi.y, r=self.cab_poi.r)
        CaBotRclpyUtil.info(F"turn target {str(pose)}")
        self.delegate.turn_towards(pose.orientation, self.goal_handle_callback, self.done_callback, clockwise=-1)

    def done_callback(self, result):
        if result:
            CaBotRclpyUtil.info(F"ElevatorTurnGoal completed {result=}")
            self._is_completed = True
            return
        if self._is_canceled:
            CaBotRclpyUtil.info("ElevatorTurnGoal not completed but cancelled")
            return

    def match(self, pose, floor):
        CaBotRclpyUtil.info(f"ElevatorTurnGoal match: self.r={self.r}, pose.r={pose.r}, distance={self.distance_to(pose)}")
        return not self.same_direction(pose.orientation) and self.distance_to(pose) < ElevatorGoal.MATCH_TOLLERANCE

    def completed(self, pose, floor):
        return self.same_direction(pose.orientation) and self.distance_to(pose) < ElevatorGoal.MATCH_TOLLERANCE


class ElevatorFloorGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorFloorGoal, self).__init__(delegate, cab_poi, **kwargs)

    def _enter(self):
        self.delegate.goto_floor(self.cab_poi.floor, self.goal_handle_callback, self.done_callback)

    def done_callback(self, result):
        CaBotRclpyUtil.info("ElevatorFloorGoal completed")
        self._is_completed = result

    def match(self, pose, floor):
        CaBotRclpyUtil.info(f"ElevatorFloorGoal match: self.floor={self._floor}, floor={floor}, self.r={self.r}, pose.r={pose.r}, distance={self.distance_to(pose)}")
        return not self.same_floor(floor) and self.same_direction(pose.orientation) and self.distance_to(pose) < 0.5

    def completed(self, pose, floor):
        return self.same_floor(floor) and self.same_direction(pose.orientation) and self.distance_to(pose) < 0.5


class ElevatorOutGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, set_forward=(3.0, 0.0), **kwargs):
        super(ElevatorOutGoal, self).__init__(delegate, cab_poi, **kwargs)
        self.set_forward = set_forward

    def _enter(self):
        CaBotRclpyUtil.info("ElevatorOutGoal _enter")
        pose = self.delegate.current_odom_pose

        start = geometry_msgs.msg.PoseStamped()
        start.header.frame_id = "local/odom"
        start.pose = pose.to_pose_msg()

        CaBotRclpyUtil.info(F"Robot position in odom {str(pose)}")

        pose.x = pose.x + math.cos(pose.r) * self.set_forward[0] - math.sin(pose.r) * self.set_forward[1]
        pose.y = pose.y + math.sin(pose.r) * self.set_forward[0] + math.cos(pose.r) * self.set_forward[1]

        CaBotRclpyUtil.info(F"goal is {str(pose)}")

        end = geometry_msgs.msg.PoseStamped()
        end.header.frame_id = "local/odom"
        end.pose = pose.to_pose_msg()
        path = nav_msgs.msg.Path()
        path.header.frame_id = "local/odom"
        path.poses = [start, end]
        CaBotRclpyUtil.info(F"publish path {str(pose)}")
        self.delegate.publish_path(path, False)

        self.delegate.navigate_to_pose(end, ElevatorGoal.LOCAL_ODOM_BT_XML, self.goal_handle_callback, self.done_callback, namespace='/local')

    def done_callback(self, future):
        CaBotRclpyUtil.info("ElevatorOutGoal completed")
        status = future.result().status
        self._is_completed = (status == GoalStatus.STATUS_SUCCEEDED)
        self._is_canceled = (status != GoalStatus.STATUS_SUCCEEDED)

    def match(self, pose, floor):
        CaBotRclpyUtil.info(f"ElevatorOutGoal match: self.floor={self._floor}, floor={floor}, distance={self.distance_to(pose)}")
        return self.same_floor(floor) and self.distance_to(pose) < self.set_forward_distance

    def completed(self, pose, floor):
        return self.same_floor(floor) and self.distance_to(pose) > self.set_forward_distance


"""
TODO
class EscalatorInGoal(Goal):
    def __init__(self, delegate, links):
        # find a palce before the elevator
        remain = 2.0
        last_link = None
        for link in reversed(links):
            if remain - link.length < 0:
                last_link = link
                break
            remain -= link.length

        self.last_link = last_link
        ratio = remain / last_link.length
        sg = last_link.start_node.local_geometry
        eg = last_link.end_node.local_geometry
        x = sg.x * ratio + eg.x * (1-ratio)
        y = sg.y * ratio + eg.y * (1-ratio)

        print (remain, last_link.length, ratio)
        print sg
        print eg
        print (x, y)

        super(EscalatorInGoal, self).__init__(delegate, angle=0, floor=last_link.floor, x=x, y=y, r=0)

class EscalatorOutGoal(Goal):
    def __init__(self, links):
        pass
"""


# create multiple goals to queue target node
def make_queue_goals(delegate, queue_route, anchor):
    goals = []
    for queue_r_idx, queue_r in enumerate(queue_route):
        if queue_r_idx == 0 or queue_r_idx == len(queue_route)-1:
            # skip node
            continue
        else:
            # find navigation start, end nodes
            if queue_r_idx == 1:
                queue_r_start = queue_route[0]
            else:
                queue_r_start = queue_route[queue_r_idx-1].target_node
            if queue_r_idx == len(queue_route)-2:
                queue_r_end = queue_route[queue_r_idx+1]
            else:
                queue_r_end = queue_route[queue_r_idx+1].source_node
            # add goal to turn for goal
            queue_r_end_pose = geoutil.Pose.pose_from_points(queue_r_end.local_geometry, queue_r_start.local_geometry, backward=True)
            goals.append(QueueTurnGoal(delegate, queue_r.floor, queue_r_end_pose))

            # check if next goal is queue target node (e.g. cashier in store)
            is_target = False
            if queue_r_idx == len(queue_route)-2:
                is_target = True

            # check if link has wait POIs, and get interval if it exists
            queue_interval = None
            queue_waits = list(filter(lambda x: isinstance(x, geojson.QueueWaitPOI), queue_r.pois))
            if queue_waits and len(queue_waits) >= 1:
                queue_interval = queue_waits[0].interval

            # add navigation goal
            goals.append(QueueNavGoal(delegate, [queue_r_start, queue_r, queue_r_end], anchor, queue_interval, is_target, is_exiting=False))

    return goals


# current code assumes queue exit route is narrow, and different BT XML is used for changing footprint
class QueueNavGoal(NavGoal):
    QUEUE_SOCIAL_DISTANCE_X = 0.8
    QUEUE_SOCIAL_DISTANCE_Y = 0.8
    QUEUE_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_queue.xml"
    QUEUE_EXIT_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_queue_exit.xml"

    def __init__(self, delegate, navcog_route, anchor, queue_interval, is_target, is_exiting, **kwargs):
        # set is_last as True if the goal is queue target goal or queue last goal for annoucing arrival information
        super(QueueNavGoal, self).__init__(delegate, navcog_route, anchor, is_last=(is_target or isinstance(self, QueueNavLastGoal)), **kwargs)
        self.queue_interval = queue_interval
        self.is_target = is_target
        self.is_exiting = is_exiting

    def nav_params_keys(self):
        return {
            "/cabot/people_speed_control_node": ["social_distance_x", "social_distance_y"]
        }

    def nav_params(self):
        return {
            "/cabot/people_speed_control_node": {
                "social_distance_x": QueueNavGoal.QUEUE_SOCIAL_DISTANCE_X,
                "social_distance_y": QueueNavGoal.QUEUE_SOCIAL_DISTANCE_Y
            }
        }

    @property
    def is_social_navigation_enabled(self):
        return False

    def _enter(self):
        # change queue_interval setting
        if self.queue_interval is not None:
            self.delegate.current_queue_interval = self.queue_interval

        # publish navcog path
        path = self.ros_path
        path.header.stamp = CaBotRclpyUtil.now().to_msg()
        self.delegate.publish_path(path)
        if self.is_exiting:
            self.delegate.navigate_to_pose(self.to_pose_stamped_msg(frame_id=self.global_map_name), QueueNavGoal.QUEUE_EXIT_BT_XML, self.goal_handle_callback, self.done_callback)
        else:
            self.delegate.navigate_to_pose(self.to_pose_stamped_msg(frame_id=self.global_map_name), QueueNavGoal.QUEUE_BT_XML, self.goal_handle_callback, self.done_callback)

    def done_callback(self, future):
        self._is_completed = future.done()
        if self._is_completed and self.is_target:
            # If robot arrive queue target (e.g. cashier), sleep for a while.
            # User will release handle and robot will stop.
            # TODO: this may not work as expected
            # self._is_completed needs to be set to true after 10 seconds if you want to wait, like L762
            time.sleep(10)


class QueueTurnGoal(Goal):
    def __init__(self, delegate, floor, pose, **kwargs):
        super(QueueTurnGoal, self).__init__(delegate, angle=180, floor=floor, pose_msg=pose.to_pose_msg(), **kwargs)
        self.target_orientation = pose.orientation

    def _enter(self):
        CaBotRclpyUtil.info(F"QueueTurnGoal turn_towards, target {str(self.target_orientation)}")
        self.delegate.turn_towards(self.target_orientation, self.goal_handle_callback, self.done_callback)

    def done_callback(self, result):
        if result:
            CaBotRclpyUtil.info("QueueTurnGoal completed")
            self._is_completed = result
            return
        if self._is_canceled:
            CaBotRclpyUtil.info("QueueTurnGoal not completed but cancelled")
            return
        self.delegate.turn_towards(self.target_orientation, self.goal_handle_callback, self.done_callback)


class QueueNavFirstGoal(QueueNavGoal):
    def __init__(self, delegate, navcog_route, anchor, **kwargs):
        super(QueueNavFirstGoal, self).__init__(delegate, navcog_route, anchor, queue_interval=None, is_target=False, is_exiting=False, **kwargs)

    def _enter(self):
        # initialize flag to call queue start arrived info and queue proceed info
        self.delegate.need_queue_start_arrived_info = True
        self.delegate.need_queue_proceed_info = False
        super(QueueNavFirstGoal, self)._enter()


class QueueNavLastGoal(QueueNavGoal):
    def __init__(self, delegate, navcog_route, anchor, **kwargs):
        super(QueueNavLastGoal, self).__init__(delegate, navcog_route, anchor, queue_interval=None, is_target=False, is_exiting=True, **kwargs)

    def done_callback(self, future):
        super(QueueNavLastGoal, self).done_callback(future)

        # reset queue_interval setting
        if self.delegate.initial_queue_interval is not None:
            self.delegate.current_queue_interval = self.delegate.initial_queue_interval

        # reset flag to call queue start arrived info and queue proceed info
        self.delegate.need_queue_start_arrived_info = False
        self.delegate.need_queue_proceed_info = False
