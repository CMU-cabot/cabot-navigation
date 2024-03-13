# Copyright (c) 2022  Carnegie Mellon University
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

import math
import inspect
import numpy
import time

from cabot_ui import geoutil, geojson
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

import tf_transformations
import nav_msgs.msg
import geometry_msgs.msg

from action_msgs.msg import GoalStatus

from cabot_common import util


class GoalInterface(object):
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
    CaBotRclpyUtil.info(F"{groute}")

    if len(groute) == 0:
        CaBotRclpyUtil.error("groute length should not be 0")
        return []
    if len(groute) == 1:
        # if route is only a node
        return [NavGoal(delegate, groute, anchor, is_last=True)]

    goals = []
    route_objs = []
    index = 1
    narrow = groute[1].is_narrow

    while index < len(groute):
        link_or_node = groute[index]
        route_objs.append(link_or_node)
        index += 1
        if not isinstance(link_or_node, geojson.RouteLink):
            continue
        link = link_or_node

        # there is a manual door
        # find manual door
        doors = list(filter(lambda x: isinstance(x, geojson.DoorPOI) and not x.is_auto, link.pois))
        if doors:
            if len(doors) > 1:
                # will not support
                CaBotRclpyUtil.warn("don't put multiple door pois into a link")

            # set goal 1 meter behind the door
            if len(route_objs) >= 2:
                goals.append(NavGoal(delegate, route_objs[:-1], anchor, target_poi=doors[0], set_back=(1.0, 0.0)))
            # CaBotRclPyUtil.info(goals[-1])
            # ask user to pass the door
            goals.append(DoorGoal(delegate, link, anchor, doors[0]))
            # CaBotRclPyUtil.info(goals[-1])
            route_objs = []

        # find queue target
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
                goals.append(ElevatorInGoal(delegate, src_cabs[0]))
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
            goals.append(ElevatorOutGoal(delegate, dest_cabs[0], set_forward=dest_cabs[0].set_forward))
            # CaBotRclPyUtil.info(goals[-1])
            route_objs = [link]

        if link.is_narrow and not narrow:
            narrow = True
            goals.append(NavGoal(delegate, route_objs[:-1], anchor))
            route_objs = [link]

        if (not link.is_narrow) and narrow:
            narrow = False
            goals.append(NarrowGoal(delegate, route_objs[:-1], anchor))
            route_objs = [link]

        # TODO: escalator
    if len(route_objs) > 0:
        if narrow:
            narrow = False
            goals.append(NarrowGoal(delegate, route_objs, anchor, is_last=True))
        else:
            goals.append(NavGoal(delegate, route_objs, anchor, is_last=True))

    if yaw is not None:
        goal_node = groute[-1]  # should be Node
        goals.append(TurnGoal(delegate, goal_node, anchor, yaw))

    CaBotRclpyUtil.info(F"goals: {goals}")
    return goals


def create_ros_path(navcog_route, anchor, global_map_name):
    """convert a NavCog path to ROS path"""
    # convert route to points
    points = []
    if len(navcog_route) == 0:
        CaBotRclpyUtil.error("create_ros_path, navcog_route length should not be 0")
        return points
    CaBotRclpyUtil.info(F"create_ros_path, {str(navcog_route)}")
    last_index = len(navcog_route)-1

    def convert(g, a=anchor):
        return geoutil.global2local(g, a)

    for (index, item) in enumerate(navcog_route):
        if index == 0 and isinstance(item.geometry, geojson.LineString):
            # if the first item is link, add the source node
            points.append(convert(item.source_node.geometry))
        elif index == last_index:
            if isinstance(item.geometry, geojson.Point):
                # if navcog_route only has one Node
                if len(navcog_route) == 1:
                    points.append(convert(item.geometry))
                # if last item is Point (Node), it would be same as the previous link target node
                continue

        # TODO: This is a hulistic rule to deal with the last leaf link towards doorway in corridor
        # It needs to check if the last link has perpendicular turn
        # if the link is a leaf of the graph and short
        # if isinstance(item, geojson.RouteLink):
        #     if item.is_leaf and item.length < 3.0:
        #         continue

        if isinstance(item.geometry, geojson.Point):
            points.append(convert(item.geometry))
        elif isinstance(item.geometry, geojson.LineString):
            points.append(convert(item.target_node.geometry))
        else:
            CaBotRclpyUtil.info("geometry is not point or linestring {item.geometry}")
        CaBotRclpyUtil.info(F"{index}: {str(points)}")

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

    CaBotRclpyUtil.info(F"path {path}")
    return (path, path.poses[-1] if len(path.poses) > 0 else None)


def estimate_next_goal(goals, current_pose, current_floor):
    for i in range(len(goals), 0, -1):
        goal = goals[i-1]
        if goal.completed(pose=current_pose, floor=current_floor):
            continue
        if goal.match(pose=current_pose, floor=current_floor):
            return (goal, i-1)
    return (None, -1)  # might be reached the goal


class Goal(geoutil.TargetPlace):
    GOAL_XY_THRETHOLD = 0.5
    GOAL_ANGLE_THRETHOLD_IN_DEGREE = 15

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

    def reset(self):
        self._is_completed = False
        self._is_canceled = False

    def enter(self):
        if self._is_canceled:
            CaBotRclpyUtil.info(f"{self} enter called, but already cancelled")
            return
        self.delegate.enter_goal(self)

    def check(self, current_pose):
        pass

    def update_goal(self, goal):
        pass

    @property
    def is_completed(self):
        return self._is_completed

    @property
    def is_canceled(self):
        return self._is_canceled

    @property
    def need_to_announce_arrival(self):
        return False

    def exit(self):
        self.delegate.exit_goal(self)

    @property
    def current_statement(self):
        return self._current_statement

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

    def goal_handle_callback(self, handle):
        self._handles.append(handle)
        if self._is_canceled:
            self.cancel()

    def cancel(self, callback=None):
        try:
            self._cancel(callback)
        except:  # noqa: #722
            import traceback
            self._logger.error(traceback.format_exc())

    def _cancel(self, callback=None):
        self._is_canceled = True

        if len(self._handles) > 0:
            handle = self._handles.pop(0)
            future = handle.cancel_goal_async()

            def done_callback(future):
                self._logger.info(f"cancel future result = {future.result}")
                self.delegate._process_queue.append((self.cancel, callback))
            future.add_done_callback(done_callback)
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


class NavGoal(Goal):
    DEFAULT_BT_XML = "package://cabot_bt/behavior_trees/navigate_w_replanning_and_recovery.xml"
    NEIGHBOR_THRESHOLD = 1.0

    def __init__(self, delegate, navcog_route, anchor, target_poi=None, set_back=(0, 0), **kwargs):
        if navcog_route is None or len(navcog_route) == 0:
            raise RuntimeError("navcog_route should have one more object")
        if anchor is None:
            raise RuntimeError("anchor should be provided")

        # to prevent callback when doing cancel/resume for area change
        self.prevent_callback = False

        # need init global_map_name for initialization
        self.global_map_name = delegate.global_map_name()
        self.navcog_route = navcog_route
        self.anchor = anchor
        (self.ros_path, last_pose) = create_ros_path(self.navcog_route, self.anchor, self.global_map_name)
        self.pois = self._extract_pois()
        self.handle = None

        floor = None
        last_obj = navcog_route[-1]
        if isinstance(last_obj, geojson.Node):
            floor = last_obj.floor
        elif isinstance(last_obj, geojson.RouteLink):
            floor = last_obj.end_node.floor

        # if target_poi is specified, the last_pose will be the target_poi
        if target_poi is not None:
            last_pose = target_poi.to_pose_msg()

            # if set_back is specified too, move the last_pose based on the last link direction
            if set_back[0] > 0 or set_back[1] > 0:
                i = 1
                while isinstance(last_obj, geojson.Node) and i < len(navcog_route):
                    i += 1
                    last_obj = navcog_route[-i]
                if not isinstance(last_obj, geojson.RouteLink):
                    raise RuntimeError("There should be at least one link towards target POI,"
                                       "if it is provided with set_back (={}) \n===POI===\n{}\n========="
                                       .format(set_back, str(target_poi)))

                backward = geoutil.Pose.pose_from_points(last_obj.source_node.local_geometry,
                                                         last_obj.target_node.local_geometry, backward=True)

                CaBotRclpyUtil.info(F"set_back {backward.r:.4} * ({str(set_back)})")

                last_pose.position.x += math.cos(backward.r) * set_back[0] - math.sin(backward.r) * set_back[1]
                last_pose.position.y += math.sin(backward.r) * set_back[0] + math.cos(backward.r) * set_back[1]
            self.ros_path.poses[-1].pose.position.x = last_pose.position.x
            self.ros_path.poses[-1].pose.position.y = last_pose.position.y

        super(NavGoal, self).__init__(delegate, angle=180, floor=floor, pose_msg=last_pose, **kwargs)

        self._need_to_announce_arrival = self.is_last
        if 'need_to_announce_arrival' in kwargs:
            self._need_to_announce_arrival = bool(kwargs['need_to_announce_arrival'])

        self._goal_name_pron = None
        self._goal_description = None
        if self._need_to_announce_arrival:
            if not isinstance(navcog_route[-1], geojson.Node):
                CaBotRclpyUtil.info(navcog_route[-1])
                return
            CaBotRclpyUtil.info(str(navcog_route[-1]))
            if not navcog_route[-1].facility:
                return

            if navcog_route[-1].facility.name_pron:
                self._goal_name_pron = navcog_route[-1].facility.name_pron
            else:
                self._goal_name_pron = navcog_route[-1].facility.name
            self._goal_description = navcog_route[-1].facility.long_description
            if self.goal_name_pron:
                CaBotRclpyUtil.info(F"{self.goal_name_pron.encode('utf-8')}")
            if self.goal_description:
                CaBotRclpyUtil.info(F"{self.goal_description.encode('utf-8')}")

    @property
    def need_to_announce_arrival(self):
        return self._need_to_announce_arrival

    @property
    def goal_name_pron(self):
        return self._goal_name_pron

    @property
    def goal_description(self):
        return self._goal_description

    def _extract_pois(self):
        """extract pois along the route"""
        temp = []
        for (_, item) in enumerate(self.navcog_route):
            if isinstance(item, geojson.RouteLink):
                print(item._id)
                for poi in item.pois:
                    print("  ", type(poi), poi._id)
                temp.extend(item.pois)
        return temp

    def enter(self):
        CaBotRclpyUtil.info("NavGoal enter")
        # reset social distance setting if necessary
        if self.delegate.initial_social_distance is not None and self.delegate.current_social_distance is not None \
            and (self.delegate.initial_social_distance.x != self.delegate.current_social_distance.x or
                 self.delegate.initial_social_distance.y != self.delegate.current_social_distance.y):
            msg = self.delegate.initial_social_distance
            self.delegate.set_social_distance_pub.publish(msg)

        CaBotRclpyUtil.info("NavGoal set social distance")
        # publish navcog path
        path = self.ros_path
        path.header.stamp = CaBotRclpyUtil.now().to_msg()
        self.delegate.publish_path(path)
        CaBotRclpyUtil.info("NavGoal publish path")
        super(NavGoal, self).enter()

        # wanted a path (not only a pose) in planner plugin, but it is not possible
        # bt_navigator will path only a pair of consecutive poses in the path to the plugin
        # so we use navigate_to_pose and planner will listen the published path
        # self.delegate.navigate_through_poses(self.ros_path.poses[1:], NavGoal.DEFAULT_BT_XML, self.done_callback)
        self.delegate.navigate_to_pose(self.ros_path.poses[-1], NavGoal.DEFAULT_BT_XML, self.goal_handle_callback, self.done_callback)

    def done_callback(self, future):
        if self.prevent_callback:
            self.prevent_callback = False
            return

        CaBotRclpyUtil.info(F"NavGoal completed result={future.result()}")
        status = future.result().status
        self._is_completed = (status == GoalStatus.STATUS_SUCCEEDED)
        self._is_canceled = (status != GoalStatus.STATUS_SUCCEEDED)

    def update_goal(self, goal):
        CaBotRclpyUtil.info("Updated goal position")
        # self.delegate.send_goal(goal, self.done_callback)

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

    def enter(self):
        super(TurnGoal, self).enter()
        CaBotRclpyUtil.info("call turn_towards")
        CaBotRclpyUtil.info(F"turn target {str(self.orientation)}")
        self.delegate.turn_towards(self.orientation, self.goal_handle_callback, self.done_callback)

    def done_callback(self, result):
        if result:
            CaBotRclpyUtil.info("TurnGoal completed")
            self._is_completed = result
            return
        if self._is_canceled:
            CaBotRclpyUtil.info("TurnGoal not completed but cancelled")
            return
        self.delegate.turn_towards(self.orientation, self.goal_handle_callback, self.done_callback)

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
    APPROACHED_THRESHOLD = 0.5

    def __init__(self, delegate, link, anchor, poi):
        self._link = link
        self._anchor = anchor
        target = geoutil.TargetPlace(
            x=link.end_node.local_geometry.x,
            y=link.end_node.local_geometry.y,
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
        # CaBotRclpyUtil.info(F"DoorGoal.check distance_to (({self.x}, {self.y}) ({current_pose.x}, {current_pose.y})) = {self.distance_to(current_pose)}"
        #                     F", {pose_to_door.r} <=> {self.r} - {geoutil.in_angle(pose_to_door, self, 90)}")
        if self.distance_to(current_pose) < DoorGoal.APPROACHED_THRESHOLD and geoutil.in_angle(pose_to_door, self, 90):
            self._is_completed = True

    def exit(self):
        self.delegate.door_passed()
        self.delegate.set_pause_control(False)
        super(DoorGoal, self).exit()

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
    ELEVATOR_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_elevator.xml"
    LOCAL_ODOM_BT_XML = "package://cabot_bt/behavior_trees/navigate_w_local_odom.xml"

    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorGoal, self).__init__(delegate, target=cab_poi, **kwargs)
        self.cab_poi = cab_poi


class ElevatorWaitGoal(ElevatorGoal):
    ELEVATOR_SOCIAL_DISTANCE_X = 0.0
    ELEVATOR_SOCIAL_DISTANCE_Y = 0.0

    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorWaitGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        # change social distance setting
        msg = geometry_msgs.msg.Point()
        msg.x = self.ELEVATOR_SOCIAL_DISTANCE_X
        msg.y = self.ELEVATOR_SOCIAL_DISTANCE_X
        self.delegate.set_social_distance_pub.publish(msg)

        super(ElevatorWaitGoal, self).enter()

        CaBotRclpyUtil.info("call turn_towards")
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


class ElevatorInGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorInGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        super(ElevatorInGoal, self).enter()
        # use odom frame for navigation
        self.delegate.navigate_to_pose(self.to_pose_stamped_msg(frame_id=self.global_map_name), ElevatorGoal.ELEVATOR_BT_XML, self.goal_handle_callback, self.done_callback)

    def done_callback(self, future):
        CaBotRclpyUtil.info("ElevatorInGoal completed")
        status = future.result().status
        self._is_completed = (status == GoalStatus.STATUS_SUCCEEDED)
        self._is_canceled = (status != GoalStatus.STATUS_SUCCEEDED)


class ElevatorTurnGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorTurnGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        super(ElevatorTurnGoal, self).enter()
        CaBotRclpyUtil.info("call turn_towards")
        pose = geoutil.Pose(x=self.cab_poi.x, y=self.cab_poi.y, r=self.cab_poi.r)
        CaBotRclpyUtil.info(F"turn target {str(pose)}")
        self.delegate.turn_towards(pose.orientation, self.goal_handle_callback, self.done_callback, clockwise=-1)

    def done_callback(self, result):
        if result:
            CaBotRclpyUtil.info("ElevatorTurnGoal completed")
            self._is_completed = result
            return
        if self._is_canceled:
            CaBotRclpyUtil.info("ElevatorTurnGoal not completed but cancelled")
            return
        pose = geoutil.Pose(x=self.cab_poi.x, y=self.cab_poi.y, r=self.cab_poi.r)
        CaBotRclpyUtil.info(F"turn target {str(pose)}")
        self.delegate.turn_towards(pose.orientation, self.goal_handle_callback, self.done_callback)


class ElevatorFloorGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorFloorGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        super(ElevatorFloorGoal, self).enter()
        self.delegate.goto_floor(self.cab_poi.floor, self.goal_handle_callback, self.done_callback)

    def done_callback(self, result):
        CaBotRclpyUtil.info("ElevatorFloorGoal completed")
        self._is_completed = result


class ElevatorOutGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, set_forward=(3.0, 0.0), **kwargs):
        super(ElevatorOutGoal, self).__init__(delegate, cab_poi, **kwargs)
        self.set_forward = set_forward

    def enter(self):
        super(ElevatorOutGoal, self).enter()
        CaBotRclpyUtil.info("ElevatorOutGoal enter")
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

        # reset social distance setting
        if self.delegate.initial_social_distance is not None:
            msg = self.delegate.initial_social_distance
            self.delegate.set_social_distance_pub.publish(msg)


class NarrowGoal(NavGoal):
    NARROW_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_narrow.xml"
    LITTLE_NARROW_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_little_narrow.xml"

    def __init__(self, delegate, navcog_route, anchor, **kwargs):
        # if one of the link is very narrow, it needs to announce
        self._need_narrow_announce = False
        for link_or_node in navcog_route:
            CaBotRclpyUtil.info(F"{link_or_node}")
            if not isinstance(link_or_node, geojson.RouteLink):
                continue
            self._need_narrow_announce = self._need_narrow_announce or link_or_node.need_narrow_announce

        super(NarrowGoal, self).__init__(delegate, navcog_route, anchor, **kwargs)

    def enter(self):
        CaBotRclpyUtil.info("NarrowGoal enter")
        # reset social distance setting if necessary
        if self.delegate.initial_social_distance is not None and self.delegate.current_social_distance is not None \
            and (self.delegate.initial_social_distance.x != self.delegate.current_social_distance.x or
                 self.delegate.initial_social_distance.y != self.delegate.current_social_distance.y):
            msg = self.delegate.initial_social_distance
            self.delegate.set_social_distance_pub.publish(msg)

        CaBotRclpyUtil.info("NarrowGoal set social distance")
        # publish navcog path
        path = self.ros_path
        path.header.stamp = CaBotRclpyUtil.now().to_msg()
        self.delegate.publish_path(path)
        CaBotRclpyUtil.info("NarrowGoal publish path")

        if self._need_narrow_announce:
            self.delegate.please_follow_behind()
            self.wait_for_announce()
        else:
            self.narrow_enter(NarrowGoal.LITTLE_NARROW_BT_XML)

    def done_callback(self, future):
        CaBotRclpyUtil.info("NarrowGoal completed")
        status = future.result().status
        self._is_canceled = (status != GoalStatus.STATUS_SUCCEEDED)
        if self._is_canceled:
            return
        if self._need_narrow_announce:
            self.delegate.please_return_position()
            self.wait_next_navi(future)
        else:
            self._is_completed = (status == GoalStatus.STATUS_SUCCEEDED)

    def narrow_enter(self, bt):
        super(NavGoal, self).enter()
        # wanted a path (not only a pose) in planner plugin, but it is not possible
        # bt_navigator will path only a pair of consecutive poses in the path to the plugin
        # so we use navigate_to_pose and planner will listen the published path
        # basically the same as a NavGoal, use BT_XML that makes the footprint the same as an elevator to pass through narrow spaces
        # self.delegate.navigate_through_poses(self.ros_path.poses[1:], NavGoal.DEFAULT_BT_XML, self.done_callback)
        self.delegate.navigate_to_pose(self.ros_path.poses[-1], bt, self.goal_handle_callback, self.done_callback)

    @util.setInterval(5, times=1)
    def wait_for_announce(self):
        self.narrow_enter(NarrowGoal.NARROW_BT_XML)

    @util.setInterval(5, times=1)
    def wait_next_navi(self, future):
        status = future.result().status
        self._is_completed = (status == GoalStatus.STATUS_SUCCEEDED)


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
    QUEUE_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_queue.xml"
    QUEUE_EXIT_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_queue_exit.xml"

    def __init__(self, delegate, navcog_route, anchor, queue_interval, is_target, is_exiting, **kwargs):
        # set is_last as True if the goal is queue target goal or queue last goal for annoucing arrival information
        super(QueueNavGoal, self).__init__(delegate, navcog_route, anchor, is_last=(is_target or isinstance(self, QueueNavLastGoal)), **kwargs)
        self.queue_interval = queue_interval
        self.is_target = is_target
        self.is_exiting = is_exiting

    def enter(self):
        # change queue_interval setting
        if self.queue_interval is not None:
            self.delegate.current_queue_interval = self.queue_interval

        # publish navcog path
        path = self.ros_path
        path.header.stamp = CaBotRclpyUtil.now().to_msg()
        self.delegate.publish_path(path)
        super(NavGoal, self).enter()
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

    def enter(self):
        super(QueueTurnGoal, self).enter()
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
    QUEUE_SOCIAL_DISTANCE_X = 0.8
    QUEUE_SOCIAL_DISTANCE_Y = 0.8

    def __init__(self, delegate, navcog_route, anchor, **kwargs):
        super(QueueNavFirstGoal, self).__init__(delegate, navcog_route, anchor, queue_interval=None, is_target=False, is_exiting=False, **kwargs)

    def enter(self):
        # change social distance setting
        msg = geometry_msgs.msg.Point()
        msg.x = self.QUEUE_SOCIAL_DISTANCE_X
        msg.y = self.QUEUE_SOCIAL_DISTANCE_Y
        self.delegate.set_social_distance_pub.publish(msg)

        # initialize flag to call queue start arrived info and queue proceed info
        self.delegate.need_queue_start_arrived_info = True
        self.delegate.need_queue_proceed_info = False

        super(QueueNavFirstGoal, self).enter()


class QueueNavLastGoal(QueueNavGoal):
    def __init__(self, delegate, navcog_route, anchor, **kwargs):
        super(QueueNavLastGoal, self).__init__(delegate, navcog_route, anchor, queue_interval=None, is_target=False, is_exiting=True, **kwargs)

    def done_callback(self, future):
        super(QueueNavLastGoal, self).done_callback(future)

        # reset social distance setting
        if self.delegate.initial_social_distance is not None:
            msg = self.delegate.initial_social_distance
            self.delegate.set_social_distance_pub.publish(msg)

        # reset queue_interval setting
        if self.delegate.initial_queue_interval is not None:
            self.delegate.current_queue_interval = self.delegate.initial_queue_interval

        # reset flag to call queue start arrived info and queue proceed info
        self.delegate.need_queue_start_arrived_info = False
        self.delegate.need_queue_proceed_info = False
