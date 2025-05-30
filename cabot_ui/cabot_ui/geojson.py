# Copyright (c) 2020, 2024  Carnegie Mellon University and Miraikan
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

"""
MapService GeoJson mapper

MapService: https://github.com/hulop/MapService

Author: Daisuke Sato<daisukes@cmu.edu>
"""
# -*- coding: utf-8 -*-
import sys
import traceback
import enum
import copy
import math
import json
import scipy
import scipy.spatial
import numpy
import numpy.linalg
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import angles
import geometry_msgs.msg
from cabot_ui import geoutil, i18n
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class RouteStruct(enum.IntEnum):
    """ Route Struct Class """
    """ See p.14 of https://www.mlit.go.jp/common/001244374.pdf """
    Sidewalk = 1
    SidewalkNotSeparatedFromRoadway = 2
    Crosswalk = 3
    CrosswalkWithoutSurfaceMarking = 4
    Underpass = 5
    PedestrianBridge = 6
    InFacility = 7
    OtherRouteStruct = 8
    Unknown = 99


class Gradient(enum.IntEnum):
    Unknown = 99
    AlmostFlat = 1
    Up = 2
    Down = 3


class Geometry(object):
    """Geometry class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Geometry subclasses object"""
        if 'type' in dic:
            if dic['type'] == "Point":
                cls = Point
            elif dic['type'] == "LineString":
                cls = LineString
        if cls == Geometry:
            return cls(**dic)
        return cls.marshal(dic)

    def __init__(self, **dic):
        s = super(Geometry, self)
        if self.__class__.mro()[-2] == s.__thisclass__:
            s.__init__()
        else:
            s.__init__(**dic)

        if 'coordinates' in dic:
            self.coordinates = dic['coordinates']
        if 'type' in dic:
            self.geometry_type = dic['type']


class Point(Geometry, geoutil.Latlng):
    """Point class representing global point"""

    @classmethod
    def marshal(cls, dic):
        """marshal Point object"""
        return cls(**dic)

    def __init__(self, **dic):
        c = dic['coordinates']
        super(Point, self).__init__(lat=c[1], lng=c[0], **dic)


class LineString(Geometry):
    """Point class representing global line (start to end)"""
    @classmethod
    def marshal(cls, dic):
        """marshal LineString object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(LineString, self).__init__(**dic)
        self.start = geoutil.Latlng(lat=self.coordinates[0][1], lng=self.coordinates[0][0])
        self.end = geoutil.Latlng(lat=self.coordinates[1][1], lng=self.coordinates[1][0])

    def distance_to(self, point):
        if isinstance(point, geoutil.Latlng):
            return self.nearest_point_on_line(point).distance_to(point)
        raise RuntimeError(F"Need to pass a Point object ({type(point)})")

    def nearest_point_on_line(self, point):
        A = geoutil.latlng2mercator(self.start)
        B = geoutil.latlng2mercator(self.end)
        C = geoutil.latlng2mercator(point)

        # Distance between A and B
        distAB = math.sqrt(math.pow(A.x - B.x, 2) + math.pow(A.y - B.y, 2))

        # Direction vector from A to B
        vecABx = (B.x - A.x) / distAB
        vecABy = (B.y - A.y) / distAB

        # Time from A to C
        timeAC = max(0, min(distAB, vecABx * (C.x - A.x) + vecABy * (C.y - A.y)))

        # LatLng of the point
        x = timeAC * vecABx + A.x
        y = timeAC * vecABy + A.y

        return geoutil.mercator2latlng(geoutil.Point(x=x, y=y))


class Properties(object):
    @classmethod
    def marshal(cls, dic):
        """marshal Properties object"""
        return cls(**dic)

    DEFAULT_VALUES = {
        "hulop_building": None,
        "hulop_major_category": None,
        "hulop_sub_category": None,
        "hulop_minor_category": None,
        "hulop_heading": 0,
        "hulop_angle": 180,
        "hulop_height": 0,
        "hulop_long_description": None,
        "hulop_short_description": None,
        "hulop_description": None,
        "hulop_location_description": None,
        "hulop_content": None,
        "hulop_tags": None,
        "hulop_poi_external_category": None,
        "hulop_show_labels_zoomlevel": None,
        "hulop_road_width": 1.5,
        "rt_struct": RouteStruct.InFacility,
        "vtcl_slope": Gradient.Unknown,
        "vSlope_max": 0,
        }

    def __getattr__(self, name):
        value = self.__dict__.get(name)
        if not value:
            if name in Properties.DEFAULT_VALUES:
                return Properties.DEFAULT_VALUES[name]

            raise AttributeError(F"{self.__class__.__name__}.{name} is invalid")
        return value

    def __init__(self, **dic):
        for key in dic:
            try:
                setattr(self, key, dic[key])
            except:  # noqa E722
                CaBotRclpyUtil.error(F"Cannot use unicode string for a property name: \"{key.encode('utf8')}\"")

    def __str__(self):
        return json.dumps(self.__dict__, sort_keys=True, indent=2)


class LinkKDTree:
    def __init__(self):
        self._link_index = []
        self._link_points = []
        self._link_kdtree = None

    def reset(self):
        self._link_index = []
        self._link_points = []
        self._link_kdtree = None

    def build(self, links):
        for obj in links:
            if obj.start_node and obj.end_node:
                sp = numpy.array([obj.start_node.local_geometry.x, obj.start_node.local_geometry.y])  # noqa E501
                ep = numpy.array([obj.end_node.local_geometry.x, obj.end_node.local_geometry.y])
                self._add_link_index(sp, ep, obj)
        if self._link_points:
            self._link_kdtree = scipy.spatial.KDTree(self._link_points)

    def _add_link_index(self, sp, ep, obj):
        mp = (sp + ep) / 2.0
        self._link_points.append(mp)
        self._link_index.append(obj)
        if numpy.linalg.norm(sp - ep) > 1:
            self._add_link_index(sp, mp, obj)
            self._add_link_index(mp, ep, obj)

    def get_nearest_link(self, node, exclude=None):
        point = node.local_geometry
        latlng = node.geometry
        _, index = self._link_kdtree.query([point.x, point.y], 50)

        min_index = None
        min_dist = 1000
        for i in index:
            if i == len(self._link_index):
                continue
            link = self._link_index[i]
            if exclude is not None and exclude(link):
                continue

            # use local geometry if available, which is more efficient
            if link.local_geometry:
                dist = link.local_geometry.distance_to(point)
            else:
                dist = link.geometry.distance_to(latlng)
            if node.floor is not None:
                if link.start_node.floor != node.floor and \
                   link.end_node.floor != node.floor:
                    dist += 1000
            if dist < min_dist:
                min_dist = dist
                min_index = i

        if min_index is None:
            return (None, None)
        return (self._link_index[min_index], min_dist)


class Object(object):
    """Object class"""

    @classmethod
    def marshal_list(cls, objects):
        """marshal list of Object subclasses objects"""
        temp = []
        for obj in objects:
            temp.append(cls.marshal(obj))
        return temp

    @classmethod
    def marshal_dict(cls, objects):
        """marshal dict of Object subclasses objects"""
        temp = {}
        for key in objects.keys():
            temp[key] = cls.marshal(objects[key])
        return temp

    @classmethod
    def marshal(cls, dic):
        """marshal Object subclasses object"""
        if 'node' in dic:
            cls = Landmark
        else:
            prop = dic['properties'] if 'properties' in dic else None
            if prop is not None:
                if 'node_id' in prop:
                    cls = Node
                if 'link_id' in prop:
                    cls = Link
                if 'facil_id' in prop:
                    cls = Facility

        if cls == Object:
            return cls(**dic)
        return cls.marshal(dic)

    _id_map = {}
    _all_objects = []

    @staticmethod
    def get_object_by_id(_id, func=None):
        """get object having id by callback function, it can be defered"""
        if _id in Object._id_map:
            if isinstance(Object._id_map[_id], list):
                Object._id_map[_id].append(func)
            else:
                if func is not None and callable(func):
                    func(Object._id_map[_id])
                    return None
                return Object._id_map[_id]
        else:
            Object._id_map[_id] = [func]
        return None

    @staticmethod
    def get_objects_by_type(_type):
        """get objects of specified type"""
        temp = []
        for obj in Object._all_objects:
            if isinstance(obj, _type):
                temp.append(obj)
        return temp

    @staticmethod
    def get_objects_by_exact_type(_type):
        """get objects of specified type"""
        temp = []
        for obj in Object._all_objects:
            if type(obj) is _type:
                temp.append(obj)
        return temp

    @staticmethod
    def get_all_objects():
        return Object._all_objects

    @staticmethod
    def _unregister(obj):
        _id = obj._id
        if _id in Object._id_map:
            del Object._id_map[_id]

    @staticmethod
    def _register(obj):
        """store object with id and type"""
        # register with id
        _id = obj._id
        if _id in Object._id_map:
            if isinstance(Object._id_map[_id], list):
                for func in Object._id_map[_id]:
                    if callable(func):
                        func(obj)
                Object._id_map[_id] = obj
                Object._all_objects.append(obj)
            else:
                # raise RuntimeError("duplicate id")
                pass
        else:
            Object._id_map[_id] = obj
            Object._all_objects.append(obj)

    @staticmethod
    def reset_all_objects():
        """reset all state in the objects"""
        for obj in Object._all_objects:
            obj.reset()
        # dirty hack to deal with _TEMP_NODE_
        temp_node = Object.get_object_by_id("_TEMP_NODE_")
        if temp_node:
            Object._unregister(temp_node)

    _kdtree = LinkKDTree()

    @staticmethod
    def get_nearest_link(node, exclude=None):
        return Object._kdtree.get_nearest_link(node, exclude)

    @staticmethod
    def update_anchor_all(anchor):
        """update anchor of all object"""
        Object._kdtree.reset()
        for obj in Object._all_objects:
            obj.update_anchor(anchor)
        Object._kdtree.build(Object.get_objects_by_type(Link))

    def __init__(self, **dic):
        s = super(Object, self)
        if self.__class__.mro()[-2] == s.__thisclass__:
            s.__init__()
        else:
            s.__init__(**dic)

        if 'geometry' in dic:
            self.geometry = Geometry.marshal(dic['geometry'])
        if 'properties' in dic:
            self.properties = Properties.marshal(dic['properties'])
        if '_id' in dic:
            self._id = dic['_id']
            if 'no_registration' not in dic or not dic['no_registration']:
                Object._register(self)
        self.anchor = None
        self.local_geometry = None

    def __str__(self):
        ret = F"{type(self)}, ({hex(id(self))})\n"
        for key in self.__dict__:
            value = getattr(self, key)
            if isinstance(value, Object):
                ret += F"{key}: {type(value)}<{value._id}>\n"
            else:
                ret += F"{key}: {str(value)}\n"

        import inspect
        for method in inspect.getmembers(type(self), predicate=lambda o: isinstance(o, property)):
            ret += F"{method[0]}: {method[1].__get__(self, type(self))}\n"

        return ret

    def __repr__(self):
        return F"{type(self)}<{self._id}>"

    def update_anchor(self, anchor):
        self.anchor = anchor
        if anchor is not None:
            try:
                self.local_geometry = geoutil.global2local(self.geometry, anchor)
            except:  # noqa E722
                CaBotRclpyUtil.error(F"Could not convert geometry: {self.local_geometry}")

    def distance_to(self, point):
        if isinstance(point, geoutil.Point):
            return self.local_geometry.distance_to(point)
        if isinstance(point, geoutil.Latlng):
            return self.geometry.distance_to(point)

    def reset(self):
        pass

    def copy(self):
        return copy.deepcopy(self)


class NavigationMode(enum.Enum):
    Standard = 0
    Narrow = 1
    Tight = 2
    Crosswalk = 5

    @classmethod
    def get_mode(cls, width, rt_struct):
        if width < 1.0:
            return NavigationMode.Tight
        if width < 1.2:
            return NavigationMode.Narrow
        if rt_struct == RouteStruct.Crosswalk or rt_struct == RouteStruct.CrosswalkWithoutSurfaceMarking:
            return NavigationMode.Crosswalk
        return NavigationMode.Standard


class Link(Object):
    """Link class"""
    ROUTE_TYPE_WALKWAY = 1
    ROUTE_TYPE_MOVING_WALKWAY = 2
    ROUTE_TYPE_RAILROAD_CROSSING = 3
    ROUTE_TYPE_ELEVATOR = 4
    ROUTE_TYPE_ESCALATOR = 5
    ROUTE_TYPE_STAIRS = 6
    ROUTE_TYPE_SLOPE = 7
    ROUTE_TYPE_UNKNOWN = 99

    @classmethod
    def marshal(cls, dic):
        """marshal Link subclasses object"""
        if 'properties' in dic:
            prop = dic['properties']
            if 'sourceNode' in prop:
                cls = RouteLink
        if cls == Link:
            return cls(**dic)
        return cls.marshal(dic)

    def __init__(self, **dic):
        super(Link, self).__init__(**dic)
        self.start_node = None
        self.end_node = None
        self.pois = []
        self.floor = 0
        Object.get_object_by_id(self.properties.start_id, self._set_start_node)
        Object.get_object_by_id(self.properties.end_id, self._set_end_node)

    def _set_start_node(self, node):
        self.start_node = node
        self._update()

    def _set_end_node(self, node):
        self.end_node = node
        self._update()

    def _update(self):
        if self.start_node is not None and \
           self.end_node is not None:
            self.floor = (self.start_node.floor + self.end_node.floor)/2.0

    @property
    def is_elevator(self):
        """wheather this links is an elevator or not"""
        return self.properties.route_type == Link.ROUTE_TYPE_ELEVATOR

    @property
    def is_escalator(self):
        """wheather this links is an escalator or not"""
        return self.properties.route_type == Link.ROUTE_TYPE_ESCALATOR

    @property
    def is_leaf(self):
        """wheather this links is a leaf or not"""
        if self.start_node is None or self.end_node is None:
            return False
        return self.start_node.is_leaf or self.end_node.is_leaf

    @property
    def navigation_mode(self):
        return NavigationMode.get_mode(
            width=self.properties.hulop_road_width,
            rt_struct=self.properties.rt_struct
            )

    @property
    def length(self):
        """distance from start to end"""
        if self.start_node is None or self.end_node is None:
            return float('nan')
        return self.start_node.geometry.distance_to(self.end_node.geometry)

    @property
    def pose(self):
        return geoutil.Pose.pose_from_points(self.start_node.local_geometry, self.end_node.local_geometry)

    @property
    def gradient(self):
        return self.properties.vtcl_slope

    @property
    def max_gradient(self):
        return self.properties.vSlope_max

    def register_poi(self, poi):
        self.pois.append(poi)

    def update_anchor(self, anchor):
        if isinstance(self.geometry, Point):
            super().update_anchor(anchor)
            return
        self.anchor = anchor
        if anchor is not None:
            try:
                start = geoutil.global2local(self.geometry.start, anchor)
                end = geoutil.global2local(self.geometry.end, anchor)
                self.local_geometry = geoutil.Line(start=start, end=end)
            except:  # noqa E722
                CaBotRclpyUtil.error(traceback.format_exc())

    def within_link(self, point):
        return self.local_geometry.within_link(point)


class RouteLink(Link):
    """Route Link class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Directed Link object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(RouteLink, self).__init__(no_registration=True, **dic)
        self.source_node = None
        self.target_node = None
        Object.get_object_by_id(self.properties.sourceNode, self._set_source_node)
        Object.get_object_by_id(self.properties.targetNode, self._set_target_node)
        Object.get_object_by_id(self._id, self._found_link)

    def update_anchor(self, anchor):
        super().update_anchor(anchor)
        # here is to work around to get POIs on the first temp link
        if self.is_temp:
            # the source_node should be updated with an anchor beforehand
            link, _ = Object.get_nearest_link(self.source_node)
            self.pois = link.pois

    def _set_source_node(self, node):
        self.source_node = node

    def _set_target_node(self, node):
        self.target_node = node

    def _found_link(self, link):
        self.update_anchor(link.anchor)
        self.original_link = link
        self.pois = link.pois

    @property
    def is_temp(self):
        return self._id.startswith("_TEMP_LINK")

    @property
    def pose(self):
        return geoutil.Pose.pose_from_points(self.source_node.local_geometry, self.target_node.local_geometry)

    @property
    def gradient(self):
        # if link is reveresed, flip the gradient
        if self.start_node != self.source_node:
            if super().gradient == Gradient.Up:
                return Gradient.Down
            elif super().gradient == Gradient.Down:
                return Gradient.Up
        return super().gradient


class Node(Object):
    """Node class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Node object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(Node, self).__init__(**dic)
        self.links = []
        for i in range(1, 100):
            attr = F"link{i}_id"
            if hasattr(self.properties, attr):
                Object.get_object_by_id(getattr(self.properties, attr), self._add_link)

        self.facility = None
        Facility.get_facility_by_id(self._id, self._set_facility)

    def _add_link(self, link):
        self.links.append(link)

    def _set_facility(self, facility):
        self.facility = facility

    @property
    def floor(self):
        if hasattr(self.properties, 'floor'):
            return self.properties.floor
        return 0

    @property
    def is_leaf(self):
        """wheather this node is the end of leaf link"""
        return len(self.links) == 1

    @property
    def is_elevator(self):
        """wheather this node is connected to elevator link"""
        res = False
        for link in self.links:
            res = res or link.is_elevator
        return res


class Entrance(geoutil.TargetPlace):
    def __init__(self, facility, i, node):
        self.facility = facility
        self.node = node
        self.node_id = node._id
        self._id = f"{facility._id}_ent{i}"
        self.name = i18n.localized_attr(facility.properties, f"ent{i}_n")
        super(Entrance, self).__init__(r=0, x=0, y=0, angle=60, floor=self.floor)
        Object._register(self)

    @property
    def floor(self):
        return self.node.floor

    def update_anchor(self, anchor):
        self.anchor = anchor

    def reset(self):
        self.reset_target()

    def set_target(self, link):
        gpoint = link.geometry.nearest_point_on_line(self.node.geometry)
        lpoint = geoutil.global2local(gpoint, self.anchor)
        self.update_pose(lpoint, link.pose.r + math.pi)
        return gpoint

    def approaching_statement(self):
        return None

    def approached_statement(self):
        p1 = geoutil.q_from_points(self, self.node.local_geometry)
        diff = geoutil.q_diff(self.quaternion, p1)
        _, _, angle = euler_from_quaternion(diff)
        CaBotRclpyUtil.debug(f"Entrance.approacehd_statement {diff} {angle}")
        if math.fabs(angle) < math.pi / 4:
            direction = "IN_FRONT"
        else:
            direction = "RIGHT_SIDE" if angle < 0 else "LEFT_SIDE"
        i18n_direction = i18n.localized_string(direction)
        return i18n.localized_string("APPROACEHD_TO_FACILITY").format(self.facility.name, i18n_direction)

    def passed_statement(self):
        return None

    # distance is adjusted to by the TargetPoint orientation
    #                     <- T Target (orientation)
    #                        |
    #                        |
    # robot R ---distance--- o
    #
    # adjusted = True
    def distance_to(self, robot):
        dist_TR = robot.distance_to(self)
        pose_TR = geoutil.Pose.pose_from_points(self, robot)
        yaw = geoutil.diff_angle(self.orientation, pose_TR.orientation)
        adjusted = dist_TR * math.cos(yaw)
        CaBotRclpyUtil.debug(f"Entrance.distance_to dist={dist_TR}, yaw={yaw}, adjusted={adjusted}")
        return adjusted


class Facility(Object):
    """Facility class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Facility subclasses object"""
        if 'properties' in dic:
            prop = dic['properties']
            if 'hulop_major_category' in prop:
                category = prop['hulop_major_category']
                if category == '_nav_poi_':
                    cls = POI
        if cls == Facility:
            return cls(**dic)
        return cls.marshal(dic)

    def __init__(self, **dic):
        super(Facility, self).__init__(**dic)
        self.entrances = []
        for i in range(1, 100):
            attr = F"ent{i}_node"
            if hasattr(self.properties, attr):
                Facility._id_map[getattr(self.properties, attr)] = self
                Object.get_object_by_id(getattr(self.properties, attr), self._add_entrance(i))
        if hasattr(self.properties, "hulop_tags"):
            hulop_tags = getattr(self.properties, "hulop_tags")
            self._is_read = ("read" in hulop_tags) if hulop_tags else False

    def _add_entrance(self, i):
        def inner_func(node):
            self.entrances.append(Entrance(self, i, node))
        return inner_func

    @property
    def floor(self):
        if self.entrances:
            return self.entrances[0].floor  # assume all entrance is same floor
        return 0

    @property
    def name(self):
        return i18n.localized_attr(self.properties, "name")

    @property
    def name_pron(self):
        return i18n.localized_attr(self.properties, "name_hira", only_if="ja")

    @property
    def long_description(self):
        return i18n.localized_attr(self.properties, "hulop_long_description")

    @property
    def is_read(self):
        return self._is_read

    _id_map = {}

    @staticmethod
    def get_facility_by_id(_id, func=None):
        """get facility having id by callback function, it can be defered"""
        if _id in Facility._id_map:
            if isinstance(Facility._id_map[_id], list):
                Facility._id_map[_id].append(func)
            else:
                if func is not None and callable(func):
                    func(Facility._id_map[_id])
                    return None
                return Facility._id_map[_id]
        else:
            Facility._id_map[_id] = [func]
        return None


class POI(Facility, geoutil.TargetPlace):
    """POI class"""

    @classmethod
    def marshal(cls, dic):
        """marshal POI object"""
        if 'properties' in dic:
            prop = dic['properties']
            if 'hulop_sub_category' in prop:
                category = prop['hulop_sub_category']
                if category == '_nav_door_':
                    cls = DoorPOI
                if category == '_nav_info_':
                    cls = InfoPOI
                if category == '_cabot_speed_':
                    cls = SpeedPOI
                if category == '_nav_elevator_cab_':
                    cls = ElevatorCabPOI
                if category == '_nav_queue_wait_':
                    cls = QueueWaitPOI
                if category == '_nav_queue_target_':
                    cls = QueueTargetPOI

        if cls == POI:
            return cls(**dic)
        return cls.marshal(dic)

    def __init__(self, **dic):
        if 'properties' in dic:
            self.prop = dic['properties']
            r = (-self._get_prop('hulop_heading') + 90) / 180.0 * math.pi
            angle = self._get_prop('hulop_angle')

        super(POI, self).__init__(r=r, x=0, y=0, angle=angle, floor=self.floor, **dic)

        self.sub_category = self.properties.hulop_sub_category \
            if hasattr(self.properties, 'hulop_sub_category') else ""
        self.minor_category = self.properties.hulop_minor_category \
            if hasattr(self.properties, 'hulop_minor_category') else ""

        # backward compatibility
        self.local_pose = self

    def _get_prop(self, key):
        return self.prop[key] if key in self.prop else Properties.DEFAULT_VALUES[key]

    @property
    def floor(self):
        return self._get_prop('hulop_height')

    def approaching_statement(self):
        return None

    def approached_statement(self):
        return None

    def passed_statement(self):
        return None

    def update_anchor(self, anchor):
        super(POI, self).update_anchor(anchor)
        if anchor is not None:
            rad = (-self.properties.hulop_heading + 90 + anchor.rotate) / 180.0 * math.pi
            self.update_pose(self.local_geometry, rad)

    def reset(self):
        self.reset_target()

    # distance is adjusted to by the TargetPoint orientation
    #                     <- T Target (orientation)
    #                        |
    #                        |
    # robot R ---distance--- o
    #
    def distance_to(self, robot, adjusted=False):
        dist_TR = super(POI, self).distance_to(robot)
        if not adjusted:
            return dist_TR
        pose_TR = geoutil.Pose.pose_from_points(self, robot)
        yaw = geoutil.diff_angle(self.orientation, pose_TR.orientation)
        adjusted = dist_TR * math.cos(yaw)
        # CaBotRclpyUtil.debug(f"dist={dist_TR}, yaw={yaw}, adjusted={adjusted}")
        return adjusted


class DoorPOI(POI):
    """POI class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Door POI object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(DoorPOI, self).__init__(**dic)

    @property
    def title(self):
        if self.is_auto:
            return i18n.localized_string("AUTO_DOOR")
        else:
            return i18n.localized_string("DOOR")

    @property
    def is_auto(self):
        """wheather this is auto door or not"""
        return self.minor_category is not None and \
            '_flag_auto_' in self.minor_category

    def approaching_statement(self):
        return i18n.localized_string("DOOR_POI_APPROACHING", self.title)


class InfoPOI(POI):
    """Nav Info POI class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Info POI object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(InfoPOI, self).__init__(**dic)
        if self.minor_category not in ["_priority_low_", "_priority_high_", "_priority_normal_", "_priority_required_", "", None]:
            CaBotRclpyUtil.error(f"Invalid value for hulop_minor_category: {self.minor_category}")

    def get_minor_category(self):
        return self.minor_category

    def approached_statement(self):
        return self.name


class SpeedPOI(POI):
    """Cabot Speed POI class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Speed POI object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(SpeedPOI, self).__init__(**dic)
        self.limit = float(self.properties.hulop_content)


class ElevatorCabPOI(POI):
    """Elevator Cab POI class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Elevator Cab POI object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(ElevatorCabPOI, self).__init__(**dic)
        self.set_back = (3.0, 0.0)
        self.set_forward = (3.0, 0.0)
        self.door = (1.0, 0.0)
        if self.properties.hulop_content:
            try:
                hulop_content_json = json.loads(self.properties.hulop_content)
                if "set_back" in hulop_content_json:
                    self.set_back = hulop_content_json["set_back"]
                if "set_forward" in hulop_content_json:
                    self.set_forward = hulop_content_json["set_forward"]
                if "door" in hulop_content_json:
                    self.door = hulop_content_json["door"]
                if "buttons" in hulop_content_json:
                    self.buttons = hulop_content_json["buttons"]
            except:  # noqa E722
                traceback.print_exc(file=sys.std_out)

    @property
    def door_geometry(self):
        x = self.x + math.cos(self.r) * self.door[0] - math.sin(self.r) * self.door[1]
        y = self.y + math.sin(self.r) * self.door[0] + math.cos(self.r) * self.door[1]
        return geoutil.Point(x=x, y=y)

    def where_is_buttons(self, pose):
        x = self.x + math.cos(self.r) * self.buttons[0] - math.sin(self.r) * self.buttons[1]
        y = self.y + math.sin(self.r) * self.buttons[0] + math.cos(self.r) * self.buttons[1]

        b_pos = geoutil.Point(x=x, y=y)
        b_pose = geoutil.Pose.pose_from_points(pose, b_pos)
        dir = angles.shortest_angular_distance(pose.r, b_pose.r)

        CaBotRclpyUtil.info(f"{pose=}, {b_pos=}, {b_pose=}, {dir=}")

        if abs(dir) > math.pi / 3 * 2:
            return "BACK"
        elif abs(dir) > math.pi / 3:
            if dir > 0:
                return "LEFT"
            elif dir < 0:
                return "RIGHT"
        elif abs(dir) < math.pi / 10:
            return "FRONT"
        elif dir > 0:
            return "FRONT_LEFT"
        elif dir < 0:
            return "FRONT_RIGHT"

        CaBotRclpyUtil.error("should not happen")
        return None


class QueueWaitPOI(POI):
    """Queue Wait POI class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Queue TaWaitrget POI object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(QueueWaitPOI, self).__init__(**dic)
        self.interval = 1.0
        hulop_content_json = json.loads(self.properties.hulop_content)
        if "interval" in hulop_content_json:
            self.interval = float(hulop_content_json["interval"])
        self.is_copied = False
        self.link_orientation = None

#    def approached_statement(self):
#        return "queue wait point"

    def register_link(self, link):
        source_pose = geoutil.Pose.pose_from_points(link.source_node.local_geometry,
                                                    link.target_node.local_geometry)
        quat = quaternion_from_euler(0, 0, source_pose.r)

        self.link_orientation = geometry_msgs.msg.Quaternion()
        self.link_orientation.x = quat[0]
        self.link_orientation.y = quat[1]
        self.link_orientation.z = quat[2]
        self.link_orientation.w = quat[3]

    def copy_to_link(self, link, local_geometry_x, local_geometry_y):
        copied_poi = copy.deepcopy(self)
        copied_poi.x = local_geometry_x
        copied_poi.y = local_geometry_y
        copied_poi.local_geometry.x = local_geometry_x
        copied_poi.local_geometry.y = local_geometry_y
        copied_poi.geometry = geoutil.local2global(copied_poi.local_geometry, copied_poi.anchor)
        copied_poi.link_orientation = self.link_orientation

        link.register_poi(copied_poi)
        self.is_copied = True
        return copied_poi


class QueueTargetPOI(POI):
    """Queue Target POI class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Queue Target POI object"""
        return cls(**dic)

    def __init__(self, **dic):
        super(QueueTargetPOI, self).__init__(**dic)
        self.enter_node = None
        self.exit_node = None
        hulop_content_json = json.loads(self.properties.hulop_content)
        Object.get_object_by_id(hulop_content_json["enter"], self._set_enter_node)
        Object.get_object_by_id(hulop_content_json["exit"], self._set_exit_node)

    def _set_enter_node(self, node):
        self.enter_node = node

    def _set_exit_node(self, node):
        self.exit_node = node


class Landmark(Facility):
    """Landmark class"""

    @classmethod
    def marshal(cls, dic):
        """marshal Landmark object"""
        return cls(**dic)

    def __init__(self, **dic):
        self._id = dic['node']+"_landmark"
        super(Landmark, self).__init__(**dic)
