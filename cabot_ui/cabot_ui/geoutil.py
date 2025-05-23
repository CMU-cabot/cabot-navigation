# Copyright (c) 2020, 2022  Carnegie Mellon University
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
Geography Utility

Author: Daisuke Sato<daisukes@cmu.edu>
"""

import math
import numpy
import numpy.linalg
import yaml

import geometry_msgs.msg
from pyproj import CRS, Geod
from pyproj import Transformer
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

from tf_transformations import quaternion_multiply, euler_from_quaternion, quaternion_from_euler


def msg_from_q(q):
    """get Quaternion message from array"""
    msg = geometry_msgs.msg.Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def msg_from_p(p):
    """get Point message from array"""
    msg = geometry_msgs.msg.Point()
    msg.x = p[0]
    msg.y = p[1]
    msg.z = p[2]
    return msg


def msg_from_pq(p, q):
    """get Pose message from pose and quaternion array"""
    pose = geometry_msgs.msg.Pose()
    pose.position = msg_from_p(p)
    pose.orientation = msg_from_q(q)
    return pose


def q_from_msg(msg):
    """get array from Quaternion message"""
    if not isinstance(msg, geometry_msgs.msg.Quaternion):
        raise RuntimeError("msg should be a Quaternion message")
    return [msg.x, msg.y, msg.z, msg.w]


def p_from_msg(msg):
    """get array from Point message"""
    if isinstance(msg, Point):
        msg = msg.to_point_msg()
    if not isinstance(msg, geometry_msgs.msg.Point):
        raise RuntimeError("msg should be a Point message")
    return [msg.x, msg.y, msg.z]


def q_from_points(msg1, msg2):
    # Convert points to numpy arrays
    p1 = p_from_msg(msg1)
    p2 = p_from_msg(msg2)

    # Calculate angles relative to the x-axis
    delta_theta = numpy.arctan2(p2[1]-p1[1], p2[0]-p1[0])

    # Construct the quaternion
    q_w = numpy.cos(delta_theta / 2)  # real part
    q_z = numpy.sin(delta_theta / 2)  # z-axis part, since rotation is around z-axis in 2D

    # Since we're rotating in 2D, the quaternion's x and y components are 0
    q_x, q_y = 0, 0

    return (q_x, q_y, q_z, q_w)


def q_inverse(q):
    """get inverse quaternion"""
    return [q[0], q[1], q[2], -q[3]]


def q_diff(q1, q2):
    rot = [0, 0, 1, 0]  # 180 degree turn
    return quaternion_multiply(quaternion_multiply(q2, rot), q_inverse(q1))


def get_yaw(q):
    _, _, yaw = euler_from_quaternion(q)
    return yaw


# pose1 and pose2 is supporsed to be facing
def in_angle(pose1, pose2, margin_in_degree):
    """
    pose1: robot pose
    pose2: POI pose
    margin_in_degree: POI angle margin in degree
    return True if robot orientation is in POI angle
    """
    if isinstance(pose1, Pose):
        pose1 = pose1.to_pose_msg()
    if isinstance(pose2, Pose):
        pose2 = pose2.to_pose_msg()

    margin = margin_in_degree / 180.0 * math.pi
    p1_p2 = q_from_points(pose1.position, pose2.position)

    quat1 = q_from_msg(pose1.orientation)
    quat2 = q_from_msg(pose2.orientation)

    # check orientation diff from pose2 orientation to rotated pose1 orientation
    _, _, yaw1 = euler_from_quaternion(q_diff(quat2, quat1))

    # check orientation diff from pose2 orientation to rotated p1->p2 orientation
    _, _, yaw2 = euler_from_quaternion(q_diff(quat2, p1_p2))

    # check both in the margin
    return abs(yaw1) <= margin and abs(yaw2) <= margin


# quat1 and quat2 is supporsed to be facing
def diff_in_angle(quat1, quat2, margin_in_degree):
    """
    margin_in_degree: angle margin in degree
    return True if two poses in margin
    """
    margin = margin_in_degree / 180.0 * math.pi

    # check orientation diff from pose2 orientation to rotated pose1 orientation
    _, _, yaw1 = euler_from_quaternion(q_diff(quat2, quat1))

    # check both in the margin
    return abs(yaw1) <= margin


# diff of 180 degree turn of quat1 and quat2
def diff_angle(msg1, msg2):
    """
    return yaw difference
    """
    quat1 = q_from_msg(msg1)
    quat2 = q_from_msg(msg2)
    quat3 = quaternion_multiply(quat2, q_inverse(quat1))
    _, _, yaw1 = euler_from_quaternion(quat3)
    return yaw1


def get_rotation(src, target):
    q1 = q_from_msg(src)
    q2 = q_from_msg(target)
    _, _, yaw = euler_from_quaternion(quaternion_multiply(q2, q_inverse(q1)))
    return yaw


def normalize_angle(ang):
    while (ang <= -math.pi):
        ang += 2 * math.pi
    while (ang > math.pi):
        ang -= 2 * math.pi
    return ang


def get_projected_point_to_line(point, line_point, line_orientation):
    """
    calculate the point that project a given point perpendicular to given line segment
    point: input point in numpy.array
    line_point: input line segment start point in numpy.array
    line_orientation: input line segment orientation in geometry_msgs.msg.Quaternion
    return point in numpy.array
    """
    line_quat_arr = [line_orientation.x, line_orientation.y, line_orientation.z, line_orientation.w]
    line_euler = euler_from_quaternion(line_quat_arr)
    line_vec = numpy.array([math.cos(line_euler[2]), math.sin(line_euler[2])])
    line_to_point_vec = point - line_point

    dot_product = numpy.dot(line_to_point_vec, line_vec)
    norm_line_vec = numpy.linalg.norm(line_vec)
    projection = dot_product/norm_line_vec
    line_cross_vector = line_vec * (projection/norm_line_vec)
    return numpy.array([line_point[0]+line_cross_vector[0], line_point[1]+line_cross_vector[1]])


def is_forward_point(pose1, pose2):
    """
    calculate if the given point is in forward direction of the given pose
    pose1: robot pose
    pose2: POI pose
    return True if the point is in forward direction of the robot
    """
    q1 = q_from_msg(pose1.orientation)
    q2 = q_from_points(pose2, pose1)
    return diff_in_angle(q1, q2, 90)


class Point(object):
    """represent a 2D point"""
    def __init__(self, **kwargs):
        s = super(Point, self)
        if self.__class__.mro()[-2] == s.__thisclass__:
            s.__init__()
        else:
            s.__init__(**kwargs)

        if 'xy' in kwargs:
            self.x = kwargs['xy'].x
            self.y = kwargs['xy'].y
        else:
            self.x = kwargs['x']
            self.y = kwargs['y']

    def __repr__(self):
        return F"({self.x:.2f}, {self.y:.2f})"

    def distance_to(self, obj):
        """euclid distance between the instance point and the passed point"""
        if isinstance(obj, Point):
            return math.sqrt((self.x - obj.x) * (self.x - obj.x)
                             + (self.y - obj.y) * (self.y - obj.y))
        raise RuntimeError(F"need to pass a Point object ({type(obj)})")

    def interpolate(self, obj, ratio):
        """get a point between the instance point and the passed point
        defined by the 'ratio'. 'ratio'==1 means the passed point itself"""
        return Point(x=self.x * (1.0 - ratio) + obj.x * ratio,
                     y=self.y * (1.0 - ratio) + obj.y * ratio)

    def to_point_msg(self):
        """convert Point into ROS point msg"""
        point = geometry_msgs.msg.Point()
        point.x = self.x
        point.y = self.y
        point.z = 0.0
        return point


class Line(object):
    """represent a 2D point"""
    def __init__(self, **kwargs):
        s = super(Line, self)
        if self.__class__.mro()[-2] == s.__thisclass__:
            s.__init__()
        else:
            s.__init__(**kwargs)

        if 'start' in kwargs:
            self.start = kwargs['start']
        if 'end' in kwargs:
            self.end = kwargs['end']

    def __repr__(self):
        return F"({self.start}, {self.end})"

    def within_link(self, point):
        A = self.start
        B = self.end
        dx = B.x - A.x
        dy = B.y - A.y
        dot = (point.x - A.x) * dx + (point.y - A.y) * dy
        if dot < 0:
            return False
        len_sq = dx * dx + dy * dy
        if dot > len_sq:
            return False
        return True

    def distance_to(self, point):
        if isinstance(point, Point):
            return self.nearest_point_on_line(point).distance_to(point)
        raise RuntimeError(F"Need to pass a Point object ({type(point)})")

    def nearest_point_on_line(self, obj):
        A = self.start
        B = self.end
        C = obj

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

        return Point(x=x, y=y)


class Pose(Point):
    """represent a 2D pose. init with x, y, and r"""
    def __init__(self, **kwargs):
        if 'pose_msg' in kwargs:
            pose = Pose.from_pose_msg(kwargs['pose_msg'])
            super(Pose, self).__init__(x=pose.x, y=pose.y, **kwargs)
            self.r = pose.r
        else:
            super(Pose, self).__init__(**kwargs)
            self.r = kwargs['r']

    def __repr__(self):
        return F"{type(self)}({self.x:#8.2f}, {self.y:#8.2f})[{self.r/math.pi*180:#8.2f} deg]"

    @staticmethod
    def pose_from_points(p1, p2, backward=False):
        p1_p2 = q_from_points(p1, p2)
        if backward:
            _, _, yaw = euler_from_quaternion(q_diff([0, 0, 0, 1], p1_p2))
            return Pose(x=p1.x, y=p1.y, r=yaw)
        else:
            _, _, yaw = euler_from_quaternion(p1_p2)
            return Pose(x=p1.x, y=p1.y, r=yaw)

    @property
    def orientation(self):
        orientation = geometry_msgs.msg.Quaternion()
        q = quaternion_from_euler(0.0, 0.0, self.r)
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
        return orientation

    @property
    def quaternion(self):
        return quaternion_from_euler(0.0, 0.0, self.r)

    @classmethod
    def from_pose_msg(cls, msg):
        """instanciate Pose from ROS pose msg"""
        pose = msg
        if isinstance(msg, geometry_msgs.msg.PoseStamped):
            pose = msg.pose
        quat = pose.orientation
        quat_arr = [quat.x, quat.y, quat.z, quat.w]
        euler = euler_from_quaternion(quat_arr)
        return Pose(x=pose.position.x, y=pose.position.y, r=euler[2])

    def to_pose_msg(self):
        """convert Pose into ROS pose msg"""
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        q = quaternion_from_euler(0.0, 0.0, self.r)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def to_pose_stamped_msg(self, frame_id="map"):
        """convert Pose into ROS pose msg"""
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        q = quaternion_from_euler(0.0, 0.0, self.r)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose


class Latlng(object):
    """represent a global coordinate. init with lat, and lng"""
    def __init__(self, **kwargs):
        s = super(Latlng, self)
        if self.__class__.mro()[-2] == s.__thisclass__:
            s.__init__()
        else:
            s.__init__(**kwargs)

        self.lat = kwargs['lat']
        self.lng = kwargs['lng']

    def __repr__(self):
        return F"[{self.lat:.7f}, {self.lng:.7f}]"

    def distance_to(self, obj):
        """distance between the instance latlng and the passed latlng"""
        if isinstance(obj, Latlng):
            geo = Geod(ellps='WGS84')
            (_, _, dist) = geo.inv(self.lng, self.lat, obj.lng, obj.lat)
            return dist
        raise RuntimeError(F"need to pass a Latlng object ({type(obj)})")


class LatlngPose(Latlng):
    def __init__(self, latlng, r):
        self.lat = latlng.lat
        self.lng = latlng.lng
        self.r = r

    def __repr__(self):
        return F"[{self.lat:.7f}, {self.lng:.7f}] @ {self.r:.2f}"


class Anchor(Latlng):
    """represent an anchor point. init with lat, lng, and rotate"""
    def __init__(self, **kwargs):
        super(Anchor, self).__init__(**kwargs)
        self.rotate = kwargs['rotate']

    def __repr__(self):
        return F"[{self.lat:.7f}, {self.lng:.7f}]({self.rotate:.2f})"


# Define the CRS objects
crs_4326 = CRS('epsg:4326')
crs_3857 = CRS('epsg:3857')

# Create transformers using the CRS objects
transformer4326_3857 = Transformer.from_crs(crs_4326, crs_3857, always_xy=True)
transformer3857_4326 = Transformer.from_crs(crs_3857, crs_4326, always_xy=True)


def latlng2mercator(latlng):
    """convert a LatLng point into a Mercator point"""
    (x, y) = transformer4326_3857.transform(latlng.lng, latlng.lat)
    return Point(x=x, y=y)


def mercator2latlng(mercator):
    """convert a Mercatro point into a LatLng point"""
    (lng, lat) = transformer3857_4326.transform(mercator.x, mercator.y)
    return Latlng(lat=lat, lng=lng)


def get_point_resolution(anchor):
    """get a resolution at an anchor point"""
    RADIUS = 6378137
    return 1.0 / math.cosh(anchor.y / RADIUS)


def mercator2xy(src_mercator, anchor):
    """convert a Mercator point into a local point in the anchor coordinate"""
    res = get_point_resolution(anchor)
    dx = (src_mercator.x - anchor.x) * res
    dy = (src_mercator.y - anchor.y) * res
    rad = anchor.rotate / 180.0 * math.pi
    c = math.cos(rad)
    s = math.sin(rad)
    x = dx * c - dy * s
    y = dx * s + dy * c
    return Point(x=x, y=y)


def xy2mercator(src_xy, anchor):
    """convert a local point in the anchor coordinate into a Mercator point"""
    r = get_point_resolution(anchor)
    x = src_xy.x
    y = src_xy.y
    rad = - anchor.rotate / 180.0 * math.pi
    c = math.cos(rad)
    s = math.sin(rad)
    dx = (x * c - y * s) / r
    dy = (x * s + y * c) / r
    return Point(x=anchor.x + dx, y=anchor.y + dy)


def global2local(latlng, anchor):
    """convert a global point into a local point in the anchor coordinate"""
    temp = latlng2mercator(anchor)
    anchor.x = temp.x
    anchor.y = temp.y
    mer = latlng2mercator(latlng)
    xy = mercator2xy(mer, anchor)
    return xy


def local2global(xy, anchor):
    """convert a local point in the anchor coordinate into the global point"""
    temp = latlng2mercator(anchor)
    anchor.x = temp.x
    anchor.y = temp.y
    mer = xy2mercator(xy, anchor)
    latlng = mercator2latlng(mer)
    if hasattr(xy, 'r'):
        r = xy.r - (90 + anchor.rotate) / 180.0 * math.pi
        while r > math.pi:
            r -= math.pi*2
        while r < -math.pi:
            r += math.pi*2
        latlngr = LatlngPose(latlng, r)
        return latlngr
    else:
        return latlng


def get_anchor(anchor_file=None):
    """get anchor"""
    if anchor_file is None:
        return None

    with open(anchor_file) as file:
        data = yaml.safe_load(file)

    CaBotRclpyUtil.info(F"data={data}")
    if 'anchor' not in data:
        return None
    temp = data["anchor"]
    lat = temp["latitude"]
    lng = temp["longitude"]
    rot = temp["rotate"]
    anchor = Anchor(lat=lat, lng=lng, rotate=rot)
    CaBotRclpyUtil.info(F"data={data['anchor']}")
    CaBotRclpyUtil.info(F"anchor={anchor}")
    return anchor


# this class should be used with Point
class TargetPlace(Pose):
    def __init__(self, **kwargs):
        if 'target' in kwargs:
            target = kwargs['target']
            super(TargetPlace, self).__init__(x=target.x, y=target.y, r=target.r, **kwargs)
            self._angle = target._angle
            self._floor = target._floor
        else:
            super(TargetPlace, self).__init__(**kwargs)
            self._angle = kwargs['angle']
            self._floor = kwargs['floor']

        self.reset_target()

    def same_floor(self, floor, tollerance=0.1):
        return abs(self._floor - floor) < tollerance

    def same_direction(self, orientation, tollerance=0.1):
        return abs(diff_angle(self.orientation, orientation)) < tollerance

    def reset_target(self):
        self._was_approaching = False
        self._pose_approaching = None

        self._was_approached = False
        self._pose_approached = None

        self._was_passed = False
        self._pose_passed = None

    def update_pose(self, point, rotate):
        self.x = point.x
        self.y = point.y
        self.r = normalize_angle(rotate)

    APPROACHING_THRETHOLD = 5.0
    APPROACHED_THRETHOLD = 1.0
    PASSED_THRETHOLD = 1.0

    def in_angle(self, pose):
        return in_angle(pose, self, self._angle)

    def is_approaching(self, pose):
        """the pose is approaching to the poi"""
        if self._was_approaching:
            return False

        if not self.in_angle(pose):
            return False

        if self.distance_to(pose) < TargetPlace.APPROACHING_THRETHOLD:
            self._was_approaching = True
            self._pose_approaching = pose
            return True

        return False

    def is_approached(self, pose):
        if self._was_approached:
            return False

        if not self.in_angle(pose):
            return False

        if self.distance_to(pose) < TargetPlace.APPROACHED_THRETHOLD:
            self._was_approached = True
            self._pose_approached = pose
            return True

        return False

    def is_passed(self, pose):
        if self._was_passed:
            return False

        if self._was_approached and \
           self._pose_approached is not None and \
           self._pose_approached.distance_to(pose) > TargetPlace.PASSED_THRETHOLD and \
           math.fabs(self.distance_to(pose)) > TargetPlace.PASSED_THRETHOLD:
            self._was_passed = True
            self._pose_passed = pose
            return True

        return False
