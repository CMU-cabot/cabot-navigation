# ******************************************************************************
#  Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# ******************************************************************************

import math


def isCenterPointInShape(center_point, vertex_points):
    # Check if the center of circle is in a polygon
    # Crossing Number Algorithm
    count = 0
    num_vertex = len(vertex_points)
    for i in range(num_vertex):
        p0 = vertex_points[i]
        p1 = vertex_points[(i+1) % num_vertex]

        # is the vector from p0 to p1 upward/downward?
        is_upward_vector = (p0['y'] <= center_point['y']) and (p1['y'] > center_point['y'])
        is_downward_vector = (p0['y'] > center_point['y']) and (p1['y'] <= center_point['y'])

        if is_upward_vector or is_downward_vector:
            crossing_point_x = p0['x'] + (center_point['y']-p0['y']) * (p1['x']-p0['x']) / (p1['y']-p0['y'])
            if center_point['x'] < crossing_point_x:
                count += 1
    return count % 2 == 1  # true if odd


def isPolygonTouchingCircle(center_point, radius, vertex_points):
    num_vertex = len(vertex_points)
    for i in range(num_vertex):
        isCrossed = isLineTouchingCircle(center_point, radius, vertex_points[i], vertex_points[(i+1) % num_vertex])
        if isCrossed:
            return True
    return False


def isLineTouchingCircle(center_point, radius, v0, v1):
    # Check if the line touches edge of a circle
    c0x = center_point['x'] - v0['x']
    c0y = center_point['y'] - v0['y']
    lx = v1['x'] - v0['x']
    ly = v1['y'] - v0['y']

    # calculate distance from center of circle to line
    distance_circ_line = abs((c0x*ly - c0y*lx) / math.sqrt(lx**2+ly**2))
    if distance_circ_line >= radius:
        # potential collision if this condition is false
        # no collision if true
        return False

    c1x = center_point['x'] - v1['x']
    c1y = center_point['y'] - v1['y']
    dot0 = lx*c0x + ly*c0y
    dot1 = lx*c1x + ly*c1y
    if dot0*dot1 <= 0.:
        # this condition is another expression of "int(dot0 >= 0) ^ int(dot1 >= 0)"
        # true because each endpoints lie on opposite sides of the center of circle
        # no need to validate if these points are outside of circle. this function is part of a touching polygon validation
        return True
    len_c0 = math.sqrt(c0x**2 + c0y**2)
    len_c1 = math.sqrt(c1x**2 + c1y**2)
    if (len_c0 >= radius) ^ (len_c1 >= radius):
        return True
    return False


def isCircleCollidingWithPolygon(center_point, radius, vertex_points):
    return isPolygonTouchingCircle(center_point, radius, vertex_points) or isCenterPointInShape(center_point, vertex_points)


if __name__ == "__main__":
    circle_center_point = {'x': 3., 'y': 0.}
    circle_radius = 2.3
    polygon_vertex_points = [{'x': 0., 'y': 1.}, {'x': -1., 'y': -1.}, {'x': 1., 'y': -1.}]
    res = isCircleCollidingWithPolygon(circle_center_point, circle_radius, polygon_vertex_points)
    print(res)
