// Copyright (c) 2024  Kufusha Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef PEDESTRIAN_PLUGIN__GEOMETRY_UTILS_HPP_
#define PEDESTRIAN_PLUGIN__GEOMETRY_UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>

geometry_msgs::msg::Point operator+(
  geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
  geometry_msgs::msg::Point ret;

  ret.x = p1.x + p2.x;
  ret.y = p1.y + p2.y;
  ret.z = p1.z + p2.z;

  return ret;
}

#endif  // PEDESTRIAN_PLUGIN__GEOMETRY_UTILS_HPP_
