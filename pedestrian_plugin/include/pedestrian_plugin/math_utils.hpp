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

#ifndef PEDESTRIAN_PLUGIN__MATH_UTILS_HPP_
#define PEDESTRIAN_PLUGIN__MATH_UTILS_HPP_

#include <cmath>

namespace math
{

constexpr double RAD_TO_DEG = 180.0 / M_PI;

inline int getDistanceIndex(const double dist, const double resolution)
{
  return std::round(dist / resolution);
}

inline int getAngleIndex(const double deg, const double resolution)
{
  return std::round(deg / resolution);
}

inline double rad2deg(const double rad)
{
  return rad * RAD_TO_DEG;
}

inline double normalizeDeg(const double deg)
{
  double normalized_deg = std::fmod(deg, 360.0);
  if (normalized_deg < 0.0) {
    normalized_deg += 360.0;
  }
  return normalized_deg;
}

inline double getDistance(const double x1, const double y1, const double x2, const double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;

  return std::sqrt(dx * dx + dy * dy);
}

inline double getAngleRad(const double x1, const double y1, const double x2, const double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;

  return std::atan2(dy, dx);
}

inline double getAngleDeg(const double x1, const double y1, const double x2, const double y2)
{
  return rad2deg(getAngleRad(x1, y1, x2, y2));
}

inline double roundDistance(const double distance_m, const double resolution)
{
  return std::round(distance_m / resolution) * resolution;
}

inline double roundAngleDeg(const double deg, const double resolution)
{
  return normalizeDeg(std::round(deg / resolution) * resolution);
}

}  // namespace math

#endif  // PEDESTRIAN_PLUGIN__MATH_UTILS_HPP_
