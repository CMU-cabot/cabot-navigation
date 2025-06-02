// Copyright (c) 2025  Carnegie Mellon University, IBM Corporation, and others
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

#include <cabot_navigation2/limit_fov_obstacle_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::LimitFovObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace cabot_navigation2
{

LimitFovObstacleLayer::LimitFovObstacleLayer()
{
}

LimitFovObstacleLayer::~LimitFovObstacleLayer()
{
}

void LimitFovObstacleLayer::raytraceFreespace(const nav2_costmap_2d::Observation & clearing_observation, double * min_x, double * min_y, double * max_x, double * max_y)
{
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_WARN(logger_, "sensor is out of map");
    return;
  }

  touch(ox, oy, min_x, min_y, max_x, max_y);

  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
  unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);

  const sensor_msgs::msg::PointCloud2 & cloud = *(clearing_observation.cloud_);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    double a = wx - ox;
    double b = wy - oy;

    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    unsigned int x1, y1;
    if (!worldToMap(wx, wy, x1, y1)) {
      continue;
    }

    MarkCell marker(costmap_, nav2_costmap_2d::FREE_SPACE);
    raytraceSuperCoverLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);

    updateRaytraceBounds(
      ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
      clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
      max_y);
  }
}

}  // namespace cabot_navigation2
