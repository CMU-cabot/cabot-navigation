// Copyright (c) 2020  Carnegie Mellon University
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

#ifndef CABOT_NAVIGATION2__NAVCOG_PATH_UTIL_HPP_
#define CABOT_NAVIGATION2__NAVCOG_PATH_UTIL_HPP_

#include <vector>

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#define FIRST_LINK_THRETHOLD (0.0)

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;

namespace cabot_navigation2
{
struct PathWidth
{
  double left;
  double right;
  double length;
};

struct PathEstimateOptions
{
  // 1 = left, 0 = center, -1 = right
  double path_adjusted_center = 0;
  double path_adjusted_minimum_path_width = 1.0;
  double path_min_width = 0.5;
  double path_width = 2.0;
  double robot_radius = 0.45;
  double safe_margin = 0.05;
  double path_length_to_width_factor = 5;
};

Path normalizedPath(const Path & path);
Path adjustedPathByStart(
  const Path & path,
  const PoseStamped & pose);

std::vector<PathWidth> estimatePathWidthAndAdjust(
  Path & path,
  nav2_costmap_2d::Costmap2D * costmap,
  PathEstimateOptions options);

PoseStamped nearestPointFromPointOnLine(PoseStamped p, PoseStamped l1, PoseStamped l2);
double distance(PoseStamped p1, PoseStamped p2);

PathWidth estimateWidthAt(
  double x, double y, double yaw,
  nav2_costmap_2d::Costmap2D * costmap,
  PathEstimateOptions options);
double normalized_diff(double a, double b);
void removeOutlier(std::vector<PathWidth> & estimate);
bool has_collision(PoseStamped p, const nav2_costmap_2d::Costmap2D * costmap, const int cost_threshold);
Path adjustedPathByCollision(
  const Path & path,
  const nav2_costmap_2d::Costmap2D * costmap, const int cost_threshold);
}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__NAVCOG_PATH_UTIL_HPP_
