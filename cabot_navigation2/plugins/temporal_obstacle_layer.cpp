// Copyright (c) 2025  Carnegie Mellon University
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

// TemporalObstacleLayer introduce temporal obstacle handling in the costmap.
// Unlike the ObstacleLayer, it allows obstacles to be gradually raised and decayed over time.

#include "cabot_navigation2/temporal_obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::Observation;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace cabot_navigation2
{

TemporalObstacleLayer::TemporalObstacleLayer()
: nav2_costmap_2d::ObstacleLayer(),
  temp_costmap_(NULL),
  temp_costmap2_(NULL)
{
}

TemporalObstacleLayer::~TemporalObstacleLayer()
{
}

void TemporalObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(), "TemporalObstacleLayer::onInitialize");

  declareParameter("decaying_rate", rclcpp::ParameterValue(25));
  declareParameter("rising_rate", rclcpp::ParameterValue(25));

  node->get_parameter(name_ + "." + "decaying_rate", decaying_rate_);
  node->get_parameter(name_ + "." + "rising_rate", rising_rate_);

  nav2_costmap_2d::ObstacleLayer::onInitialize();
}

void TemporalObstacleLayer::initMaps(unsigned int size_x, unsigned int size_y)
{
  std::unique_lock<mutex_t> lock(*getMutex());
  Costmap2D::initMaps(size_x, size_y);
  delete[] temp_costmap_;
  delete[] temp_costmap2_;
  temp_costmap_ = new unsigned char[size_x * size_y];
  temp_costmap2_ = new unsigned char[size_x * size_y];
}

void TemporalObstacleLayer::resetMaps()
{
  auto node = node_.lock();
  std::unique_lock<mutex_t> lock(*getMutex());
  Costmap2D::resetMaps();
}


// Here is almost the sme as the ObstacleLayer::updateBounds, but we use a temporary costmap
void
TemporalObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  auto node = node_.lock();

  // raytrace freespace
  // for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
  //   RCLCPP_INFO(node->get_logger(), "Clearing obstacles");
  //   raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  // }
  memset(temp_costmap_, 0, size_x_ * size_y_ * sizeof(unsigned char));

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin();
    it != observations.end(); ++it)
  {
    const Observation & obs = *it;

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too low, we won't add it
      if (pz < min_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too low");
        continue;
      }

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist =
        (px -
        obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
        (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_max_range) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      // if the point is too close, do not conisder it
      if (sq_dist < sq_obstacle_min_range) {
        RCLCPP_DEBUG(logger_, "The point is too close");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      // mark the temp_costmap_ rather than updating the costmap_ directly
      temp_costmap_[index] = 1;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

// this is the same as the ObstacleLayer::raytraceFreespace, but we use a temporary costmap
void
TemporalObstacleLayer::raytraceFreespace(
  const Observation & clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::msg::PointCloud2 & cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_WARN(
      logger_,
      "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). "
      "The costmap cannot raytrace for it.",
      ox, oy,
      origin_x_, origin_y_,
      origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin
  // and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
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

    // the maximum value to raytrace to is the end of the map
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

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1)) {
      continue;
    }

    unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);
    // update temp_costmap_ with the raytrace line
    MarkCell marker(temp_costmap_, 0);
    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);

    updateRaytraceBounds(
      ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
      clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
      max_y);
  }
}


void
TemporalObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  int j = 0;
  // update costmap_ based on the temporary costmap
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  for (int i = 0; i < size_x_ * size_y_; ++i) {
    unsigned char & cost = temp_costmap_[i];
    if (cost == 0) {
      temp_costmap2_[i] = std::max(static_cast<int>(temp_costmap2_[i]) - decaying_rate_ / 2, static_cast<int>(FREE_SPACE));
    } else if (cost == 1) {
      temp_costmap2_[i] = std::min(static_cast<int>(temp_costmap2_[i]) + rising_rate_ / 2, static_cast<int>(LETHAL_OBSTACLE));
    }
    costmap_[i] = std::min(static_cast<int>(temp_costmap2_[i]) * 2, static_cast<int>(LETHAL_OBSTACLE));
  }
  ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}
}  // namespace cabot_navigation2

// Register this plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::TemporalObstacleLayer, nav2_costmap_2d::Layer)
