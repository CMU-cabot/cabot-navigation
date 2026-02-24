/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Carnegie Mellon University and Miraikan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "cabot_navigation2/sngnn_layer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace cabot_navigation2
{

SNGNNLayer::SNGNNLayer()
: last_map_(nullptr),
  robot_x_(0),
  robot_y_(0),
  robot_yaw_(0),
  need_recalculation_(false)
{
}

SNGNNLayer::~SNGNNLayer()
{
}

void SNGNNLayer::activate()
{
  nav2_costmap_2d::Layer::activate();
}

void SNGNNLayer::deactivate()
{
  nav2_costmap_2d::Layer::deactivate();
}

void SNGNNLayer::reset()
{
  nav2_costmap_2d::Layer::reset();
  std::lock_guard<std::mutex> guard(mutex_);
  current_ = false;
}

bool SNGNNLayer::isClearable()
{
  return false;
}

void SNGNNLayer::onInitialize()
{
  auto node = node_.lock();
  
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("topic", rclcpp::ParameterValue("/sngnn_costmap"));
  node->get_parameter(name_ + "." + "topic", topic_);

  declareParameter("map_resolution", rclcpp::ParameterValue(0.05));
  node->get_parameter(name_ + "." + "map_resolution", map_resolution_);

  declareParameter("update_width", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_width", update_width_);

  declareParameter("update_height", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_height", update_height_);

  declareParameter("max_cost", rclcpp::ParameterValue(252.0));
  node->get_parameter(name_ + "." + "max_cost", max_cost_);

  declareParameter("cost_threshold", rclcpp::ParameterValue(180.0));
  node->get_parameter(name_ + "." + "cost_threshold", cost_threshold_);

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&SNGNNLayer::incomingMap, this, std::placeholders::_1));

  current_ = true;
  
  RCLCPP_INFO(node->get_logger(), "SNGNNLayer initialized. Subscribing to: %s", topic_.c_str());
}

void SNGNNLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  std::lock_guard<std::mutex> guard(mutex_);
  last_map_ = new_map;
}

void SNGNNLayer::onFootprintChanged()
{
  if (!enabled_) {
    return;
  }
  need_recalculation_ = true;
}

void SNGNNLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;
  
  double w = update_width_;
  double h = update_height_;
  
  *min_x = std::min(*min_x, robot_x_ - w/2);
  *min_y = std::min(*min_y, robot_y_ - h/2);
  *max_x = std::max(*max_x, robot_x_ + w/2);
  *max_y = std::max(*max_y, robot_y_ + h/2);
}

void SNGNNLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> guard(mutex_);
  if (!last_map_) return;

  double res = map_resolution_; 
  unsigned int width = last_map_->info.width;
  unsigned int height = last_map_->info.height;
  double ox = last_map_->info.origin.position.x;
  double oy = last_map_->info.origin.position.y;
  
  const auto& data = last_map_->data;
  double c = cos(robot_yaw_);
  double s = sin(robot_yaw_);

  for (unsigned int y = 0; y < height; ++y) {
    double ly = oy + (y + 0.5) * res;

    for (unsigned int x = 0; x < width; ++x) {
      // (height - 1 - y) absorbs vertical flip of the image
      int index = (height - 1 - y) * width + x;
      if (index < 0 || index >= static_cast<int>(data.size())) continue;

      int8_t val = data[index];
      
      if (val < 0) continue; 

      unsigned int map_cost = (unsigned int)(val * 2.52);
      if (map_cost > max_cost_) map_cost = static_cast<unsigned int>(max_cost_);
      
      if (map_cost < cost_threshold_) { 
        continue; 
      }

      double lx = ox + (x + 0.5) * res;
      
      // Transform to world frame (assuming SNGNN map is in robot frame)
      double wx = robot_x_ + lx * c - ly * s;
      double wy = robot_y_ + lx * s + ly * c;
      
      unsigned int mx, my;
      if (master_grid.worldToMap(wx, wy, mx, my)) {
        unsigned char existing = master_grid.getCost(mx, my);
        if (existing == NO_INFORMATION || map_cost > existing) {
           master_grid.setCost(mx, my, static_cast<unsigned char>(map_cost));
        }
      }
    }
  }
}

}  // namespace cabot_navigation2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::SNGNNLayer, nav2_costmap_2d::Layer)
