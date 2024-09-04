/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "cabot_navigation2/group_obstacle_layer.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cabot_navigation2
{

GroupObstacleLayer::GroupObstacleLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

GroupObstacleLayer::~GroupObstacleLayer()
{
}

void GroupObstacleLayer::activate()
{
  nav2_costmap_2d::Layer::activate();
}

void GroupObstacleLayer::deactivate()
{
  nav2_costmap_2d::Layer::deactivate();
}

void GroupObstacleLayer::reset()
{
  nav2_costmap_2d::Layer::reset();
}

bool GroupObstacleLayer::isClearable()
{
  return true;
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
GroupObstacleLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("group_topic", rclcpp::ParameterValue("/group_predictions"));
  node->get_parameter(name_ + "." + "group_topic", group_topic_);

  declareParameter("update_width", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_width", update_width);

  declareParameter("update_height", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_height", update_height);

  declareParameter("init_cost", rclcpp::ParameterValue(50.0));
  node->get_parameter(name_ + "." + "init_cost", init_cost);

  declareParameter("discount_factor", rclcpp::ParameterValue(0.9));
  node->get_parameter(name_ + "." + "discount_factor", discount_factor);

  group_sub_ = node->create_subscription<lidar_process_msgs::msg::GroupTimeArray>(
    group_topic_, 10,
    std::bind(&GroupObstacleLayer::groupCallBack, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
GroupObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
  if (need_recalculation_) {
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    *min_x = robot_x - update_width / 2.0;
    *min_y = robot_y - update_height / 2.0;
    *max_x = robot_x + update_width / 2.0;
    *max_y = robot_y + update_height / 2.0;
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
GroupObstacleLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GroupObstacleLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
GroupObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  auto node = node_.lock();
  std::chrono::milliseconds s = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

  if (!enabled_) {
    return;
  }

  if (!last_group_)
  {
    return;
  }
  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  double cost_value = init_cost;
  for (auto it = last_group_->group_sequences.begin(); it != last_group_->group_sequences.end(); it++) {
    auto curr_groups = it;
    for (auto it2 = curr_groups->groups.begin(); it2 != curr_groups->groups.end(); it2++) {
      auto group = it2;
      geometry_msgs::msg::Point left = group->left;
      geometry_msgs::msg::Point center = group->center;
      geometry_msgs::msg::Point right = group->right;
      geometry_msgs::msg::Point right_offset = group->right_offset;
      geometry_msgs::msg::Point left_offset = group->left_offset;

      int lx, ly, cx, cy, rx, ry, rox, roy, lox, loy;
      nav2_costmap_2d::MapLocation l, c, r, ro, lo;
      master_grid.worldToMapEnforceBounds(left.x, left.y, lx, ly);
      master_grid.worldToMapEnforceBounds(center.x, center.y, cx, cy);
      master_grid.worldToMapEnforceBounds(right.x, right.y, rx, ry);
      master_grid.worldToMapEnforceBounds(right_offset.x, right_offset.y, rox, roy);
      master_grid.worldToMapEnforceBounds(left_offset.x, left_offset.y, lox, loy);

      RCLCPP_INFO(node->get_logger(), "Group point: x %.2f, y %.2f", left.x, left.y);
      RCLCPP_INFO(node->get_logger(), "Group map point: x %d, y %d", lx, ly);

      l.x = (unsigned int)lx;
      l.y = (unsigned int)ly;
      c.x = (unsigned int)cx;
      c.y = (unsigned int)cy;
      r.x = (unsigned int)rx;
      r.y = (unsigned int)ry;
      ro.x = (unsigned int)rox;
      ro.y = (unsigned int)roy;
      lo.x = (unsigned int)lox;
      lo.y = (unsigned int)loy;

      std::vector<nav2_costmap_2d::MapLocation> map_polygon;
      map_polygon.push_back(l);
      map_polygon.push_back(c);
      map_polygon.push_back(r);
      map_polygon.push_back(ro);
      map_polygon.push_back(lo);

      std::vector<nav2_costmap_2d::MapLocation> polygon_cells;
      master_grid.convexFillCells(map_polygon, polygon_cells);

      for (unsigned int i = 0; i < polygon_cells.size(); i++) {
        unsigned int index = master_grid.getIndex(polygon_cells[i].x, polygon_cells[i].y);
        if (master_array[index] < cost_value && cost_value < LETHAL_OBSTACLE) {
          master_array[index] = cost_value;
        }
      }
    }
    cost_value = cost_value * discount_factor;
  }

  //updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  std::chrono::milliseconds e = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  RCLCPP_INFO(node->get_logger(), "GrouObstacleLayer update time cost: %d ms", e - s);
}

void GroupObstacleLayer::groupCallBack(const lidar_process_msgs::msg::GroupTimeArray::SharedPtr group)
{
  // Group sequences: First time, then groups
  auto node = node_.lock();
  last_group_ = group;
}

}  // namespace cabot_navigation2

// This is the macro allowing a cabot_navigation2::GroupObstacleLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::GroupObstacleLayer, nav2_costmap_2d::Layer)