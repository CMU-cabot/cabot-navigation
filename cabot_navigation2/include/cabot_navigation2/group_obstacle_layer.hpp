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
#ifndef CABOT_NAVIGATION2__GROUP_OBSTACLE_LAYER_HPP_
#define CABOT_NAVIGATION2__GROUP_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "lidar_process_msgs/msg/group_array1_d.hpp"
#include "lidar_process_msgs/msg/group_time_array.hpp"
#include "lidar_process_msgs/msg/group_array.hpp"
#include "lidar_process_msgs/msg/group.hpp"

#include "geometry_msgs/msg/point.hpp"

namespace cabot_navigation2
{

class GroupObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  GroupObstacleLayer();
  ~GroupObstacleLayer() override;

  void activate() override;
  void deactivate() override;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  void onFootprintChanged() override;

  bool isClearable() override;

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double update_width, update_height;
  double init_cost, discount_factor;

  void groupCallBack(const lidar_process_msgs::msg::GroupArray1D::SharedPtr group_series);

  // Indicates that the entire costmap should be recalculated next time.
  bool need_recalculation_;

  std::string group_topic_;
  rclcpp::Subscription<lidar_process_msgs::msg::GroupArray1D>::SharedPtr group_sub_;
  int group_quantity_;
  int group_points_ = 10;
  std::vector<float> groups_;
  lidar_process_msgs::msg::GroupTimeArray::SharedPtr last_group_;

  std::mutex mutex_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
};

}  // namespace cabot_navigation2

#endif  // CABOT_NAVIGATION2__GROUP_OBSTACLE_LAYER_HPP_