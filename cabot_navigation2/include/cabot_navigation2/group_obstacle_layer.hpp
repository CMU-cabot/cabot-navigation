// Copyright (c) 2024  Miraikan
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

#ifndef CABOT_NAVIGATION2__GROUP_OBSTACLE_LAYER_HPP_
#define CABOT_NAVIGATION2__GROUP_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "lidar_process_msgs/msg/group_time_array.hpp"
#include "lidar_process_msgs/msg/group_array.hpp"
#include "lidar_process_msgs/msg/group.hpp"

#include "geometry_msgs/msg/point.hpp"

namespace cabot_navigation2
{

class GroupObstacleLayer : public nav2_costmap_2d::ObstacleLayer
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

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double update_width, update_height;
  double init_cost, discount_factor;

  void groupCallBack(const lidar_process_msgs::msg::GroupTimeArray::SharedPtr group_series);

  // Indicates that the entire costmap should be recalculated next time.
  bool need_recalculation_;

  std::string group_topic_;
  rclcpp::Subscription<lidar_process_msgs::msg::GroupTimeArray>::SharedPtr group_sub_;
  lidar_process_msgs::msg::GroupTimeArray::SharedPtr last_group_;

  std::mutex mutex_;
};

}  // namespace cabot_navigation2

#endif  // CABOT_NAVIGATION2__GROUP_OBSTACLE_LAYER_HPP_
