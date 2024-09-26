// Copyright (c) 2024  Carnegie Mellon University, IBM Corporation, and others
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

#ifndef CABOT_NAVIGATION2__GRID_MAP_GROUND_FILTER_NODE_HPP_
#define CABOT_NAVIGATION2__GRID_MAP_GROUND_FILTER_NODE_HPP_

#include <memory>
#include <string>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "cabot_navigation2/abstract_ground_filter_node.hpp"

namespace cabot_navigation2
{

class GridMapGroundFilterNode : public AbstractGroundFilterNode
{
public:
  explicit GridMapGroundFilterNode(const rclcpp::NodeOptions & options);

protected:
  void filterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) override;

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr input);
  int calcLivoxGridEstimatedNumPoints(float distance, float resolution);

  std::string odom_topic_;
  float grid_map_resolution_;
  float grid_map_length_;
  float grid_num_points_raio_threshold_;
  float grid_var_threshold_;
  float grid_prob_prior_;
  float grid_prob_free_;
  float grid_prob_occupied_;
  float grid_occupancy_threshold_;
  float ground_update_rate_;
  float ground_distance_threshold_;

  float log_odds_prior_;
  float log_odds_free_;
  float log_odds_occupied_;
  float grid_map_spiral_iterate_radius_;

  std::recursive_mutex grid_map_mutex_;
  std::shared_ptr<grid_map::GridMap> grid_map_ptr_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr debug_grid_map_pub_;
};  // class GridMapGroundFilterNode

}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__GRID_MAP_GROUND_FILTER_NODE_HPP_
