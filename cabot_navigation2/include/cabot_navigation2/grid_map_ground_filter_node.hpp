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
#include <vector>

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
  void filterGround(const rclcpp::Time & time, const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) override;

private:
  void moveGridMap(const grid_map::Position & gmap_origin_position);
  int calcLivoxGridEstimatedNumPoints(float distance, float resolution);
  bool isVisibleAngle(const grid_map::Position & check_position, const grid_map::Position & sensor_position, double sensor_yaw);
  void inflateBinaryMat(const cv::Mat & binary_mat, cv::Mat & inflate_binary_mat, int inflate_size);

  int num_threads_;
  std::string odom_topic_;
  float grid_resolution_;
  float grid_length_;
  std::vector<int64_t> grid_patch_sizes_;
  std::vector<double> grid_patch_change_distances_;
  int grid_occupied_inflate_size_;
  int grid_num_points_min_threshold_;
  float grid_num_points_raio_threshold_;
  float grid_var_threshold_;
  float grid_prob_prior_;
  float grid_prob_free_;
  float grid_prob_occupied_;
  float grid_prob_forget_rate_;
  float grid_prob_free_threshold_;
  float grid_prob_occupied_threshold_;
  float outlier_old_ground_threshold_;
  float outlier_los_ground_threshold_;
  float ground_estimate_angle_min_;
  float ground_estimate_angle_max_;
  float ground_slope_threshold_;
  float ground_confidence_interpolate_decay_;

  static const int livox_num_points_;
  static const float livox_tan_fov_angle_;

  std::vector<int64_t> grid_half_patch_sizes_;
  float log_odds_prior_;
  float log_odds_free_;
  float log_odds_occupied_;
  float ground_estimate_radius_;

  std::shared_ptr<grid_map::GridMap> grid_map_ptr_;
  cv::Mat grid_map_is_observed_occupied_;
  cv::Mat grid_map_is_occupied_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_outlier_pointcloud_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr debug_grid_map_pub_;
};  // class GridMapGroundFilterNode

}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__GRID_MAP_GROUND_FILTER_NODE_HPP_
