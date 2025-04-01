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

#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#include <pcl_ros/transforms.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "cabot_navigation2/grid_map_ground_filter_node.hpp"

namespace cabot_navigation2
{

// https://www.livoxtech.com/mid-70/specs
const int GridMapGroundFilterNode::livox_num_points_ = 100000;
const float GridMapGroundFilterNode::livox_tan_fov_angle_ = std::tan(M_PI * (70.4 / 2.0) / 180.0);

GridMapGroundFilterNode::GridMapGroundFilterNode(const rclcpp::NodeOptions & options)
: AbstractGroundFilterNode("grid_map_ground_filter_node", options),
  num_threads_(2),
  odom_topic_("/odom"),
  grid_resolution_(0.05),
  grid_length_(10.0),
  grid_patch_sizes_({3, 5}),
  grid_patch_change_distances_({1.0}),
  grid_occupied_inflate_size_(3),
  grid_num_points_min_threshold_(5),
  grid_num_points_raio_threshold_(0.1),
  grid_var_threshold_(0.0005),
  grid_prob_prior_(0.5),
  grid_prob_free_(0.1),
  grid_prob_occupied_(0.9),
  grid_prob_forget_rate_(0.2),
  grid_prob_free_threshold_(0.15),
  grid_prob_occupied_threshold_(0.55),
  outlier_old_ground_threshold_(0.05),
  outlier_los_ground_threshold_(0.05),
  ground_estimate_angle_min_(-0.614),
  ground_estimate_angle_max_(0.614),
  ground_slope_threshold_(0.262),
  ground_confidence_interpolate_decay_(0.5)
{
  num_threads_ = declare_parameter("num_threads", num_threads_);
  odom_topic_ = declare_parameter("odom_topic", odom_topic_);
  grid_resolution_ = declare_parameter("grid_resolution", grid_resolution_);
  grid_length_ = declare_parameter("grid_length", grid_length_);
  grid_patch_sizes_ = declare_parameter("grid_patch_sizes", grid_patch_sizes_);
  grid_patch_change_distances_ = declare_parameter("grid_patch_change_distances", grid_patch_change_distances_);
  if (grid_patch_sizes_.size() - 1 != grid_patch_change_distances_.size()) {
    RCLCPP_ERROR(get_logger(), "Invalid grid patch parameters, set length of patch sizes should be one value larger than length of patch change distances.");
    rclcpp::shutdown();
    std::exit(EXIT_FAILURE);
  }
  grid_occupied_inflate_size_ = declare_parameter("grid_occupied_inflate_size", grid_occupied_inflate_size_);
  grid_num_points_min_threshold_ = declare_parameter("grid_num_points_min_threshold", grid_num_points_min_threshold_);
  grid_num_points_raio_threshold_ = declare_parameter("grid_num_points_raio_threshold", grid_num_points_raio_threshold_);
  grid_var_threshold_ = declare_parameter("grid_var_threshold", grid_var_threshold_);
  grid_prob_prior_ = declare_parameter("grid_prob_prior", grid_prob_prior_);
  grid_prob_free_ = declare_parameter("grid_prob_free", grid_prob_free_);
  grid_prob_occupied_ = declare_parameter("grid_prob_occupied", grid_prob_occupied_);
  grid_prob_forget_rate_ = declare_parameter("grid_prob_forget_rate", grid_prob_forget_rate_);
  grid_prob_free_threshold_ = declare_parameter("grid_prob_free_threshold", grid_prob_free_threshold_);
  grid_prob_occupied_threshold_ = declare_parameter("grid_prob_occupied_threshold", grid_prob_occupied_threshold_);
  outlier_old_ground_threshold_ = declare_parameter("outlier_old_ground_threshold", outlier_old_ground_threshold_);
  outlier_los_ground_threshold_ = declare_parameter("outlier_los_ground_threshold", outlier_los_ground_threshold_);
  ground_estimate_angle_min_ = declare_parameter("ground_estimate_angle_min", ground_estimate_angle_min_);
  ground_estimate_angle_max_ = declare_parameter("ground_estimate_angle_max", ground_estimate_angle_max_);
  ground_slope_threshold_ = declare_parameter("ground_slope_threshold", ground_slope_threshold_);
  ground_confidence_interpolate_decay_ = declare_parameter("ground_confidence_interpolate_decay", ground_confidence_interpolate_decay_);

  for (auto patch_size : grid_patch_sizes_) {
    grid_half_patch_sizes_.push_back(std::floor(patch_size / 2));
  }
  log_odds_prior_ = std::log(grid_prob_prior_ / (1.0 - grid_prob_prior_));
  log_odds_free_ = std::log(grid_prob_free_ / (1.0 - grid_prob_free_));
  log_odds_occupied_ = std::log(grid_prob_occupied_ / (1.0 - grid_prob_occupied_));
  ground_estimate_radius_ = grid_length_ / 2.0;

  if (publish_debug_ground_) {
    debug_outlier_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_debug_ground_topic_ + "/outlier_pointcloud", rclcpp::QoS(1).transient_local());
    debug_grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(output_debug_ground_topic_, rclcpp::QoS(1).transient_local());
  }
}

void GridMapGroundFilterNode::moveGridMap(const grid_map::Position & gmap_origin_position)
{
  if (!grid_map_ptr_) {
    grid_map_ptr_ = std::make_shared<grid_map::GridMap, const std::vector<std::string>>(
      {"num_points", "mean_points_z", "var_points_z", "is_observed_enough_points", "is_observed_ground", "log_odds", "occupancy", "is_free", "observed_ground_z",
        "valid_ground_z", "estimated_ground_z", "ground_confidence"});
    grid_map_ptr_->setFrameId("odom");
    grid_map_ptr_->setGeometry(grid_map::Length(grid_length_, grid_length_), grid_resolution_, gmap_origin_position);
    grid_map_ptr_->move(gmap_origin_position);
    (*grid_map_ptr_)["num_points"].setZero();
    (*grid_map_ptr_)["mean_points_z"].setZero();
    (*grid_map_ptr_)["var_points_z"].setZero();
    (*grid_map_ptr_)["is_observed_enough_points"].setZero();
    (*grid_map_ptr_)["is_observed_ground"].setZero();
    (*grid_map_ptr_)["log_odds"].setConstant(log_odds_prior_);
    (*grid_map_ptr_)["occupancy"].setConstant(grid_prob_prior_);
    (*grid_map_ptr_)["is_free"].setZero();
    (*grid_map_ptr_)["observed_ground_z"].setConstant(std::numeric_limits<float>::max());
    (*grid_map_ptr_)["valid_ground_z"].setConstant(std::numeric_limits<float>::max());
    (*grid_map_ptr_)["estimated_ground_z"].setConstant(std::numeric_limits<float>::max());
    (*grid_map_ptr_)["ground_confidence"].setZero();

    const grid_map::Size gmap_size = grid_map_ptr_->getSize();
    grid_map_is_observed_occupied_ = cv::Mat::zeros(gmap_size(0), gmap_size(1), CV_8UC1);
    grid_map_is_occupied_ = cv::Mat::zeros(gmap_size(0), gmap_size(1), CV_8UC1);
    return;
  }

  std::vector<grid_map::BufferRegion> new_regions;
  grid_map_ptr_->move(gmap_origin_position, new_regions);

  // store references for efficient access
  grid_map::Matrix & num_points_layer = (*grid_map_ptr_)["num_points"];
  grid_map::Matrix & mean_points_z_layer = (*grid_map_ptr_)["mean_points_z"];
  grid_map::Matrix & var_points_z_layer = (*grid_map_ptr_)["var_points_z"];
  grid_map::Matrix & is_observed_enough_points_layer = (*grid_map_ptr_)["is_observed_enough_points"];
  grid_map::Matrix & is_observed_ground_layer = (*grid_map_ptr_)["is_observed_ground"];
  grid_map::Matrix & log_odds_layer = (*grid_map_ptr_)["log_odds"];
  grid_map::Matrix & occupancy_layer = (*grid_map_ptr_)["occupancy"];
  grid_map::Matrix & is_free_layer = (*grid_map_ptr_)["is_free"];
  grid_map::Matrix & observed_ground_z_layer = (*grid_map_ptr_)["observed_ground_z"];
  grid_map::Matrix & valid_ground_z_layer = (*grid_map_ptr_)["valid_ground_z"];
  grid_map::Matrix & estimated_ground_z_layer = (*grid_map_ptr_)["estimated_ground_z"];
  grid_map::Matrix & ground_confidence_layer = (*grid_map_ptr_)["ground_confidence"];

  for (auto region : new_regions) {
    for (auto iterator = grid_map::SubmapIterator(*grid_map_ptr_, region); !iterator.isPastEnd(); ++iterator) {
      grid_map::Index gmap_index = *iterator;

      num_points_layer(gmap_index(0), gmap_index(1)) = 0.0;
      mean_points_z_layer(gmap_index(0), gmap_index(1)) = 0.0;
      var_points_z_layer(gmap_index(0), gmap_index(1)) = 0.0;
      is_observed_enough_points_layer(gmap_index(0), gmap_index(1)) = 0.0;
      is_observed_ground_layer(gmap_index(0), gmap_index(1)) = 0.0;
      log_odds_layer(gmap_index(0), gmap_index(1)) = log_odds_prior_;
      occupancy_layer(gmap_index(0), gmap_index(1)) = grid_prob_prior_;
      is_free_layer(gmap_index(0), gmap_index(1)) = 0.0;
      observed_ground_z_layer(gmap_index(0), gmap_index(1)) = std::numeric_limits<float>::max();
      valid_ground_z_layer(gmap_index(0), gmap_index(1)) = std::numeric_limits<float>::max();
      estimated_ground_z_layer(gmap_index(0), gmap_index(1)) = std::numeric_limits<float>::max();
      ground_confidence_layer(gmap_index(0), gmap_index(1)) = 0.0;
    }
  }

  grid_map_ptr_->convertToDefaultStartIndex();
}

int GridMapGroundFilterNode::calcLivoxGridEstimatedNumPoints(float distance, float resolution)
{
  const float fov_radius = distance * livox_tan_fov_angle_;
  const float fov_area = M_PI * std::pow(fov_radius, 2.0);
  return livox_num_points_ * std::pow(resolution, 2.0) / fov_area;
}

bool GridMapGroundFilterNode::isVisibleAngle(const grid_map::Position & check_position, const grid_map::Position & sensor_position, double sensor_yaw)
{
  float angle = std::atan2(check_position.y() - sensor_position.y(), check_position.x() - sensor_position.x()) - sensor_yaw;
  if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  } else if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  if ((angle >= ground_estimate_angle_min_) && (angle <= ground_estimate_angle_max_)) {
    return true;
  } else {
    return false;
  }
}

void GridMapGroundFilterNode::inflateBinaryMat(const cv::Mat & binary_mat, cv::Mat & inflate_binary_mat, int inflate_size)
{
  cv::dilate(binary_mat, inflate_binary_mat, cv::Mat::ones(inflate_size, inflate_size, CV_8UC1), cv::Point(-1, -1), 1);
}

void GridMapGroundFilterNode::filterGround(
  const rclcpp::Time & time, const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground,
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered)
{
  // transform point cloud to map frame
  geometry_msgs::msg::PoseStamped gmap_origin_pose;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_input(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      "odom", target_frame_,
      rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));

    pcl_ros::transformPointCloud(*input, *transformed_input, tf_stamped);

    geometry_msgs::msg::PoseStamped origin_pose;
    origin_pose.header.frame_id = target_frame_;
    origin_pose.header.stamp = time;
    origin_pose.pose.position.x = 0.0;
    origin_pose.pose.position.y = 0.0;
    origin_pose.pose.position.z = 0.0;
    origin_pose.pose.orientation.x = 0.0;
    origin_pose.pose.orientation.y = 0.0;
    origin_pose.pose.orientation.z = 0.0;
    origin_pose.pose.orientation.w = 1.0;
    tf2::doTransform(origin_pose, gmap_origin_pose, tf_stamped);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  // update grid map position
  grid_map::Position gmap_origin_position(gmap_origin_pose.pose.position.x, gmap_origin_pose.pose.position.y);
  moveGridMap(gmap_origin_position);

  // store references for efficient access
  grid_map::Matrix & num_points_layer = (*grid_map_ptr_)["num_points"];
  grid_map::Matrix & mean_points_z_layer = (*grid_map_ptr_)["mean_points_z"];
  grid_map::Matrix & var_points_z_layer = (*grid_map_ptr_)["var_points_z"];
  grid_map::Matrix & is_observed_enough_points_layer = (*grid_map_ptr_)["is_observed_enough_points"];
  grid_map::Matrix & is_observed_ground_layer = (*grid_map_ptr_)["is_observed_ground"];
  grid_map::Matrix & log_odds_layer = (*grid_map_ptr_)["log_odds"];
  grid_map::Matrix & occupancy_layer = (*grid_map_ptr_)["occupancy"];
  grid_map::Matrix & is_free_layer = (*grid_map_ptr_)["is_free"];
  grid_map::Matrix & observed_ground_z_layer = (*grid_map_ptr_)["observed_ground_z"];
  grid_map::Matrix & valid_ground_z_layer = (*grid_map_ptr_)["valid_ground_z"];
  grid_map::Matrix & estimated_ground_z_layer = (*grid_map_ptr_)["estimated_ground_z"];
  grid_map::Matrix & ground_confidence_layer = (*grid_map_ptr_)["ground_confidence"];

  // collect visible area grid map indices in spiral order
  std::vector<grid_map::Index> visible_grid_spiral_indices;
  std::vector<grid_map::Index> invisible_grid_spiral_indices;
  grid_map::Index gmap_origin_pose_index;
  grid_map_ptr_->getIndex(gmap_origin_position, gmap_origin_pose_index);
  const double gmap_origin_pose_yaw = tf2::getYaw(gmap_origin_pose.pose.orientation);
  for (auto iterator = grid_map::SpiralIterator(*grid_map_ptr_, gmap_origin_position, ground_estimate_radius_); !iterator.isPastEnd(); ++iterator) {
    grid_map::Index gmap_index = *iterator;

    if ((gmap_index(0) == gmap_origin_pose_index(0)) && (gmap_index(1) == gmap_origin_pose_index(1))) {
      // collect sensor position as invisible area
      invisible_grid_spiral_indices.push_back(gmap_index);
    } else {
      grid_map::Position gmap_position;
      grid_map_ptr_->getPosition(gmap_index, gmap_position);

      if (grid_map_ptr_->isInside(gmap_position)) {
        if (isVisibleAngle(gmap_position, gmap_origin_position, gmap_origin_pose_yaw)) {
          visible_grid_spiral_indices.push_back(gmap_index);
        } else {
          invisible_grid_spiral_indices.push_back(gmap_index);
        }
      }
    }
  }

  // collect patch sizes for visible areas
  std::vector<int> visible_grid_spiral_patch_sizes;
  std::vector<int> visible_grid_spiral_half_patch_sizes;
  visible_grid_spiral_patch_sizes.reserve(visible_grid_spiral_indices.size());
  visible_grid_spiral_half_patch_sizes.reserve(visible_grid_spiral_indices.size());
  const float gmap_resolution = grid_map_ptr_->getResolution();
  for (unsigned int i = 0, j = 0; i < visible_grid_spiral_indices.size(); i++) {
    grid_map::Index gmap_index = visible_grid_spiral_indices[i];
    float gmap_distance = std::hypot(gmap_index(0) - gmap_origin_pose_index(0), gmap_index(1) - gmap_origin_pose_index(1)) * gmap_resolution;
    if ((gmap_distance > grid_patch_change_distances_[j]) && (j < grid_patch_sizes_.size() - 1)) {
      j += 1;
    }
    visible_grid_spiral_patch_sizes.push_back(grid_patch_sizes_[j]);
    visible_grid_spiral_half_patch_sizes.push_back(grid_half_patch_sizes_[j]);
  }

  // add points to grid map
  const grid_map::Size gmap_size = grid_map_ptr_->getSize();
  std::vector<std::vector<std::vector<double>>> grid_map_pointcloud_z(gmap_size(0), std::vector<std::vector<double>>(gmap_size(1)));
  pcl::PointIndices outlier_indices;
  for (unsigned int i = 0; i < transformed_input->points.size(); i++) {
    const auto & p = transformed_input->points[i];
    const auto & p_pos = grid_map::Position(p.x, p.y);
    if (!grid_map_ptr_->isInside(p_pos)) {
      continue;
    }

    grid_map::Index p_index;
    grid_map_ptr_->getIndex(p_pos, p_index);

    // check outlier by previous ground height in line of sight
    bool is_outlier = false;
    if (p.z < estimated_ground_z_layer(p_index(0), p_index(1)) - outlier_old_ground_threshold_) {
      // point is enough below previous estimated ground height
      float z_slope = (p.z - gmap_origin_pose.pose.position.z) / std::hypot(p.x - gmap_origin_pose.pose.position.x, p.y - gmap_origin_pose.pose.position.y);

      auto los_iterator = grid_map::LineIterator(*grid_map_ptr_, gmap_origin_position, p_pos);
      ++los_iterator;
      for (; !los_iterator.isPastEnd(); ++los_iterator) {
        grid_map::Index los_gmap_index = *los_iterator;

        if (valid_ground_z_layer(los_gmap_index(0), los_gmap_index(1)) != std::numeric_limits<float>::max()) {
          float los_dist = std::hypot(los_gmap_index(0) - gmap_origin_pose_index(0), los_gmap_index(1) - gmap_origin_pose_index(1)) * gmap_resolution;
          float los_z = gmap_origin_pose.pose.position.z + z_slope * los_dist;
          if (los_z < valid_ground_z_layer(los_gmap_index(0), los_gmap_index(1)) - outlier_los_ground_threshold_) {
            // line of sight is enough below previous validated ground height
            is_outlier = true;
            break;
          }
        }
      }
    }

    if (!is_outlier) {
      grid_map_pointcloud_z[p_index(0)][p_index(1)].push_back(p.z);
    } else {
      outlier_indices.indices.push_back(i);
    }
  }

  // initialize grid map except log odds
  num_points_layer.setZero();
  mean_points_z_layer.setZero();
  var_points_z_layer.setZero();
  is_observed_enough_points_layer.setZero();
  is_observed_ground_layer.setZero();
  occupancy_layer.setConstant(grid_prob_prior_);
  is_free_layer.setZero();
  observed_ground_z_layer.setConstant(std::numeric_limits<float>::max());
  valid_ground_z_layer.setConstant(std::numeric_limits<float>::max());
  estimated_ground_z_layer.setConstant(std::numeric_limits<float>::max());
  ground_confidence_layer.setZero();

  grid_map_is_observed_occupied_.setTo(0);
  grid_map_is_occupied_.setTo(0);

  // calculate mean, variance in each grid cell
#pragma omp parallel for num_threads(num_threads_) schedule(dynamic, 1)
  for (auto iterator = visible_grid_spiral_indices.begin(); iterator != visible_grid_spiral_indices.end(); ++iterator) {
    grid_map::Index gmap_index = *iterator;

    int num_points = 0;
    float mean_points_z = 0.0;
    float m2_points_z = 0.0;
    for (const auto & p_z : grid_map_pointcloud_z[gmap_index(0)][gmap_index(1)]) {
      num_points += 1;
      float delta = p_z - mean_points_z;
      mean_points_z += delta / num_points;
      m2_points_z += delta * (p_z - mean_points_z);
    }
    num_points_layer(gmap_index(0), gmap_index(1)) = num_points;
    mean_points_z_layer(gmap_index(0), gmap_index(1)) = mean_points_z;
    if (num_points > 1) {
      var_points_z_layer(gmap_index(0), gmap_index(1)) = m2_points_z / (num_points - 1);
    }
  }

  // update occupancy in visible area
#pragma omp parallel for num_threads(num_threads_) schedule(dynamic, 1)
  for (unsigned int i = 0; i < visible_grid_spiral_indices.size(); i++) {
    grid_map::Index gmap_index = visible_grid_spiral_indices[i];

    const float grid_distance = std::hypot(gmap_index(0) - (gmap_size(0) / 2.0), gmap_index(1) - (gmap_size(1) / 2.0)) * gmap_resolution;

    const int grid_estimated_num_points = calcLivoxGridEstimatedNumPoints(grid_distance, gmap_resolution);
    const int grid_num_points_threshold = std::max(static_cast<int>(grid_num_points_raio_threshold_ * grid_estimated_num_points), grid_num_points_min_threshold_);

    bool is_enough_points = false;
    float var_points_z = 0.0;
    int num_points_grid = num_points_layer(gmap_index(0), gmap_index(1));
    if (num_points_grid > grid_num_points_threshold) {
      // observe enough points in the grid, use points only in the grid
      is_enough_points = true;
      var_points_z = var_points_z_layer(gmap_index(0), gmap_index(1));
    } else {
      // do not observe enough points in the grid, use points in patch
      const int patch_size = visible_grid_spiral_patch_sizes[i];
      const int half_patch_size = visible_grid_spiral_half_patch_sizes[i];
      const auto & num_points_patch = num_points_layer.block(gmap_index(0) - half_patch_size, gmap_index(1) - half_patch_size, patch_size, patch_size);
      const auto & var_points_z_patch = var_points_z_layer.block(gmap_index(0) - half_patch_size, gmap_index(1) - half_patch_size, patch_size, patch_size);
      const float & num_points_patch_sum = num_points_patch.sum();
      if (num_points_patch_sum > grid_num_points_threshold) {
        is_enough_points = true;
        var_points_z = var_points_z_patch.cwiseProduct(num_points_patch).sum() / num_points_patch_sum;
      }
    }

    if (is_enough_points) {
      is_observed_enough_points_layer(gmap_index(0), gmap_index(1)) = 1.0;
      if (var_points_z > grid_var_threshold_) {
        grid_map_is_observed_occupied_.at<unsigned char>(gmap_index(0), gmap_index(1)) = 1;
        // update log odds as occupied area
        log_odds_layer(gmap_index(0), gmap_index(1)) = log_odds_layer(gmap_index(0), gmap_index(1)) - log_odds_prior_ + log_odds_occupied_;
      } else {
        is_observed_ground_layer(gmap_index(0), gmap_index(1)) = 1.0;
        // update log odds as free area
        log_odds_layer(gmap_index(0), gmap_index(1)) = log_odds_layer(gmap_index(0), gmap_index(1)) - log_odds_prior_ + log_odds_free_;
      }
    } else {
      // do not observe enough point, forget previous updates of log odds
      log_odds_layer(gmap_index(0), gmap_index(1)) = log_odds_prior_ + (log_odds_layer(gmap_index(0), gmap_index(1)) - log_odds_prior_) * (1.0 - grid_prob_forget_rate_);
    }

    occupancy_layer(gmap_index(0), gmap_index(1)) = 1.0 - 1.0 / (1.0 + std::exp(log_odds_layer(gmap_index(0), gmap_index(1))));
    is_free_layer(gmap_index(0), gmap_index(1)) = (occupancy_layer(gmap_index(0), gmap_index(1)) < grid_prob_free_threshold_) ? 1.0 : 0.0;
    grid_map_is_occupied_.at<unsigned char>(gmap_index(0), gmap_index(1)) = (occupancy_layer(gmap_index(0), gmap_index(1)) > grid_prob_occupied_threshold_) ? 1 : 0;
  }

  // update occupancy in invisible area
#pragma omp parallel for num_threads(num_threads_) schedule(dynamic, 1)
  for (auto iterator = invisible_grid_spiral_indices.begin(); iterator != invisible_grid_spiral_indices.end(); ++iterator) {
    grid_map::Index gmap_index = *iterator;

    // invisible area, forget previous updates of log odds
    log_odds_layer(gmap_index(0), gmap_index(1)) = log_odds_prior_ + (log_odds_layer(gmap_index(0), gmap_index(1)) - log_odds_prior_) * (1.0 - grid_prob_forget_rate_);

    occupancy_layer(gmap_index(0), gmap_index(1)) = 1.0 - 1.0 / (1.0 + std::exp(log_odds_layer(gmap_index(0), gmap_index(1))));
    is_free_layer(gmap_index(0), gmap_index(1)) = (occupancy_layer(gmap_index(0), gmap_index(1)) < grid_prob_free_threshold_) ? 1.0 : 0.0;
    grid_map_is_occupied_.at<unsigned char>(gmap_index(0), gmap_index(1)) = (occupancy_layer(gmap_index(0), gmap_index(1)) > grid_prob_occupied_threshold_) ? 1 : 0;
  }

  // inflate binary observed occupied area, binary occupied area
  inflateBinaryMat(grid_map_is_observed_occupied_, grid_map_is_observed_occupied_, grid_occupied_inflate_size_);
  inflateBinaryMat(grid_map_is_occupied_, grid_map_is_occupied_, grid_occupied_inflate_size_);

  // set ground height for robot position
  observed_ground_z_layer(gmap_origin_pose_index(0), gmap_origin_pose_index(1)) = gmap_origin_pose.pose.position.z;
  valid_ground_z_layer(gmap_origin_pose_index(0), gmap_origin_pose_index(1)) = gmap_origin_pose.pose.position.z;
  estimated_ground_z_layer(gmap_origin_pose_index(0), gmap_origin_pose_index(1)) = gmap_origin_pose.pose.position.z;
  ground_confidence_layer(gmap_origin_pose_index(0), gmap_origin_pose_index(1)) = 1.0 - occupancy_layer(gmap_origin_pose_index(0), gmap_origin_pose_index(1));

  // calculate ground height for areas where ground is observed
#pragma omp parallel for num_threads(num_threads_) schedule(dynamic, 1)
  for (unsigned int i = 0; i < visible_grid_spiral_indices.size(); i++) {
    grid_map::Index gmap_index = visible_grid_spiral_indices[i];

    int num_observed_ground_z = 0;
    float sum_observed_ground_z = 0.0;
    if (is_observed_ground_layer(gmap_index(0), gmap_index(1)) && is_free_layer(gmap_index(0), gmap_index(1)) &&
      !grid_map_is_occupied_.at<unsigned char>(gmap_index(0), gmap_index(1)))
    {
      // calculate ground height by averaging minimum point height in the patch
      const int half_patch_size = visible_grid_spiral_half_patch_sizes[i];
      const int row_start = std::max(gmap_index(0) - half_patch_size, 0);
      const int row_end = std::min(gmap_index(0) + half_patch_size, gmap_size(0) - 1);
      const int col_start = std::max(gmap_index(1) - half_patch_size, 0);
      const int col_end = std::min(gmap_index(1) + half_patch_size, gmap_size(1) - 1);

      for (int row = row_start; row <= row_end; row++) {
        for (int col = col_start; col <= col_end; col++) {
          if (is_observed_ground_layer(row, col) && is_free_layer(row, col) && !grid_map_is_occupied_.at<unsigned char>(row, col)) {
            num_observed_ground_z += num_points_layer(row, col);
            sum_observed_ground_z += mean_points_z_layer(row, col) * num_points_layer(row, col);
          }
        }
      }
    }

    if (num_observed_ground_z > 0) {
      observed_ground_z_layer(gmap_index(0), gmap_index(1)) = sum_observed_ground_z / num_observed_ground_z;
    }
  }

  // validate observed ground height from center of grid map in spiral order
  for (auto iterator = visible_grid_spiral_indices.begin(); iterator != visible_grid_spiral_indices.end(); ++iterator) {
    grid_map::Index gmap_index = *iterator;

    float observed_ground_z = observed_ground_z_layer(gmap_index(0), gmap_index(1));
    if (observed_ground_z != std::numeric_limits<float>::max()) {
      // validate observed ground area by checking if it is not behind observed occupied area and slope to the closest validated ground in line of sight
      bool is_valid = false;
      grid_map::Position gmap_position;
      grid_map_ptr_->getPosition(gmap_index, gmap_position);

      auto los_iterator = grid_map::LineIterator(*grid_map_ptr_, gmap_position, gmap_origin_position);
      ++los_iterator;
      for (; !los_iterator.isPastEnd(); ++los_iterator) {
        grid_map::Index los_gmap_index = *los_iterator;

        if (grid_map_is_observed_occupied_.at<unsigned char>(los_gmap_index(0), los_gmap_index(1))) {
          // observed occupied area is found in line of sight, set as invalid ground
          break;
        }

        float los_valid_ground_z = valid_ground_z_layer(los_gmap_index(0), los_gmap_index(1));
        if (los_valid_ground_z != std::numeric_limits<float>::max()) {
          grid_map::Position los_gmap_position;
          grid_map_ptr_->getPosition(los_gmap_index, los_gmap_position);
          double dist = std::hypot(gmap_position.x() - los_gmap_position.x(), gmap_position.y() - los_gmap_position.y());
          double slope = std::atan2(observed_ground_z - los_valid_ground_z, dist);
          if (std::abs(slope) < ground_slope_threshold_) {
            // slope to the closest validated ground in line of sight is small, set as valid ground
            is_valid = true;
          }
          break;
        }
      }
      if (is_valid) {
        valid_ground_z_layer(gmap_index(0), gmap_index(1)) = observed_ground_z;
        estimated_ground_z_layer(gmap_index(0), gmap_index(1)) = observed_ground_z;
        ground_confidence_layer(gmap_index(0), gmap_index(1)) = 1.0 - occupancy_layer(gmap_index(0), gmap_index(1));
      }
    }
  }

  // estimate ground height by interpolating validated ground height from center of grid map in spiral order
  for (unsigned int i = 0; i < visible_grid_spiral_indices.size(); i++) {
    grid_map::Index gmap_index = visible_grid_spiral_indices[i];

    if (estimated_ground_z_layer(gmap_index(0), gmap_index(1)) == std::numeric_limits<float>::max()) {
      // estimate ground height by surrounding free area
      const int patch_size = visible_grid_spiral_patch_sizes[i];
      const int half_patch_size = visible_grid_spiral_half_patch_sizes[i];
      const auto & estimated_ground_z_patch = estimated_ground_z_layer.block(gmap_index(0) - half_patch_size, gmap_index(1) - half_patch_size, patch_size, patch_size);
      const auto & ground_confidence_patch = ground_confidence_layer.block(gmap_index(0) - half_patch_size, gmap_index(1) - half_patch_size, patch_size, patch_size);

      // calculate weighted average of estimated ground height
      float sum_estimated_ground_z = 0.0;
      float sum_ground_confidence = 0.0;
      int num_averaged_grid = 0;
      for (int row = 0; row < estimated_ground_z_patch.rows(); row++) {
        for (int col = 0; col < estimated_ground_z_patch.cols(); col++) {
          if ((estimated_ground_z_patch(row, col) != std::numeric_limits<float>::max()) && (ground_confidence_patch(row, col) > 0.0)) {
            sum_estimated_ground_z += estimated_ground_z_patch(row, col) * ground_confidence_patch(row, col);
            sum_ground_confidence += ground_confidence_patch(row, col);
            num_averaged_grid += 1;
          }
        }
      }

      if (sum_ground_confidence > 0) {
        estimated_ground_z_layer(gmap_index(0), gmap_index(1)) = sum_estimated_ground_z / sum_ground_confidence;
        ground_confidence_layer(gmap_index(0), gmap_index(1)) = (sum_ground_confidence / num_averaged_grid) * ground_confidence_interpolate_decay_;
      }
    }
  }

  // filter point cloud by interpolated ground height
  pcl::PointIndices ground_indices;
  pcl::PointIndices filtered_indices;
  for (unsigned int i = 0; i < transformed_input->points.size(); i++) {
    const auto & p = transformed_input->points[i];
    const auto & p_pos = grid_map::Position(p.x, p.y);
    if (!grid_map_ptr_->isInside(p_pos)) {
      continue;
    }

    grid_map::Index p_index;
    grid_map_ptr_->getIndex(p_pos, p_index);

    // ignore points in sparse areas to remove noise
    if (is_observed_enough_points_layer(p_index(0), p_index(1))) {
      const float estimated_ground_z = estimated_ground_z_layer(p_index(0), p_index(1));
      const float signed_dist = p.z - estimated_ground_z;
      if (signed_dist > ground_distance_threshold_) {
        filtered_indices.indices.push_back(i);
      } else if (std::abs(signed_dist) <= ground_distance_threshold_) {
        ground_indices.indices.push_back(i);
      }
    }
  }

  pcl::ExtractIndices<pcl::PointXYZ> ground_extract_indices;
  ground_extract_indices.setIndices(pcl::make_shared<const pcl::PointIndices>(ground_indices));
  ground_extract_indices.setInputCloud(input);
  ground_extract_indices.filter(*ground);

  pcl::ExtractIndices<pcl::PointXYZ> filtered_extract_indices;
  filtered_extract_indices.setIndices(pcl::make_shared<const pcl::PointIndices>(filtered_indices));
  filtered_extract_indices.setInputCloud(input);
  filtered_extract_indices.filter(*filtered);

  if (publish_debug_ground_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> outlier_extract_indices;
    outlier_extract_indices.setIndices(pcl::make_shared<const pcl::PointIndices>(outlier_indices));
    outlier_extract_indices.setInputCloud(input);
    outlier_extract_indices.filter(*outlier_points);

    sensor_msgs::msg::PointCloud2 outlier_points_msg;
    pcl::toROSMsg(*outlier_points, outlier_points_msg);
    outlier_points_msg.header.frame_id = "odom";
    outlier_points_msg.header.stamp = time;
    debug_outlier_pointcloud_pub_->publish(outlier_points_msg);

    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(*grid_map_ptr_);
    grid_map_msg->header.stamp = time;
    debug_grid_map_pub_->publish(std::move(grid_map_msg));
  }
}

}  // namespace cabot_navigation2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cabot_navigation2::GridMapGroundFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
