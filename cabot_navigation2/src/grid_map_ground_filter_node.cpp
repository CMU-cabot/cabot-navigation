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
#include <pcl_ros/transforms.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "cabot_navigation2/grid_map_ground_filter_node.hpp"

namespace cabot_navigation2
{

GridMapGroundFilterNode::GridMapGroundFilterNode(const rclcpp::NodeOptions & options)
: AbstractGroundFilterNode("grid_map_ground_filter_node", options),
  odom_topic_("/odom"),
  grid_map_resolution_(0.05),
  grid_map_length_(10.0),
  grid_map_patch_size_(5),
  grid_num_points_min_threshold_(5),
  grid_num_points_raio_threshold_(0.5),
  grid_var_threshold_(0.025),
  grid_prob_prior_(0.5),
  grid_prob_free_(0.1),
  grid_prob_occupied_(0.9),
  grid_prob_forget_rate_(0.05),
  grid_prob_free_threshold_(0.3),
  ground_estimate_angle_min_(-0.614),
  ground_estimate_angle_max_(0.614),
  ground_slope_threshold_(0.262),
  ground_distance_threshold_(0.05)
{
  odom_topic_ = declare_parameter("odom_topic", odom_topic_);
  grid_map_resolution_ = declare_parameter("grid_map_resolution", grid_map_resolution_);
  grid_map_length_ = declare_parameter("grid_map_length", grid_map_length_);
  grid_map_patch_size_ = declare_parameter("grid_map_patch_size", grid_map_patch_size_);
  grid_num_points_min_threshold_ = declare_parameter("grid_num_points_min_threshold", grid_num_points_min_threshold_);
  grid_num_points_raio_threshold_ = declare_parameter("grid_num_points_raio_threshold", grid_num_points_raio_threshold_);
  grid_var_threshold_ = declare_parameter("grid_var_threshold", grid_var_threshold_);
  grid_prob_prior_ = declare_parameter("grid_prob_prior", grid_prob_prior_);
  grid_prob_free_ = declare_parameter("grid_prob_free", grid_prob_free_);
  grid_prob_occupied_ = declare_parameter("grid_prob_occupied", grid_prob_occupied_);
  grid_prob_forget_rate_ = declare_parameter("grid_prob_forget_rate", grid_prob_forget_rate_);
  grid_prob_free_threshold_ = declare_parameter("grid_prob_free_threshold", grid_prob_free_threshold_);
  ground_estimate_angle_min_ = declare_parameter("ground_estimate_angle_min", ground_estimate_angle_min_);
  ground_estimate_angle_max_ = declare_parameter("ground_estimate_angle_max", ground_estimate_angle_max_);
  ground_slope_threshold_ = declare_parameter("ground_slope_threshold", ground_slope_threshold_);
  ground_distance_threshold_ = declare_parameter("ground_distance_threshold", ground_distance_threshold_);

  grid_map_half_patch_size_ = std::floor(grid_map_patch_size_ / 2);
  log_odds_prior_ = std::log(grid_prob_prior_ / (1.0 - grid_prob_prior_));
  log_odds_free_ = std::log(grid_prob_free_ / (1.0 - grid_prob_free_));
  log_odds_occupied_ = std::log(grid_prob_occupied_ / (1.0 - grid_prob_occupied_));
  ground_estimate_radius_ = grid_map_length_ / 2.0;

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, std::bind(&GridMapGroundFilterNode::odomCallback, this, std::placeholders::_1));

  if (publish_debug_ground_) {
    debug_grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(output_debug_ground_topic_, rclcpp::QoS(1).transient_local());
  }
}

void GridMapGroundFilterNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr input)
{
  grid_map::Position input_pos(input->pose.pose.position.x, input->pose.pose.position.y);

  std::unique_lock<std::recursive_mutex> lock(grid_map_mutex_);

  if (!grid_map_ptr_) {
    grid_map_ptr_ =
      std::make_shared<grid_map::GridMap, const std::vector<std::string>>(
      {"num_points", "mean_points_z", "m2_points_z", "var_points_z", "min_points_z", "max_points_z", "log_odds", "occupancy", "binary_free", "observed_ground_z", "binary_estimated_ground", "estimated_ground_z"});
    grid_map_ptr_->setFrameId("odom");
    grid_map_ptr_->setGeometry(grid_map::Length(grid_map_length_, grid_map_length_), grid_map_resolution_, input_pos);
    grid_map_ptr_->move(input_pos);
    (*grid_map_ptr_)["num_points"].setZero();
    (*grid_map_ptr_)["mean_points_z"].setZero();
    (*grid_map_ptr_)["m2_points_z"].setZero();
    (*grid_map_ptr_)["var_points_z"].setZero();
    (*grid_map_ptr_)["min_points_z"].setConstant(std::numeric_limits<float>::max());
    (*grid_map_ptr_)["max_points_z"].setConstant(std::numeric_limits<float>::min());
    (*grid_map_ptr_)["log_odds"].setConstant(log_odds_prior_);
    (*grid_map_ptr_)["occupancy"].setConstant(grid_prob_prior_);
    (*grid_map_ptr_)["binary_free"].setZero();
    (*grid_map_ptr_)["observed_ground_z"].setConstant(std::numeric_limits<float>::max());
    (*grid_map_ptr_)["binary_estimated_ground"].setZero();
    (*grid_map_ptr_)["estimated_ground_z"].setConstant(std::numeric_limits<float>::max());
    return;
  }

  std::vector<grid_map::BufferRegion> new_regions;
  grid_map_ptr_->move(input_pos, new_regions);

  for (auto region : new_regions) {
    for (auto iterator = grid_map::SubmapIterator(*grid_map_ptr_, region); !iterator.isPastEnd(); ++iterator) {
      grid_map::Index gmap_index = *iterator;

      grid_map_ptr_->at("num_points", gmap_index) = 0.0;
      grid_map_ptr_->at("mean_points_z", gmap_index) = 0.0;
      grid_map_ptr_->at("m2_points_z", gmap_index) = 0.0;
      grid_map_ptr_->at("var_points_z", gmap_index) = 0.0;
      grid_map_ptr_->at("min_points_z", gmap_index) = std::numeric_limits<float>::max();
      grid_map_ptr_->at("max_points_z", gmap_index) = std::numeric_limits<float>::min();
      grid_map_ptr_->at("log_odds", gmap_index) = log_odds_prior_;
      grid_map_ptr_->at("occupancy", gmap_index) = grid_prob_prior_;
      grid_map_ptr_->at("binary_free", gmap_index) = 0.0;
      grid_map_ptr_->at("observed_ground_z", gmap_index) = std::numeric_limits<float>::max();
      grid_map_ptr_->at("binary_estimated_ground", gmap_index) = 0.0;
      grid_map_ptr_->at("estimated_ground_z", gmap_index) = std::numeric_limits<float>::max();
    }
  }

  grid_map_ptr_->convertToDefaultStartIndex();
}

int GridMapGroundFilterNode::calcLivoxGridEstimatedNumPoints(float distance, float resolution)
{
  // https://www.livoxtech.com/mid-70/specs
  static const float fov_angle = M_PI * (70.4 / 2.0) / 180.0;
  static const int num_points = 100000;

  const float fov_radius = distance * std::tan(fov_angle);
  const float fov_area = M_PI * std::pow(fov_radius, 2.0);
  return num_points * std::pow(resolution, 2.0) / fov_area;
}

void GridMapGroundFilterNode::filterGround(const rclcpp::Time& time, const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered)
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

  std::unique_lock<std::recursive_mutex> lock(grid_map_mutex_);

  if (!grid_map_ptr_) {
    RCLCPP_WARN(get_logger(), "grid map is not ready");
    return;
  }

  grid_map::Position gmap_origin_pose_position(gmap_origin_pose.pose.position.x, gmap_origin_pose.pose.position.y);
  if (!grid_map_ptr_->isInside(gmap_origin_pose_position)) {
    RCLCPP_WARN(get_logger(), "robot is not inside grid map");
    return;
  }

  // initialize grid map except log odds
  (*grid_map_ptr_)["num_points"].setZero();
  (*grid_map_ptr_)["mean_points_z"].setZero();
  (*grid_map_ptr_)["m2_points_z"].setZero();
  (*grid_map_ptr_)["var_points_z"].setZero();
  (*grid_map_ptr_)["min_points_z"].setConstant(std::numeric_limits<float>::max());
  (*grid_map_ptr_)["max_points_z"].setConstant(std::numeric_limits<float>::min());
  (*grid_map_ptr_)["occupancy"].setConstant(grid_prob_prior_);
  (*grid_map_ptr_)["binary_free"].setZero();
  (*grid_map_ptr_)["observed_ground_z"].setConstant(std::numeric_limits<float>::max());
  (*grid_map_ptr_)["binary_estimated_ground"].setZero();
  (*grid_map_ptr_)["estimated_ground_z"].setConstant(std::numeric_limits<float>::max());

  // add points to grid map
  // TODO(Tatsuya Ishihara): calculate in parallel
  for (const auto & p : transformed_input->points) {
    const auto & p_pos = grid_map::Position(p.x, p.y);
    if (!grid_map_ptr_->isInside(p_pos)) {
      continue;
    }

    grid_map::Index p_index;
    grid_map_ptr_->getIndex(p_pos, p_index);

    const float num_points = (*grid_map_ptr_)["num_points"](p_index(0), p_index(1));
    const float mean_points_z = (*grid_map_ptr_)["mean_points_z"](p_index(0), p_index(1));
    const float m2_points_z = (*grid_map_ptr_)["m2_points_z"](p_index(0), p_index(1));
    const float min_points_z = (*grid_map_ptr_)["min_points_z"](p_index(0), p_index(1));
    const float max_points_z = (*grid_map_ptr_)["max_points_z"](p_index(0), p_index(1));

    const float new_num_points = num_points + 1;
    const float new_mean_points_z = mean_points_z + (p.z - mean_points_z) / new_num_points;
    (*grid_map_ptr_)["num_points"](p_index(0), p_index(1)) = new_num_points;
    (*grid_map_ptr_)["mean_points_z"](p_index(0), p_index(1)) = new_mean_points_z;
    const float new_m2_points_z = m2_points_z + (p.z - new_mean_points_z) * (p.z - mean_points_z);
    (*grid_map_ptr_)["m2_points_z"](p_index(0), p_index(1)) = new_m2_points_z;
    if (new_num_points > 1) {
      (*grid_map_ptr_)["var_points_z"](p_index(0), p_index(1)) = new_m2_points_z / (new_num_points - 1);
    }
    (*grid_map_ptr_)["min_points_z"](p_index(0), p_index(1)) = std::min(p.z, min_points_z);
    (*grid_map_ptr_)["max_points_z"](p_index(0), p_index(1)) = std::max(p.z, max_points_z);
  }

  // update free area
  // TODO(Tatsuya Ishihara): calculate in parallel
  const grid_map::Size gmap_size = grid_map_ptr_->getSize();
  const float gmap_resolution = grid_map_ptr_->getResolution();
  for (auto iterator = grid_map::GridMapIterator(*grid_map_ptr_); !iterator.isPastEnd(); ++iterator) {
    grid_map::Index gmap_index = *iterator;

    const float grid_distance = std::sqrt((std::pow(gmap_index(0) - (gmap_size(0) / 2.0), 2.0) + std::pow(gmap_index(1) - (gmap_size(1) / 2.0), 2.0))) * gmap_resolution;
    const int grid_estimated_num_points = calcLivoxGridEstimatedNumPoints(grid_distance, gmap_resolution);

    const auto & num_points_patch =
      (*grid_map_ptr_)["num_points"].block(gmap_index(0) - grid_map_half_patch_size_, gmap_index(1) - grid_map_half_patch_size_, grid_map_patch_size_, grid_map_patch_size_);
    const auto & var_points_z_patch =
      (*grid_map_ptr_)["var_points_z"].block(gmap_index(0) - grid_map_half_patch_size_, gmap_index(1) - grid_map_half_patch_size_, grid_map_patch_size_, grid_map_patch_size_);

    const float & num_points_patch_sum = num_points_patch.sum();
    if ((num_points_patch_sum > grid_num_points_min_threshold_) && (num_points_patch_sum > grid_num_points_raio_threshold_ * grid_estimated_num_points)) {
      // observe enough point, update log odds by current observation
      const float & var_points_z_patch_mean = var_points_z_patch.cwiseProduct(num_points_patch).sum() / num_points_patch_sum;
      if (var_points_z_patch_mean > grid_var_threshold_) {
        // update log odds as occupied area
        grid_map_ptr_->at("log_odds", gmap_index) = grid_map_ptr_->at("log_odds", gmap_index) - log_odds_prior_ + log_odds_occupied_;
      } else {
        // update log odds as free area
        grid_map_ptr_->at("log_odds", gmap_index) = grid_map_ptr_->at("log_odds", gmap_index) - log_odds_prior_ + log_odds_free_;
      }
    } else {
      // do not observe enough point, forget previous updates of log odds
      grid_map_ptr_->at("log_odds", gmap_index) = log_odds_prior_ + (grid_map_ptr_->at("log_odds", gmap_index) - log_odds_prior_) * (1.0 - grid_prob_forget_rate_);
    }
    const float occupancy = 1.0 - 1.0 / (1.0 + std::exp(grid_map_ptr_->at("log_odds", gmap_index)));
    grid_map_ptr_->at("occupancy", gmap_index) = occupancy;

    const bool is_free = (occupancy < grid_prob_free_threshold_) ? 1.0 : 0.0;
    grid_map_ptr_->at("binary_free", gmap_index) = is_free;
  }

  // set ground height for robot position
  grid_map::Index gmap_origin_pose_index;
  grid_map_ptr_->getIndex(gmap_origin_pose_position, gmap_origin_pose_index);
  grid_map_ptr_->at("observed_ground_z", gmap_origin_pose_index) = gmap_origin_pose.pose.position.z;
  grid_map_ptr_->at("binary_estimated_ground", gmap_origin_pose_index) = 1.0;
  grid_map_ptr_->at("estimated_ground_z", gmap_origin_pose_index) = gmap_origin_pose.pose.position.z;

  // calculate ground height for free observed area
  // TODO(Tatsuya Ishihara): calculate in parallel
  for (auto iterator = grid_map::SpiralIterator(*grid_map_ptr_, gmap_origin_pose_position, ground_estimate_radius_); !iterator.isPastEnd(); ++iterator) {
    if (iterator.getCurrentRadius()==0.0) {
      continue;
    }
 
    grid_map::Index gmap_index = *iterator;

    grid_map::Position gmap_position;
    grid_map_ptr_->getPosition(gmap_index, gmap_position);
    if (!grid_map_ptr_->isInside(gmap_position)) {
      continue;
    }

    float observed_ground_z = std::numeric_limits<float>::max();
    if (grid_map_ptr_->at("binary_free", gmap_index)) {
      // calculate ground height by findinig minimum point height in the patch
      const auto & min_points_z_patch =
        (*grid_map_ptr_)["min_points_z"].block(gmap_index(0) - grid_map_half_patch_size_, gmap_index(1) - grid_map_half_patch_size_, grid_map_patch_size_, grid_map_patch_size_);
      const auto & binary_free_patch =
        (*grid_map_ptr_)["binary_free"].block(gmap_index(0) - grid_map_half_patch_size_, gmap_index(1) - grid_map_half_patch_size_, grid_map_patch_size_, grid_map_patch_size_);
      for (int row = 0; row < binary_free_patch.rows(); row++) {
        for (int col = 0; col < binary_free_patch.cols(); col++) {
          if ((binary_free_patch(row, col) == 1.0) && (min_points_z_patch(row, col) < observed_ground_z)) {
            observed_ground_z = min_points_z_patch(row, col);
          }
        }
      }
    }

    if (observed_ground_z != std::numeric_limits<float>::max()) {
      // check if ground height is valid by calculating slope to the closest observed ground height in line of sight
      bool is_valid = true;
      for (auto los_iterator = grid_map::LineIterator(*grid_map_ptr_, gmap_position, gmap_origin_pose_position); !los_iterator.isPastEnd(); ++los_iterator) {
        grid_map::Index gmap_los_index = *los_iterator;
        if (grid_map_ptr_->at("observed_ground_z", gmap_los_index) != std::numeric_limits<float>::max()) {
          grid_map::Position gmap_los_position;
          grid_map_ptr_->getPosition(gmap_los_index, gmap_los_position);
          double dist = std::hypot(gmap_position.x() - gmap_los_position.x(), gmap_position.y() - gmap_los_position.y());
          double slope = std::atan2(observed_ground_z - grid_map_ptr_->at("observed_ground_z", gmap_los_index), dist);
          if (slope > ground_slope_threshold_) {
            is_valid = false;
          }
          break;
        }
      }

      if (is_valid) {
        grid_map_ptr_->at("observed_ground_z", gmap_index) = observed_ground_z;
        grid_map_ptr_->at("binary_estimated_ground", gmap_index) = 1.0;
        grid_map_ptr_->at("estimated_ground_z", gmap_index) = observed_ground_z;
      }
    }
  }

  // estimate ground height from center of grid map in spiral order
  double gmap_origin_pose_yaw = tf2::getYaw(gmap_origin_pose.pose.orientation);
  for (auto iterator = grid_map::SpiralIterator(*grid_map_ptr_, gmap_origin_pose_position, ground_estimate_radius_); !iterator.isPastEnd(); ++iterator) {
    if (iterator.getCurrentRadius()==0.0) {
      continue;
    }

    grid_map::Index gmap_index = *iterator;

    grid_map::Position gmap_position;
    grid_map_ptr_->getPosition(gmap_index, gmap_position);
    if (!grid_map_ptr_->isInside(gmap_position)) {
      continue;
    }

    float angle = std::atan2(gmap_position.y() - gmap_origin_pose_position.y(), gmap_position.x() - gmap_origin_pose_position.x()) - gmap_origin_pose_yaw;
    if (angle < -M_PI) {
      angle += 2.0 * M_PI;
    } else if (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    if ((angle>=ground_estimate_angle_min_) && (angle<=ground_estimate_angle_max_)) {
      if (!(*grid_map_ptr_)["binary_estimated_ground"](gmap_index(0), gmap_index(1))) {
        // estimate ground height by surrounding free area
        const auto & binary_estimated_patch =
          (*grid_map_ptr_)["binary_estimated_ground"].block(gmap_index(0) - grid_map_half_patch_size_, gmap_index(1) - grid_map_half_patch_size_, grid_map_patch_size_, grid_map_patch_size_);
        const float binary_estimated_patch_sum = binary_estimated_patch.sum();
        if (binary_estimated_patch_sum > 0) {
          const auto & estimated_ground_z_patch =
            (*grid_map_ptr_)["estimated_ground_z"].block(gmap_index(0) - grid_map_half_patch_size_, gmap_index(1) - grid_map_half_patch_size_, grid_map_patch_size_, grid_map_patch_size_);
          const float estimated_ground_z = estimated_ground_z_patch.cwiseProduct(binary_estimated_patch).sum() / binary_estimated_patch_sum;
          (*grid_map_ptr_)["binary_estimated_ground"](gmap_index(0), gmap_index(1)) = 1.0;
          (*grid_map_ptr_)["estimated_ground_z"](gmap_index(0), gmap_index(1)) = estimated_ground_z;
        }
      }
    }
  }

  // filter point cloud by intepolated ground height
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

    const float estimated_ground_z = (*grid_map_ptr_)["estimated_ground_z"](p_index(0), p_index(1));
    const float signed_dist = p.z - estimated_ground_z;
    if (signed_dist > ground_distance_threshold_) {
      filtered_indices.indices.push_back(i);
    } else if (abs(signed_dist) <= ground_distance_threshold_) {
      ground_indices.indices.push_back(i);
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
    // publish grid map
    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(*grid_map_ptr_);
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
