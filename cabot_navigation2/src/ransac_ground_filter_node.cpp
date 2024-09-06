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

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cabot_navigation2/ransac_ground_filter_node.hpp"

namespace cabot_navigation2
{

RansacGroundFilterNode::RansacGroundFilterNode(const rclcpp::NodeOptions & options)
: AbstractGroundFilterNode("ransac_ground_filter_node", options),
  ransac_max_iteration_(10000),
  ransac_probability_(0.999),
  ransac_eps_angle_(5.0),
  ransac_input_min_height_(-0.50),
  ransac_input_max_height_(0.50),
  ransac_inlier_threshold_(0.01),
  ground_distance_threshold_(0.05),
  debug_(false),
  debug_output_plane_topic_("/ground_filter/ransac_plane")
{
  ransac_max_iteration_ = declare_parameter("ransac_max_iteration", ransac_max_iteration_);
  ransac_probability_ = declare_parameter("ransac_probability", ransac_probability_);
  ransac_eps_angle_ = declare_parameter("ransac_eps_angle", ransac_eps_angle_);
  ransac_input_min_height_ = declare_parameter("ransac_input_min_height", ransac_input_min_height_);
  ransac_input_max_height_ = declare_parameter("ransac_input_max_height", ransac_input_max_height_);
  ransac_inlier_threshold_ = declare_parameter("ransac_inlier_threshold", ransac_inlier_threshold_);
  ground_distance_threshold_ = declare_parameter("ground_distance_threshold", ground_distance_threshold_);
  debug_ = declare_parameter("debug", debug_);
  debug_output_plane_topic_ = declare_parameter("debug_output_plane_topic", debug_output_plane_topic_);

  if (debug_) {
    debug_plane_pub_ = create_publisher<visualization_msgs::msg::Marker>(debug_output_plane_topic_, 1);
  }
}

void RansacGroundFilterNode::filterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered)
{
  // select points which are used to estimate the ground plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_input(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & p : input->points) {
    if ((p.z >= ransac_input_min_height_) && (p.z <= ransac_input_max_height_)) {
      ransac_input->push_back(p);
    }
  }

  // estimate the ground plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMaxIterations(ransac_max_iteration_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ransac_inlier_threshold_);
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
  seg.setEpsAngle(ransac_eps_angle_ * (M_PI / 180.0));
  seg.setProbability(ransac_probability_);
  seg.setInputCloud(ransac_input);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() > 0) {
    // select points above the ground plane
    for (const auto & p : input->points) {
      float signed_dist = pcl::pointToPlaneDistanceSigned(p, coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
      if (signed_dist > ground_distance_threshold_) {
        filtered->push_back(p);
      } else if (abs(signed_dist) <= ground_distance_threshold_) {
        ground->push_back(p);
      }
    }
  } else {
    // if ground plane is not found, select points by fixed height
    RCLCPP_WARN(get_logger(), "failed to estimate the ground plane");
    for (const auto & p : input->points) {
      if (p.z > ground_distance_threshold_) {
        filtered->push_back(p);
      } else if (abs(p.z) <= ground_distance_threshold_) {
        ground->push_back(p);
      }
    }
  }

  if (debug_) {
    visualization_msgs::msg::Marker plane_marker;
    plane_marker.header.frame_id = target_frame_;
    plane_marker.ns = "ransac_plane";
    plane_marker.id = 0;
    plane_marker.type = visualization_msgs::msg::Marker::CUBE;
    plane_marker.action = visualization_msgs::msg::Marker::MODIFY;
    plane_marker.pose.position.x = 0;
    plane_marker.pose.position.y = 0;
    plane_marker.scale.x = 20.0;
    plane_marker.scale.y = 20.0;
    plane_marker.scale.z = 0.0;
    plane_marker.color.r = 1.0f;
    plane_marker.color.g = 0.0f;
    plane_marker.color.b = 0.0f;
    plane_marker.color.a = 0.5;

    if (inliers->indices.size() > 0) {
      const Eigen::Vector3f z_vec(0, 0, 1);
      const Eigen::Vector3f n_vec(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
      const Eigen::Vector3f plane_euler = z_vec.cross(n_vec);

      tf2::Quaternion plane_quat;
      plane_quat.setRPY(plane_euler[0], plane_euler[1], plane_euler[2]);

      plane_marker.pose.position.z = -1.0 * coefficients->values[3] / coefficients->values[2];
      plane_marker.pose.orientation = tf2::toMsg(plane_quat);
    } else {
      plane_marker.pose.position.z = 0.0;
    }
    debug_plane_pub_->publish(plane_marker);
  }
}

}  // namespace cabot_navigation2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cabot_navigation2::RansacGroundFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
