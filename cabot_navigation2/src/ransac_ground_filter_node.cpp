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

#include "cabot_navigation2/ransac_ground_filter_node.hpp"

namespace cabot_navigation2
{

RansacGroundFilterNode::RansacGroundFilterNode(const rclcpp::NodeOptions & options)
: AbstractGroundFilterNode("ransac_ground_filter_node", options),
  ransacMaxIteration_(1000),
  ransacProbability_(0.99),
  ransacEpsAngle_(30.0),
  ransacMinInlier_(1000),
  ransacInputMinThreshold_(-0.10),
  ransacInputMaxThreshold_(0.10),
  ransacInlierThreshold_(0.05)
{
  ransacMaxIteration_ = declare_parameter("ransac_max_iteration", ransacMaxIteration_);
  ransacProbability_ = declare_parameter("ransac_probability", ransacProbability_);
  ransacEpsAngle_ = declare_parameter("ransac_eps_angle", ransacEpsAngle_);
  ransacMinInlier_ = declare_parameter("ransac_min_inlier", ransacMinInlier_);
  ransacInputMinThreshold_ = declare_parameter("ransac_input_min_threshold", ransacInputMinThreshold_);
  ransacInputMaxThreshold_ = declare_parameter("ransac_input_max_threshold", ransacInputMaxThreshold_);
  ransacInlierThreshold_ = declare_parameter("ransac_inlier_threshold", ransacInlierThreshold_);
}

void RansacGroundFilterNode::filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
  // select points which are used to estimate the ground plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_ransac(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & p : input->points) {
    if ((p.z > ransacInputMinThreshold_) && (p.z < ransacInputMaxThreshold_)) {
      input_ransac->push_back(p);
    }
  }

  // estimate the ground plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMaxIterations(ransacMaxIteration_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ransacInlierThreshold_);

  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
  seg.setEpsAngle(ransacEpsAngle_ * (M_PI / 180.0));
  seg.setProbability(ransacProbability_);
  seg.setInputCloud(input_ransac);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() > ransacMinInlier_) {
    // select points above the ground plane
    for (const auto & p : input->points) {
      if (pcl::pointToPlaneDistanceSigned(p, coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]) > ransacInlierThreshold_) {
        output->push_back(p);
      }
    }
  } else {
    // if ground plane is not found, select points by fixed height
    RCLCPP_WARN(get_logger(), "failed to estimate the ground plane, number of inliers points %d", inliers->indices.size());
    for (const auto & p : input->points) {
      if (p.z > ransacInlierThreshold_) {
        output->push_back(p);
      }
    }
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
