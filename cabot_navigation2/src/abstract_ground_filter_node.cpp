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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include "cabot_navigation2/abstract_ground_filter_node.hpp"

namespace cabot_navigation2
{

AbstractGroundFilterNode::AbstractGroundFilterNode(const std::string & filter_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(filter_name, options),
  targetFrame_("livox_footprint"),
  xferFormat_(POINTCLOUD2_XYZRTL),
  ignoreNoise_(true),
  inputTopicName_("/livox/lidar"),
  outputTopicName_("/livox/lidar_filtered")
{
  RCLCPP_INFO(get_logger(), "NodeClass Constructor");
  RCLCPP_INFO(get_logger(), "Livox Point Cloud Node - %s", __FUNCTION__);

  targetFrame_ = declare_parameter("target_frame", targetFrame_);

  xferFormat_ = declare_parameter("xfer_format", xferFormat_);
  if (xferFormat_ != POINTCLOUD2_XYZRTL && xferFormat_ != POINTCLOUD2_XYZI) {
    RCLCPP_ERROR(get_logger(), "Invalid format, specify 0 (Livox pointcloud2 format, PointXYZRTL) or 2 (Standard pointcloud2 format, pcl :: PointXYZI).");
  }
  ignoreNoise_ = declare_parameter("ignore_noise", ignoreNoise_);
  inputTopicName_ = declare_parameter("input_topic", inputTopicName_);
  outputTopicName_ = declare_parameter("output_topic", outputTopicName_);

  tfBuffer_ = new tf2_ros::Buffer(get_clock());
  tfListener_ = new tf2_ros::TransformListener(*tfBuffer_, this);

  pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    inputTopicName_, 10, std::bind(&AbstractGroundFilterNode::pointCloudCallback, this, std::placeholders::_1));

  pointCloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(outputTopicName_, 1);
}

AbstractGroundFilterNode::~AbstractGroundFilterNode()
{
  RCLCPP_INFO(get_logger(), "NodeClass Destructor");
  delete tfListener_;
  delete tfBuffer_;
}

void AbstractGroundFilterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  // filter noise if point cloud tag is available
  pcl::PointCloud<pcl::PointXYZ> normal_points;
  if ((xferFormat_ == POINTCLOUD2_XYZRTL) && ignoreNoise_) {
    pcl::PointCloud<CabotLivoxPointXyzrtl>::Ptr input_points(new pcl::PointCloud<CabotLivoxPointXyzrtl>);
    pcl::fromROSMsg(*input, *input_points);

    for (const auto & p : input_points->points) {
      // Livox Mid-70 manual (https://www.livoxtech.com/jp/mid-70/downloads)
      // Livox Mid-360 manual (https://www.livoxtech.com/jp/mid-360/downloads)
      //
      // For Mid-70, tag of normal points is 0 for Bit[0-1], Bit[2-3], Bit[6-7]. Bit[4-5] is return sequence.
      // For Mid-360, tag of normal points is 0 for Bit[0-1], Bit[2-3], Bit[4-5]. Bit[6-7] is reserved.
      if (((p.tag & (1 << 0)) == 0) && ((p.tag & (1 << 1)) == 0) &&
        ((p.tag & (1 << 2)) == 0) && ((p.tag & (1 << 3)) == 0) &&
        ((p.tag & (1 << 6)) == 0) && ((p.tag & (1 << 7)) == 0))
      {
        normal_points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
      }
    }
  } else if (xferFormat_ == POINTCLOUD2_XYZI) {
    pcl::fromROSMsg(*input, normal_points);
  }

  // transform point cloud to target frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tfBuffer_->lookupTransform(
      targetFrame_, input->header.frame_id,
      rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));

    pcl_ros::transformPointCloud(normal_points, *transformed_points, tf_stamped);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  // filter ground point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_points(new pcl::PointCloud<pcl::PointXYZ>);
  filterGround(transformed_points, output_points);

  // publish output point cloud
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*output_points, output);
  output.header = input->header;
  output.header.frame_id = targetFrame_;
  pointCloudPub_->publish(output);
}

}  // namespace cabot_navigation2
