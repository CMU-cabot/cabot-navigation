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

#include "cabot_navigation2/abstract_ground_filter_node.hpp"

namespace cabot_navigation2
{

AbstractGroundFilterNode::AbstractGroundFilterNode(const std::string & filter_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(filter_name, options),
  target_frame_("livox_footprint"),
  min_range_(0.05),
  max_range_(5.0),
  min_height_(-1.8),
  max_height_(1.8),
  publish_debug_ground_(false),
  output_debug_ground_topic_("/ground_filter_ground"),
  ground_distance_threshold_(0.05),
  xfer_format_(POINTCLOUD2_XYZRTL),
  ignore_noise_(true),
  input_topic_("/livox/points"),
  output_ground_topic_("/livox/points_ground"),
  output_filtered_topic_("/livox/points_filtered")
{
  RCLCPP_INFO(get_logger(), "NodeClass Constructor");
  RCLCPP_INFO(get_logger(), "Livox Point Cloud Node - %s", __FUNCTION__);

  target_frame_ = declare_parameter("target_frame", target_frame_);
  min_range_ = declare_parameter("min_range", min_range_);
  max_range_ = declare_parameter("max_range", max_range_);
  publish_debug_ground_ = declare_parameter("publish_debug_ground", publish_debug_ground_);
  output_debug_ground_topic_ = declare_parameter("output_debug_ground_topic", output_debug_ground_topic_);
  ground_distance_threshold_ = declare_parameter("ground_distance_threshold", ground_distance_threshold_);
  xfer_format_ = declare_parameter("xfer_format", xfer_format_);
  if (xfer_format_ != POINTCLOUD2_XYZRTL && xfer_format_ != POINTCLOUD2_XYZI) {
    RCLCPP_ERROR(get_logger(), "Invalid format, specify 0 (Livox pointcloud2 format, PointXYZRTL) or 2 (Standard pointcloud2 format, pcl :: PointXYZI).");
  }
  ignore_noise_ = declare_parameter("ignore_noise", ignore_noise_);
  input_topic_ = declare_parameter("input_topic", input_topic_);
  output_ground_topic_ = declare_parameter("output_ground_topic", output_ground_topic_);
  output_filtered_topic_ = declare_parameter("output_filtered_topic", output_filtered_topic_);

  tf_buffer_ = new tf2_ros::Buffer(get_clock());
  tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_, this);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, 10, std::bind(&AbstractGroundFilterNode::pointCloudCallback, this, std::placeholders::_1));

  ground_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_ground_topic_, 1);
  filtered_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_filtered_topic_, 1);
}

AbstractGroundFilterNode::~AbstractGroundFilterNode()
{
  RCLCPP_INFO(get_logger(), "NodeClass Destructor");
  delete tf_listener_;
  delete tf_buffer_;
}

void AbstractGroundFilterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr normal_points(new pcl::PointCloud<pcl::PointXYZ>);
  if (xfer_format_ == POINTCLOUD2_XYZRTL) {
    pcl::PointCloud<CabotLivoxPointXyzrtl>::Ptr input_points(new pcl::PointCloud<CabotLivoxPointXyzrtl>);
    pcl::fromROSMsg(*input, *input_points);

    // filter noise by point cloud tag
    if (ignore_noise_) {
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
          normal_points->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
      }
    } else {
      for (const auto & p : input_points->points) {
        normal_points->push_back(pcl::PointXYZ(p.x, p.y, p.z));
      }
    }
  } else if (xfer_format_ == POINTCLOUD2_XYZI) {
    pcl::fromROSMsg(*input, *normal_points);
  }

  // transform point cloud to target frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_normal_points(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      target_frame_, input->header.frame_id,
      rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));

    pcl_ros::transformPointCloud(*normal_points, *transformed_normal_points, tf_stamped);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  // filter point cloud by range and height
  pcl::PointIndices valid_indices;
  for (unsigned int i = 0; i < transformed_normal_points->points.size(); i++) {
    const auto & p = transformed_normal_points->points[i];
    float r = hypot(p.x, p.y);
    if ((r >= min_range_) && (r <= max_range_) && (p.z >= min_height_) && (p.z <= max_height_)) {
      valid_indices.indices.push_back(i);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr valid_transformed_normal_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> valid_extract_indices;
  valid_extract_indices.setIndices(pcl::make_shared<const pcl::PointIndices>(valid_indices));
  valid_extract_indices.setInputCloud(transformed_normal_points);
  valid_extract_indices.filter(*valid_transformed_normal_points);

  // filter ground point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
  filterGround(input->header.stamp, valid_transformed_normal_points, ground_points, filtered_points);

  // publish ground point cloud
  sensor_msgs::msg::PointCloud2 ground_points_msg;
  pcl::toROSMsg(*ground_points, ground_points_msg);
  ground_points_msg.header = input->header;
  ground_points_msg.header.frame_id = target_frame_;
  ground_pointcloud_pub_->publish(ground_points_msg);

  // publish filtered point cloud
  sensor_msgs::msg::PointCloud2 filtered_points_msg;
  pcl::toROSMsg(*filtered_points, filtered_points_msg);
  filtered_points_msg.header = input->header;
  filtered_points_msg.header.frame_id = target_frame_;
  filtered_pointcloud_pub_->publish(filtered_points_msg);
}

}  // namespace cabot_navigation2
