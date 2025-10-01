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

#include "cabot_navigation2/livox_tag_filter_node.hpp"

namespace cabot_navigation2
{

LivoxTagFilterNode::LivoxTagFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("livox_tag_filter_node", options),
  xfer_format_(POINTCLOUD2_XYZRTL),
  ignore_noise_(true),
  input_topic_("/livox/points"),
  output_filtered_topic_("/livox/points_filtered"),
  queue_size_(2)
{
  RCLCPP_INFO(get_logger(), "NodeClass Constructor");
  RCLCPP_INFO(get_logger(), "Livox Point Cloud Node - %s", __FUNCTION__);

  xfer_format_ = declare_parameter("xfer_format", xfer_format_);
  if (xfer_format_ != POINTCLOUD2_XYZRTL && xfer_format_ != POINTCLOUD2_XYZI) {
    RCLCPP_ERROR(get_logger(), "Invalid format, specify 0 (Livox pointcloud2 format, PointXYZRTL) or 2 (Standard pointcloud2 format, pcl :: PointXYZI).");
  }
  ignore_noise_ = declare_parameter("ignore_noise", ignore_noise_);
  input_topic_ = declare_parameter("input_topic", input_topic_);
  output_filtered_topic_ = declare_parameter("output_filtered_topic", output_filtered_topic_);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, 10, std::bind(&LivoxTagFilterNode::pointCloudCallback, this, std::placeholders::_1));
  filter_tag_timer_ = this->create_wall_timer(std::chrono::duration<float>(0.01), std::bind(&LivoxTagFilterNode::filterTagTimerCallback, this));
  filtered_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_filtered_topic_, 1);
}

LivoxTagFilterNode::~LivoxTagFilterNode()
{
  RCLCPP_INFO(get_logger(), "NodeClass Destructor");
}

void LivoxTagFilterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_pointcloud_.size() < queue_size_) {
    queue_pointcloud_.push(input);
  } else {
    queue_pointcloud_.pop();
    queue_pointcloud_.push(input);
  }
}

void LivoxTagFilterNode::filterTagTimerCallback()
{
  sensor_msgs::msg::PointCloud2::SharedPtr input;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_pointcloud_.size() == 0) {
      return;
    }
    input = queue_pointcloud_.front();
    queue_pointcloud_.pop();
  }

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

  // publish filtered point cloud
  sensor_msgs::msg::PointCloud2 filtered_points_msg;
  pcl::toROSMsg(*normal_points, filtered_points_msg);
  filtered_points_msg.header = input->header;
  filtered_pointcloud_pub_->publish(filtered_points_msg);
}

}  // namespace cabot_navigation2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cabot_navigation2::LivoxTagFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}