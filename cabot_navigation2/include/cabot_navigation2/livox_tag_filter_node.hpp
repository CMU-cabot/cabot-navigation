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

#ifndef CABOT_NAVIGATION2__LIVOX_TAG_FILTER_NODE_HPP_
#define CABOT_NAVIGATION2__LIVOX_TAG_FILTER_NODE_HPP_

#include <string>

#include "cabot_navigation2/abstract_ground_filter_node.hpp"

namespace cabot_navigation2
{

class LivoxTagFilterNode : public rclcpp::Node
{
public:
  explicit LivoxTagFilterNode(const rclcpp::NodeOptions & options);
  ~LivoxTagFilterNode();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
  void filterTagTimerCallback();

  int xfer_format_;
  bool ignore_noise_;
  std::string input_topic_;
  std::string output_filtered_topic_;

  std::mutex mutex_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> queue_pointcloud_;
  int queue_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr filter_tag_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
};  // class LivoxTagFilterNode

}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__LIVOX_TAG_FILTER_NODE_HPP_
