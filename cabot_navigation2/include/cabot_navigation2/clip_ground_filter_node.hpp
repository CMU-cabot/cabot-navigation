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

#ifndef CABOT_NAVIGATION2__CLIP_GROUND_FILTER_NODE_HPP_
#define CABOT_NAVIGATION2__CLIP_GROUND_FILTER_NODE_HPP_

#include <visualization_msgs/msg/marker.hpp>

#include "cabot_navigation2/abstract_ground_filter_node.hpp"

namespace cabot_navigation2
{

class ClipGroundFilterNode : public AbstractGroundFilterNode
{
public:
  explicit ClipGroundFilterNode(const rclcpp::NodeOptions & options);

protected:
  void filterGround(const rclcpp::Time & time, const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) override;

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_plane_pub_;
};  // class ClipGroundFilterNode

}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__CLIP_GROUND_FILTER_NODE_HPP_
