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

#include "cabot_navigation2/clip_ground_filter_node.hpp"

namespace cabot_navigation2
{

ClipGroundFilterNode::ClipGroundFilterNode(const rclcpp::NodeOptions & options)
: AbstractGroundFilterNode("clip_ground_filter_node", options),
  clip_height_(0.05)
{
  clip_height_ = declare_parameter("clip_height", clip_height_);
}

void ClipGroundFilterNode::filterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered)
{
  for (const auto & p : input->points) {
    if (p.z > clip_height_) {
      filtered->push_back(p);
    } else if (abs(p.z) <= clip_height_) {
      ground->push_back(p);
    }
  }
}

}  // namespace cabot_navigation2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cabot_navigation2::ClipGroundFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
