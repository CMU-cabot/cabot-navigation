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

#include "cabot_navigation2/clip_ground_filter_node.hpp"

namespace cabot_navigation2
{

ClipGroundFilterNode::ClipGroundFilterNode(const rclcpp::NodeOptions & options)
: AbstractGroundFilterNode("clip_ground_filter_node", options)
{
  if (publish_debug_ground_) {
    debug_plane_pub_ = create_publisher<visualization_msgs::msg::Marker>(output_debug_ground_topic_, 1);
  }
}

void ClipGroundFilterNode::filterGround(
  const rclcpp::Time & time, const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground,
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered)
{
  pcl::PointIndices ground_indices;
  pcl::PointIndices filtered_indices;
  for (unsigned int i = 0; i < input->points.size(); i++) {
    const auto & p = input->points[i];
    if (p.z > ground_distance_threshold_) {
      filtered_indices.indices.push_back(i);
    } else if (abs(p.z) <= ground_distance_threshold_) {
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
    visualization_msgs::msg::Marker plane_marker;
    plane_marker.header.frame_id = target_frame_;
    plane_marker.header.stamp = time;
    plane_marker.ns = "clip_plane";
    plane_marker.id = 0;
    plane_marker.type = visualization_msgs::msg::Marker::CUBE;
    plane_marker.action = visualization_msgs::msg::Marker::MODIFY;
    plane_marker.pose.position.x = 0;
    plane_marker.pose.position.y = 0;
    plane_marker.pose.position.z = ground_distance_threshold_;
    plane_marker.scale.x = max_range_ * 2.0;
    plane_marker.scale.y = max_range_ * 2.0;
    plane_marker.scale.z = 0.0;
    plane_marker.color.r = 1.0f;
    plane_marker.color.g = 0.0f;
    plane_marker.color.b = 0.0f;
    plane_marker.color.a = 0.5;
    debug_plane_pub_->publish(plane_marker);
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
