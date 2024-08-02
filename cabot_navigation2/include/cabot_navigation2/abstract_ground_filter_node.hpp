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

#ifndef CABOT_NAVIGATION2__ABSTRACT_GROUND_FILTER_NODE_HPP_
#define CABOT_NAVIGATION2__ABSTRACT_GROUND_FILTER_NODE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

struct EIGEN_ALIGN16 CabotLivoxPointXyzrtl
{
  float x;
  float y;
  float z;
  float reflectivity;
  uint8_t tag;
  uint8_t line;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  CabotLivoxPointXyzrtl,
  (float, x, x)(float, y, y)(float, z, z)(float, reflectivity, reflectivity)(uint8_t, tag, tag)(uint8_t, line, line)
)

namespace cabot_navigation2
{

enum CabotPointCloudMessageType
{
  POINTCLOUD2_XYZRTL = 0,   // Livox pointcloud2(PointXYZRTL) pointcloud format
  LIVOX_POINTCLOUD = 1,     // Livox customized pointcloud format
  POINTCLOUD2_XYZI = 2,     // Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library
};

class AbstractGroundFilterNode : public rclcpp::Node
{
public:
  explicit AbstractGroundFilterNode(const std::string & filter_name, const rclcpp::NodeOptions & options);
  ~AbstractGroundFilterNode();

protected:
  virtual void filterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) = 0;

  std::string target_frame_;

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input);

  int xfer_format_;
  bool ignore_noise_;
  std::string input_topic_;
  std::string output_ground_topic_;
  std::string output_filtered_topic_;
  float min_range_;
  float max_range_;

  tf2_ros::TransformListener * tf_listener_;
  tf2_ros::Buffer * tf_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
};  // class AbstractPointCloudFilterNode

}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__ABSTRACT_GROUND_FILTER_NODE_HPP_
