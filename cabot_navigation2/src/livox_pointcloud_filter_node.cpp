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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
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

namespace CaBot
{

enum LivoxPointCloudMessageType
{
  POINTCLOUD2_XYZRTL = 0,   // Livox pointcloud2(PointXYZRTL) pointcloud format
  LIVOX_POINTCLOUD = 1,     // Livox customized pointcloud format
  POINTCLOUD2_XYZI = 2,     // Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library
};

class LivoxPointCloudFilterNode : public rclcpp::Node
{
public:
  explicit LivoxPointCloudFilterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("livox_pointcloud_filter_node", options),
    xferFormat_(POINTCLOUD2_XYZRTL),
    inputTopicName_("/livox/lidar"),
    outputTopicName_("/livox/lidar_filtered")
  {
    RCLCPP_INFO(get_logger(), "NodeClass Constructor");
    RCLCPP_INFO(get_logger(), "Livox Point Cloud Node - %s", __FUNCTION__);

    xferFormat_ = declare_parameter("xfer_format", xferFormat_);
    if (xferFormat_ != POINTCLOUD2_XYZRTL && xferFormat_ != POINTCLOUD2_XYZI) {
      RCLCPP_ERROR(get_logger(), "Invalid format, specify 0 (Livox pointcloud2 format, PointXYZRTL) or 2 (Standard pointcloud2 format, pcl :: PointXYZI).");
    }
    inputTopicName_ = declare_parameter("input_topic", inputTopicName_);
    pointCloudSub = create_subscription<sensor_msgs::msg::PointCloud2>(
      inputTopicName_, 10, std::bind(&LivoxPointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));

    outputTopicName_ = declare_parameter("output_topic", outputTopicName_);
    pointCloudPub = create_publisher<sensor_msgs::msg::PointCloud2>(outputTopicName_, 1);
  }

  ~LivoxPointCloudFilterNode()
  {
    RCLCPP_INFO(get_logger(), "NodeClass Destructor");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    sensor_msgs::msg::PointCloud2 filtered_points_msg;

    if (xferFormat_ == POINTCLOUD2_XYZRTL) {
      pcl::PointCloud<CabotLivoxPointXyzrtl>::Ptr points(new pcl::PointCloud<CabotLivoxPointXyzrtl>);
      pcl::fromROSMsg(*input, *points);

      pcl::PointCloud<CabotLivoxPointXyzrtl> filtered_points;
      for (const auto & p : points->points) {
        // Livox Mid-70 manual (https://www.livoxtech.com/jp/mid-360/downloads)
        // Livox Mid-360 manual (https://www.livoxtech.com/jp/mid-360/downloads)
        //
        // For Mid-70, tag of normal points is 0 for Bit[0-1], Bit[2-3], Bit[6-7]. Bit[4-5] is return sequence.
        // For Mid-360, tag of normal points is 0 for Bit[0-1], Bit[2-3], Bit[4-5]. Bit[6-7] is reserved.
        if (((p.tag & (1 << 0)) == 0) && ((p.tag & (1 << 1)) == 0) &&
          ((p.tag & (1 << 2)) == 0) && ((p.tag & (1 << 3)) == 0) &&
          ((p.tag & (1 << 6)) == 0) && ((p.tag & (1 << 7)) == 0))
        {
          filtered_points.push_back(p);
        }
      }
      pcl::toROSMsg(filtered_points, filtered_points_msg);
      filtered_points_msg.header = input->header;
    } else if (xferFormat_ == POINTCLOUD2_XYZI) {
      filtered_points_msg = *input;
    }

    pointCloudPub->publish(filtered_points_msg);
  }

  int xferFormat_;
  std::string inputTopicName_;
  std::string outputTopicName_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub;
};  // class LivoxPointCloudFilterNode

}  // namespace CaBot

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaBot::LivoxPointCloudFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
