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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
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

enum LowObstacleDetectVersion
{
  FIXED_HEIGHT = 1,     // Detect ground point cloud by fixed height
  RANSAC_PLANE = 2,     // Detect ground point cloud by RANSAC
};

class LivoxPointCloudFilterNode : public rclcpp::Node
{
public:
  explicit LivoxPointCloudFilterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("livox_pointcloud_filter_node", options),
    detectVersion_(FIXED_HEIGHT),
    targetFrame_("livox_footprint"),
    xferFormat_(POINTCLOUD2_XYZRTL),
    ignoreNoise_(true),
    inputTopicName_("/livox/lidar"),
    outputTopicName_("/livox/lidar_filtered"),
    clipHeigt_(0.05),
    ransacMaxIteration_(1000),
    ransacProbability_(0.99),
    ransacEpsAngle_(30.0),
    ransacMinInlier_(1000)
  {
    RCLCPP_INFO(get_logger(), "NodeClass Constructor");
    RCLCPP_INFO(get_logger(), "Livox Point Cloud Node - %s", __FUNCTION__);

    detectVersion_ = declare_parameter("low_obstacle_detect_version", detectVersion_);
    if (detectVersion_ != FIXED_HEIGHT && detectVersion_ != RANSAC_PLANE) {
      RCLCPP_ERROR(get_logger(), "Invalid format, specify 1 (detect ground by fixed height) or 2 (detect ground by RANSAC).");
    }
    targetFrame_ = declare_parameter("target_frame", targetFrame_);

    xferFormat_ = declare_parameter("xfer_format", xferFormat_);
    if (xferFormat_ != POINTCLOUD2_XYZRTL && xferFormat_ != POINTCLOUD2_XYZI) {
      RCLCPP_ERROR(get_logger(), "Invalid format, specify 0 (Livox pointcloud2 format, PointXYZRTL) or 2 (Standard pointcloud2 format, pcl :: PointXYZI).");
    }
    ignoreNoise_ = declare_parameter("ignore_noise", ignoreNoise_);
    inputTopicName_ = declare_parameter("input_topic", inputTopicName_);
    outputTopicName_ = declare_parameter("output_topic", outputTopicName_);

    clipHeigt_ = declare_parameter("clip_height", clipHeigt_);
    ransacMaxIteration_ = declare_parameter("ransac_max_iteration", ransacMaxIteration_);
    ransacProbability_ = declare_parameter("ransac_probability", ransacProbability_);
    ransacEpsAngle_ = declare_parameter("ransac_eps_angle", ransacEpsAngle_);
    ransacMinInlier_ = declare_parameter("ransac_min_inlier", ransacMinInlier_);

    tfBuffer_ = new tf2_ros::Buffer(get_clock());
    tfListener_ = new tf2_ros::TransformListener(*tfBuffer_, this);

    pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      inputTopicName_, 10, std::bind(&LivoxPointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));

    pointCloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(outputTopicName_, 1);
  }

  ~LivoxPointCloudFilterNode()
  {
    RCLCPP_INFO(get_logger(), "NodeClass Destructor");
    delete tfListener_;
    delete tfBuffer_;
  }

private:
  void filterGroundFixedHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
  {
    for (const auto & p : input->points) {
      if (p.z > clipHeigt_) {
        output->push_back(p);
      }
    }
  }

  void filterGroundRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
  {
    // select points which are used to estimate the ground plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_ransac(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & p : input->points) {
      if ((p.z > -0.10) && (p.z < 0.10)) {
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
    seg.setDistanceThreshold(clipHeigt_);

    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    seg.setEpsAngle(ransacEpsAngle_ * (M_PI / 180.0));
    seg.setProbability(ransacProbability_);
    seg.setInputCloud(input_ransac);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() > ransacMinInlier_) {
      // select points above the ground plane
      for (const auto & p : input->points) {
        if (pcl::pointToPlaneDistanceSigned(p, coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]) > clipHeigt_) {
          output->push_back(p);
        }
      }
    } else {
      // if ground plane is not found, select points by fixed height
      RCLCPP_WARN(get_logger(), "failed to estimate the ground plane, number of inliers points %d", inliers->indices.size());
      for (const auto & p : input->points) {
        if (p.z > clipHeigt_) {
          output->push_back(p);
        }
      }
    }
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    // filter noise by Livox tag information
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

    // filter point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_points(new pcl::PointCloud<pcl::PointXYZ>);
    if (detectVersion_ == FIXED_HEIGHT) {
      filterGroundFixedHeight(transformed_points, output_points);
    } else if (detectVersion_ == RANSAC_PLANE) {
      filterGroundRANSAC(transformed_points, output_points);
    }

    // publish output point cloud
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*output_points, output);
    output.header = input->header;
    output.header.frame_id = targetFrame_;
    pointCloudPub_->publish(output);
  }

  int detectVersion_;
  std::string targetFrame_;
  int xferFormat_;
  bool ignoreNoise_;
  std::string inputTopicName_;
  std::string outputTopicName_;

  float clipHeigt_;

  int ransacMaxIteration_;
  float ransacProbability_;
  float ransacEpsAngle_;
  int ransacMinInlier_;

  tf2_ros::TransformListener * tfListener_;
  tf2_ros::Buffer * tfBuffer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub_;
};  // class LivoxPointCloudFilterNode

}  // namespace CaBot

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaBot::LivoxPointCloudFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
