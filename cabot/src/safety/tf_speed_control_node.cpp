// Copyright (c) 2020, 2022  Carnegie Mellon University
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

// tf speed control
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace CaBotSafety
{
class TFSpeedControlNode : public rclcpp::Node
{
public:
  std::string limit_topic_;
  double max_speed_;
  double check_rate_;
  double tf_timeout_;
  std::string map_frame_;
  std::string robot_base_frame_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
  tf2_ros::TransformListener * tfListener;
  tf2_ros::Buffer * tfBuffer;
  rclcpp::TimerBase::SharedPtr timer_;

  explicit TFSpeedControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tf_speed_control_node", options),
    limit_topic_("tf_limit"),
    max_speed_(1.0),
    check_rate_(5.0),
    tf_timeout_(0.2),
    map_frame_("map"),
    robot_base_frame_("base_footprint")
  {
    RCLCPP_INFO(get_logger(), "TFSpeedControlNodeClass Constructor");
    tfBuffer = new tf2_ros::Buffer(get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer, this);
    onInit();
  }

  ~TFSpeedControlNode()
  {
    RCLCPP_INFO(get_logger(), "TFSpeedControlNodeClass Destructor");
    limit_pub_.reset();
    delete tfListener;
    delete tfBuffer;
    timer_.reset();
  }

private:
  void onInit()
  {
    RCLCPP_INFO(get_logger(), "tf speed control - %s", __FUNCTION__);

    limit_topic_ = declare_parameter("limit_topic", limit_topic_);
    max_speed_ = declare_parameter("max_speed", max_speed_);  // [m/s]
    check_rate_ = declare_parameter("check_rate", check_rate_);  // [Hz]
    tf_timeout_ = declare_parameter("tf_timeout", tf_timeout_);  // [seconds]

    limit_pub_ = create_publisher<std_msgs::msg::Float32>(limit_topic_, rclcpp::SystemDefaultsQoS().transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / check_rate_),
      std::bind(&TFSpeedControlNode::tfCheckLoop, this));
  }

  void tfCheckLoop()
  {
    double speed_limit = max_speed_;

    try {
      geometry_msgs::msg::TransformStamped transform_msg = tfBuffer->lookupTransform(
        robot_base_frame_, map_frame_, get_clock()->now(), rclcpp::Duration(std::chrono::duration<double>(tf_timeout_)));
      RCLCPP_INFO(get_logger(), "TFSpeedControl, lookup transform success");
    } catch (tf2::TransformException & ex) {
      speed_limit = 0.0;
      RCLCPP_INFO(get_logger(), "TFSpeedControl, lookup transform fail");
    }

    std_msgs::msg::Float32 msg;
    msg.data = speed_limit;
    limit_pub_->publish(msg);
  }
};  // class TFSpeedControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::TFSpeedControlNode)
