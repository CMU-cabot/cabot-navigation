// Copyright (c) 2024, 2025  Carnegie Mellon University and Miraikan
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace CaBotSafety
{
class WheelieControlNode : public rclcpp::Node
{
public:
  explicit WheelieControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("wheelie_speed_control_node", options),
    latest_pitch_(0.0),
    max_speed_(1.0),
    min_speed_(0.0)
  {
    imu_topic_ = this->declare_parameter("imu_topic", "/cabot/imu/data");
    wheelie_speed_topic_ = this->declare_parameter("wheelie_speed_topic", "/cabot/wheelie_speed");
    gradient_topic_ = this->declare_parameter("gradient_topic", "/cabot/gradient");
    pitch_threshold_ = this->declare_parameter("pitch_threshold", -0.15);
    max_speed_ = declare_parameter("max_speed", max_speed_);
    min_speed_ = declare_parameter("min_speed", min_speed_);

    imu_sub_ =
      create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&WheelieControlNode::imuCallback, this, std::placeholders::_1));

    wheelie_speed_pub_ = create_publisher<std_msgs::msg::Float32>(
      wheelie_speed_topic_,
      rclcpp::SystemDefaultsQoS().transient_local());

    gradient_sub_ =
      create_subscription<std_msgs::msg::Float32>(
      gradient_topic_, 10,
      std::bind(&WheelieControlNode::gradientCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),  // 100ms check
      std::bind(&WheelieControlNode::checkWheelieState, this));

    RCLCPP_INFO(this->get_logger(), "WheelieControlNode has been started.");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Subscribe IMU data and calculate roll, pitch, and yaw from the quaternion.
    // Hold the pitch angle value in latest_pitch_.
    // If the robot is in a wheelie state, the pitch angle will be negative.
    tf2::Quaternion quat(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    latest_pitch_ = pitch;
  }

  void gradientCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Subscribe gradient data and convert the percentage to radians.
    // Hold the gradient offset value in latest_gradient_offset_.
    // If it is an uphill slope, the gradient offset value will be positive.
    if (msg->data > 0.0) {
      latest_gradient_offset_ = atan(msg->data / 100.0);
    } else {
      latest_gradient_offset_ = 0.0;
    }
  }

  void checkWheelieState()
  {
    // When the pitch angle is less than the pitch threshold, the robot is considered to be in a wheelie state.
    // For uphill slopes, adjust the threshold by subtracting the gradient offset.
    std_msgs::msg::Float32 wheelie_speed_msg;
    bool wheelie_state = latest_pitch_ < pitch_threshold_ - latest_gradient_offset_;
    wheelie_speed_msg.data = wheelie_state ? min_speed_ : max_speed_;
    wheelie_speed_pub_->publish(wheelie_speed_msg);
    RCLCPP_INFO(
      this->get_logger(), "speed limit: %.3f (%.6f <=> %.6f - %.6f)",
      wheelie_speed_msg.data, latest_pitch_, pitch_threshold_, latest_gradient_offset_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gradient_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheelie_speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double pitch_threshold_;
  double latest_pitch_;
  double latest_gradient_offset_;
  double max_speed_;
  double min_speed_;

  std::string imu_topic_;
  std::string wheelie_speed_topic_;
  std::string gradient_topic_;
};  // WheelieControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::WheelieControlNode)
