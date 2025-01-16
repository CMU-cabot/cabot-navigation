// Copyright (c) 2024  Carnegie Mellon University
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
  : rclcpp::Node("wheelie_control_node", options),
    latest_pitch_(0.0),
    last_valid_speed_(0.0),
    was_wheelie_active_(false)
  {
    imu_topic_ = this->declare_parameter("imu_topic", "/cabot/imu/data");
    cmd_vel_topic_ = this->declare_parameter("cmd_vel_topic", "/cabot/cmd_vel");
    wheelie_state_topic_ = this->declare_parameter("wheelie_state_topic", "/cabot/wheelie_state");

    imu_sub_ =
      create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&WheelieControlNode::imuCallback, this, std::placeholders::_1));

    cmd_vel_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&WheelieControlNode::cmdVelCallback, this, std::placeholders::_1));

    cmd_vel_test_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cabot/cmd_vel_test", 10);
    wheelie_state_pub_ = create_publisher<std_msgs::msg::Bool>(wheelie_state_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),  // 100ms check
      std::bind(&WheelieControlNode::checkWheelieState, this));

    RCLCPP_INFO(this->get_logger(), "WheelieControlNode has been started.");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
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

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_linear_x_ = msg->linear.x;
  }

  void checkWheelieState()
  {
    bool new_wheelie_state = latest_pitch_ < pitch_threshold_;

    std_msgs::msg::Bool wheelie_msg;
    wheelie_msg.data = new_wheelie_state;
    wheelie_state_pub_->publish(wheelie_msg);

    RCLCPP_INFO(this->get_logger(), "Wheelie state: %s", new_wheelie_state ? "True" : "False");

    geometry_msgs::msg::Twist cmd_msg;
    if (new_wheelie_state) {
      cmd_msg.linear.x = 0.0;
      if (!was_wheelie_active_) {
        was_wheelie_active_ = true;
        last_valid_speed_ = latest_linear_x_;
      }
      RCLCPP_INFO(this->get_logger(), "Velocity set to 0");
    } else {
      if (was_wheelie_active_) {
        cmd_msg.linear.x = last_valid_speed_;
        was_wheelie_active_ = false;
        RCLCPP_INFO(this->get_logger(), "ReturnSpeed : %f", last_valid_speed_);
      } else {
        cmd_msg.linear.x = latest_linear_x_;
        RCLCPP_INFO(this->get_logger(), "Speed : %f", latest_linear_x_);
      }
    }

    cmd_vel_test_pub_->publish(cmd_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wheelie_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_test_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double pitch_threshold_;
  double latest_pitch_;
  double latest_linear_x_;
  double last_valid_speed_;
  bool was_wheelie_active_;

  std::string imu_topic_;
  std::string cmd_vel_topic_;
  std::string wheelie_state_topic_;
};  // WheelieControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::WheelieControlNode)
