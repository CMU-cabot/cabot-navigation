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
    was_wheelie_active_(false),
    max_speed_(1.0),
    min_speed_(0.0)
  {
    imu_topic_ = this->declare_parameter("imu_topic", "/cabot/imu/data");
    cmd_vel_topic_ = this->declare_parameter("cmd_vel_topic", "/cabot/cmd_vel");
    wheelie_speed_topic_ = this->declare_parameter("wheelie_speed_topic", "/cabot/wheelie_speed");
    pitch_threshold_ = this->declare_parameter("pitch_threshold", -0.15);
    max_speed_ = declare_parameter("max_speed", max_speed_);
    min_speed_ = declare_parameter("min_speed", min_speed_);

    imu_sub_ =
      create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&WheelieControlNode::imuCallback, this, std::placeholders::_1));

    cmd_vel_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&WheelieControlNode::cmdVelCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cabot/cmd_vel", 10);
    wheelie_speed_pub_ = create_publisher<std_msgs::msg::Float32>(wheelie_speed_topic_, 10);

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
    bool wheelie_state = latest_pitch_ < pitch_threshold_;

    std_msgs::msg::Float32 wheelie_msg;
    geometry_msgs::msg::Twist cmd_msg;
    wheelie_msg.data = wheelie_state ? min_speed_ : max_speed_;
    wheelie_speed_pub_->publish(wheelie_msg);

    RCLCPP_INFO(this->get_logger(), "current speed: %.3f (%.3f <=> %.3f)", latest_linear_x_, latest_pitch_, pitch_threshold_);

    if (wheelie_state) {
      cmd_msg.linear.x = min_speed_;
      if (!was_wheelie_active_) {
        was_wheelie_active_ = true;
        last_valid_speed_ = latest_linear_x_;
      }
      RCLCPP_INFO(this->get_logger(), "Wheelie!! Velocity set to %.3f, cmd_msg.linear.x = %.3f", min_speed_, latest_linear_x_);
    } else {
      if (was_wheelie_active_) {
        cmd_msg.linear.x = last_valid_speed_;
        was_wheelie_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Not in Wheelie. ReturnSpeed : %f", last_valid_speed_);
      } else {
        if (latest_linear_x_ > max_speed_) {
          cmd_msg.linear.x = max_speed_;
        } else {
          cmd_msg.linear.x = latest_linear_x_;
        }
        RCLCPP_INFO(this->get_logger(), "Not in Wheelie : %f", latest_linear_x_);
      }
    }

    cmd_vel_pub_->publish(cmd_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheelie_speed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double pitch_threshold_;
  double latest_pitch_;
  double latest_linear_x_;
  double last_valid_speed_;
  bool was_wheelie_active_;
  double max_speed_;
  double min_speed_;

  std::string imu_topic_;
  std::string cmd_vel_topic_;
  std::string wheelie_speed_topic_;
};  // WheelieControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::WheelieControlNode)
