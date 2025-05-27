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

#include <csignal>
#include <string>
#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include <functional>

#include "cabot/touch_speed_control_node.hpp"

using namespace std::chrono_literals;

namespace CaBotSafety
{
TouchSpeedControlNode::TouchSpeedControlNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("touch_speed_control_node", rclcpp::NodeOptions(options).use_intra_process_comms(false)),
  touch_speed_active_mode_(true),
  touch_speed_max_speed_(2.0),
  touch_speed_max_speed_inactive_(0.5)
{
  // use_intra_process_comms is currently not supported
  // publishers, diagnostic tasks, and related services
  // will be initialized when corresponding meesage arrives
  RCLCPP_INFO(get_logger(), "CABOT Touch Speed Node");

  signal(SIGINT, TouchSpeedControlNode::signalHandler);

  /* touch speed control
  * touch speed activw mode
  * True:  Touch - go,    Not Touch - no go
  * False: Touch - no go, Not Touch - go
  */
  touch_speed_max_speed_ = this->declare_parameter("touch_speed_max_speed", touch_speed_max_speed_);
  touch_speed_max_speed_inactive_ =
    this->declare_parameter("touch_speed_max_speed_inactive", touch_speed_max_speed_inactive_);
  rclcpp::QoS transient_local_qos(1);
  transient_local_qos.transient_local();
  touch_speed_switched_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "touch_speed_switched", transient_local_qos);
  set_touch_speed_active_mode_srv = this->create_service<std_srvs::srv::SetBool>(
    "set_touch_speed_active_mode", std::bind(
      &TouchSpeedControlNode::set_touch_speed_active_mode, this, std::placeholders::_1,
      std::placeholders::_2));

  touch_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    "touch", rclcpp::QoS(10), std::bind(&TouchSpeedControlNode::touch_callback, this, std::placeholders::_1));
}


// Private methods

void TouchSpeedControlNode::signalHandler(int signal)
{
  (void)signal;  // avoid unused variable warning
  exit(0);
}

void TouchSpeedControlNode::touch_callback(std_msgs::msg::Int16::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "touch callback %d", msg->data);
  std::unique_ptr<std_msgs::msg::Float32> touch_speed_msg =
    std::make_unique<std_msgs::msg::Float32>();
  if (touch_speed_active_mode_) {
    touch_speed_msg->data = msg->data ? touch_speed_max_speed_ : 0.0;
  } else {
    touch_speed_msg->data = msg->data ? 0.0 : touch_speed_max_speed_inactive_;
  }
  touch_speed_switched_pub_->publish(std::move(touch_speed_msg));
}

void TouchSpeedControlNode::set_touch_speed_active_mode(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  touch_speed_active_mode_ = req->data;
  if (touch_speed_active_mode_) {
    res->message = "touch speed active mode = True";
  } else {
    res->message = "touch speed active mode = False";
  }
  res->success = true;
}

}  // namespace CaBotSafety

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::TouchSpeedControlNode)
