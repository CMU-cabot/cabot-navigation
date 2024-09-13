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

#ifndef CABOT__TOUCH_SPEED_CONTROL_NODE_HPP_
#define CABOT__TOUCH_SPEED_CONTROL_NODE_HPP_

#include <rclcpp/node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace CaBotSafety
{
class TouchSpeedControlNode : public rclcpp::Node
{
public:
  explicit TouchSpeedControlNode(const rclcpp::NodeOptions & options);
  ~TouchSpeedControlNode() = default;

private:
  static void signalHandler(int signal);
  void touch_callback(std_msgs::msg::Int16::SharedPtr msg);
  void set_touch_speed_active_mode(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  bool touch_speed_active_mode_;
  double touch_speed_max_speed_;
  double touch_speed_max_speed_inactive_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr touch_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr touch_speed_switched_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_touch_speed_active_mode_srv;
};

}  // namespace CaBotSafety

#endif  // CABOT__TOUCH_SPEED_CONTROL_NODE_HPP_
