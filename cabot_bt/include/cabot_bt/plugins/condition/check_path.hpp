// Copyright (c) 2020, 2024  Carnegie Mellon University
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

#ifndef CABOT_BT__PLUGINS__CONDITION__CHECK_PATH_HPP_
#define CABOT_BT__PLUGINS__CONDITION__CHECK_PATH_HPP_

#include <behaviortree_cpp_v3/condition_node.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <deque>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cabot_bt
{

class CheckPathCondition : public BT::ConditionNode
{
public:
  CheckPathCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  CheckPathCondition() = delete;

  ~CheckPathCondition();

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  void normalize_path(nav_msgs::msg::Path & path);

  void correct_orientation(nav_msgs::msg::Path & path);

  bool check_path();

  bool similar_enough(nav_msgs::msg::Path path, nav_msgs::msg::Path target_path, double average_threshold, double maximum_threshold);

  BT::NodeStatus tick() override;

  void logStuck(const std::string & msg) const;

  static BT::PortsList providedPorts();

private:
  std::atomic<bool> path_okay_;
  std::atomic<bool> target_path_ready_;

  rclcpp::Node::SharedPtr node_;

  nav_msgs::msg::Path target_path_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path current_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

}  // namespace cabot_bt

#endif  // CABOT_BT__PLUGINS__CONDITION__CHECK_PATH_HPP_
