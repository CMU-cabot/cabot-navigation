// Copyright (c) 2024  Carnegie Mellon University
// Copyright (c) 2024  ALPS ALPINE CO., LTD.
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

#ifndef CABOT_NAVIGATION2__CABOT_ROTATION_SHIM_CONTROLLER_HPP_
#define CABOT_NAVIGATION2__CABOT_ROTATION_SHIM_CONTROLLER_HPP_
#include <tf2/LinearMath/Quaternion.h>
#include <nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp>

namespace cabot_navigation2
{
class CaBotRotationShimController : public nav2_rotation_shim_controller::RotationShimController
{
public:
  CaBotRotationShimController();
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr turn_pose_prefer_pub_;

protected:
  geometry_msgs::msg::TwistStamped
  computeRotateToHeadingCommand(
    const double & angular_distance_to_heading,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);
  geometry_msgs::msg::PoseStamped getNearestPathPt(const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::Pose transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt);
};
}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__CABOT_ROTATION_SHIM_CONTROLLER_HPP_
