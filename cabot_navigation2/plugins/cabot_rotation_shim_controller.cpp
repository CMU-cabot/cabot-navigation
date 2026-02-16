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


#include <cabot_navigation2/cabot_rotation_shim_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotRotationShimController, nav2_core::Controller)

namespace cabot_navigation2
{

CaBotRotationShimController::CaBotRotationShimController()
: nav2_rotation_shim_controller::RotationShimController()
{
  turn_pose_prefer_pub_ = nullptr;
}

geometry_msgs::msg::TwistStamped
CaBotRotationShimController::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  const double angular_vel = sign * std::min(std::abs(angular_distance_to_heading), rotate_to_heading_angular_vel_);
  const double & dt = control_duration_;
  const double min_feasible_angular_speed = velocity.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = velocity.angular.z + max_angular_accel_ * dt;
  cmd_vel.twist.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  isCollisionFree(cmd_vel, angular_distance_to_heading, pose);
  return cmd_vel;
}

void CaBotRotationShimController::setPlan(const nav_msgs::msg::Path & path)
{
  auto path_updated = isGoalChanged(path);
  RotationShimController::setPlan(path);
  path_updated_ = path_updated;

  RCLCPP_INFO(logger_, "Path updated: %s", path_updated_ ? "true" : "false");
}

bool CaBotRotationShimController::isGoalChanged(const nav_msgs::msg::Path & path)
{
  // Return true if rotating or if the current path is empty
  if (in_rotation_ || current_path_.poses.empty()) {
    return true;
  }

  // Check if the last pose of the current and new paths differ
  auto p0 = current_path_.poses.back().pose;
  auto p1 = path.poses.back().pose;
  auto dx = p0.position.x - p1.position.x;
  auto dy = p0.position.y - p1.position.y;
  RCLCPP_INFO(
    logger_, "Path goal changed: %f (%.2f, %.2f) - (%.2f, %.2f)", std::hypot(dx, dy),
    p0.position.x, p0.position.y, p1.position.x, p1.position.y);
  return std::hypot(dx, dy) > 1.0;
}

geometry_msgs::msg::TwistStamped CaBotRotationShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto node = node_.lock();

  if (path_updated_) {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    std::lock_guard<std::mutex> lock_reinit(mutex_);
    try {
      geometry_msgs::msg::Pose sampled_pt_base = transformPoseToBaseFrame(getNearestPathPt(pose));

      double angular_distance_to_heading =
        std::atan2(sampled_pt_base.position.y, sampled_pt_base.position.x);
      double distance_to_heading = hypot(sampled_pt_base.position.x, sampled_pt_base.position.y);

      RCLCPP_INFO(
        logger_,
        "angular_distance_to_heading: abs(%.2f) <> %.2f, distance_to_heading: %.2f <> %.2f",
        angular_distance_to_heading, angular_dist_threshold_, distance_to_heading, forward_sampling_distance_);
      if (distance_to_heading >= forward_sampling_distance_ * 0.9 &&
        fabs(angular_distance_to_heading) > angular_dist_threshold_ && velocity.linear.x < 0.1)
      {
        RCLCPP_INFO(
          logger_,
          "Robot is not within the new path's rough heading, rotating to heading...");
        if (turn_pose_prefer_pub_ == nullptr) {
          turn_pose_prefer_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/cabot/turn_pose_prefer", rclcpp::QoS(10));
          geometry_msgs::msg::PoseStamped msg;
          tf2::Quaternion quaternion;
          msg.header.stamp = clock_->now();
          msg.header.frame_id = costmap_ros_->getBaseFrameID();
          quaternion.setRPY(0.0, 0.0, angular_distance_to_heading);
          tf2::convert(quaternion, msg.pose.orientation);
          turn_pose_prefer_pub_->publish(msg);
        }
        return computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
      } else {
        RCLCPP_INFO(
          logger_,
          "Robot is at the new path's rough heading, passing to controller");
        path_updated_ = false;
        turn_pose_prefer_pub_ = nullptr;
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_DEBUG(
        logger_,
        "Rotation Shim Controller was unable to find a sampling point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());
      path_updated_ = false;
    }
  }

  // If at this point, use the primary controller to path track
  return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
}

geometry_msgs::msg::PoseStamped CaBotRotationShimController::getNearestPathPt(const geometry_msgs::msg::PoseStamped & pose)
{
  if (current_path_.poses.size() < 2) {
    throw nav2_core::PlannerException(
            "Path is too short to find a valid sampled path point for rotation.");
  }

  // find the nearest pose in the path from the current robot pose
  double dx, dy, dist;
  double mind = 10000;
  unsigned int mini;
  for (unsigned int i = 1; i != current_path_.poses.size() && i < 100; i++) {
    dx = current_path_.poses[i].pose.position.x - pose.pose.position.x;
    dy = current_path_.poses[i].pose.position.y - pose.pose.position.y;
    dist = hypot(dx, dy);
    if (dist < mind) {
      mind = dist;
      mini = i;
    }
  }

  RCLCPP_INFO(
    node_.lock()->get_logger(), "minimum distance %ld %.2f (%.2f, %.2f) - (%.2f, %.2f)",
    mini, mind, current_path_.poses[mini].pose.position.x, current_path_.poses[mini].pose.position.y,
    pose.pose.position.x, pose.pose.position.y);

  geometry_msgs::msg::Pose start = current_path_.poses.front().pose;
  // Find the first point at least sampling distance away from the nearest point
  for (unsigned int i = mini; i != current_path_.poses.size(); i++) {
    dx = current_path_.poses[i].pose.position.x - start.position.x;
    dy = current_path_.poses[i].pose.position.y - start.position.y;
    if (hypot(dx, dy) >= forward_sampling_distance_) {
      current_path_.poses[i].header.frame_id = current_path_.header.frame_id;
      current_path_.poses[i].header.stamp = clock_->now();  // Get current time transformation
      RCLCPP_INFO(
        node_.lock()->get_logger(), "return %ld %.2f (%.2f, %.2f) - (%.2f, %.2f)",
        i, hypot(dx, dy), current_path_.poses[i].pose.position.x, current_path_.poses[i].pose.position.y,
        pose.pose.position.x, pose.pose.position.y);
      return current_path_.poses[i];
    }
  }

  throw nav2_core::PlannerException(
          std::string(
            "Unable to find a sampling point at least %0.2f from the robot,"
            "passing off to primary controller plugin.", forward_sampling_distance_));
}

geometry_msgs::msg::Pose
CaBotRotationShimController::transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt)
{
  auto node = node_.lock();
  geometry_msgs::msg::PoseStamped pt_base;
  if (!nav2_util::transformPoseInTargetFrame(pt, pt_base, *tf_, costmap_ros_->getBaseFrameID(), 1.0)) {
    throw nav2_core::PlannerException("Failed to transform pose to base frame!");
  }
  return pt_base.pose;
}
}  // namespace cabot_navigation2
