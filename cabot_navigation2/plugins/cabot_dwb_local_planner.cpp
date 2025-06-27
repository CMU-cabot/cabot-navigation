// Copyright (c) 2024  Carnegie Mellon University
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

#include <cabot_navigation2/cabot_dwb_local_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <dwb_core/exceptions.hpp>
#include <dwb_core/illegal_trajectory_tracker.hpp>
#include <nav_2d_utils/tf_help.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_util/geometry_utils.hpp>
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotDWBLocalPlanner, nav2_core::Controller)

using nav2_util::geometry_utils::euclidean_distance;

namespace cabot_navigation2
{

CaBotDWBLocalPlanner::CaBotDWBLocalPlanner()
: dwb_core::DWBLocalPlanner()
{
}

// CaBotDWBLocalPlanner extends transformGlobalPlan method of DWBLocalPlanner
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

nav_2d_msgs::msg::Twist2DStamped
CaBotDWBLocalPlanner::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = clock_->now();
  }

  nav_2d_msgs::msg::Path2D transformed_plan;
  nav_2d_msgs::msg::Pose2DStamped goal_pose;

  // adjust forward_prune_distance based on the current speed
  auto node = node_.lock();
  double temp = forward_prune_distance_;
  double temp2 = prune_distance_;
  double current_speed = std::hypot(velocity.x, velocity.y);
  double max_speed_xy;
  node->get_parameter(dwb_plugin_name_ + ".max_speed_xy", max_speed_xy);
  forward_prune_distance_ = temp * std::max(0.25, std::min(1.0, (current_speed + 0.25) / max_speed_xy));
  prune_distance_ = temp2 * std::max(0.25, std::min(1.0, (current_speed + 0.25) / max_speed_xy));
  prepareGlobalPlan(pose, transformed_plan, goal_pose);
  forward_prune_distance_ = temp;
  prune_distance_ = temp2;

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
    if (!critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan)) {
      RCLCPP_WARN(rclcpp::get_logger("DWBLocalPlanner"), "A scoring function failed to prepare");
    }
  }

  try {
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);

    // Return Value
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, best.traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException & e) {
    nav_2d_msgs::msg::Twist2D empty_cmd;
    dwb_msgs::msg::Trajectory2D empty_traj;
    // debrief stateful scoring functions
    for (dwb_core::TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(empty_cmd);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, empty_traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    throw;
  }
}

nav_2d_msgs::msg::Path2D
CaBotDWBLocalPlanner::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  if (!nav_2d_utils::transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw dwb_core::
          PlannerTFException("Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the plan
  // near the robot (the end of the plan). This determines the amount of the
  // plan passed on to the critics
  double transform_end_threshold;
  double forward_prune_dist = forward_prune_distance_;
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  } else {
    transform_end_threshold = dist_threshold;
  }

  // customize begin
  // Customized: Find the closest point from the robot pose
  auto nearest_point_from_robot = std::min_element(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&](const auto & pose1, const auto & pose2) {
      return euclidean_distance(pose1, robot_pose.pose) < euclidean_distance(pose2, robot_pose.pose);
    });
  double nearest_distance = euclidean_distance(*nearest_point_from_robot, robot_pose.pose);

  // Customized: Find the first pose less than half of transform_start_threshold - adhoc solution
  auto search_start_point = nearest_point_from_robot;
  if (nearest_distance < transform_start_threshold / 2) {
    search_start_point = std::find_if(
      global_plan_.poses.begin(), global_plan_.poses.end(),
      [&](const auto & pose) {
        return euclidean_distance(pose, robot_pose.pose) < transform_start_threshold / 2;
      });
  }

  // Customized: Find the first pose in the end of the plan that's further than transform_end_threshold
  // from the robot using integrated distance
  // accumulate distance so that the search can ignore after a sharp turn
  double accumulated_distance = 0.0;
  auto transformation_end = std::find_if(
    search_start_point, global_plan_.poses.end(),
    [&](const auto & pose) {
      if (pose != *search_start_point) {
        accumulated_distance += euclidean_distance(*std::prev(&pose), pose);
      }
      return accumulated_distance > transform_end_threshold + transform_start_threshold / 2;
    });

  // Customized: Find the first pose in the beginning of the plan that's further than transform_start_threshold
  // from the robot using integrated distance
  // accumulate distance so that the search can ignore after a sharp turn
  accumulated_distance = 0.0;
  auto transformation_begin_reverse = std::find_if(
    std::make_reverse_iterator(search_start_point), global_plan_.poses.rend(),
    [&](const auto & pose) {
      if (pose != *search_start_point) {
        accumulated_distance += euclidean_distance(*std::prev(&pose), pose);
      }
      return accumulated_distance > transform_start_threshold - transform_start_threshold / 2;
    });
  auto transformation_begin = (transformation_begin_reverse.base());

  RCLCPP_INFO(
    logger_, "transformation_begin = %ld, transformation_end = %ld, size = %ld",
    std::distance(global_plan_.poses.begin(), transformation_begin),
    std::distance(global_plan_.poses.begin(), transformation_end),
    global_plan_.poses.size());
  if (transformation_end < transformation_begin) {
    RCLCPP_ERROR(logger_, "transformation_begin should be lower than transformation_end");
    transformation_begin = transformation_end;
  }
  // customize end

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.pose = global_plan_pose;
      nav_2d_utils::transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose.pose;
    };

  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    pub_->publishGlobalPlan(global_plan_);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}
}  // namespace cabot_navigation2
