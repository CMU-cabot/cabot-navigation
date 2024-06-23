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

// CaBotDWBLocalPlanner extends transformGlobalPlan of DWBLocalPlanner
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

  // Find the first pose in the global plan that's further than prune distance
  // from the robot using integrated distance
  auto prune_point = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than transform_start_threshold
  // from the robot.
  auto transformation_begin = std::find_if(
    begin(global_plan_.poses), prune_point,
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose.pose, global_plan_pose) < transform_start_threshold;
    });

  // Find the first pose in the end of the plan that's further than transform_end_threshold
  // from the robot using integrated distance
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose.pose) > transform_end_threshold;
    });

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
