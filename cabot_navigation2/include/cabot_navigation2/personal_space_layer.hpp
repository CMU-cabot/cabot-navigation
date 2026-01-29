/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Carnegie Mellon University and Miraikan
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
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef CABOT_NAVIGATION2__PERSONAL_SPACE_LAYER_HPP_
#define CABOT_NAVIGATION2__PERSONAL_SPACE_LAYER_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "rank_nav_msgs/msg/people_personal_space.hpp"

namespace cabot_navigation2
{

class PersonalSpaceLayer : public nav2_costmap_2d::Layer
{
public:
  PersonalSpaceLayer();
  ~PersonalSpaceLayer() override;

  void activate() override;
  void deactivate() override;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  void onFootprintChanged() override;

  bool isClearable() override;

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double robot_x_, robot_y_, robot_yaw_;
  double update_width_, update_height_;
  double min_cost_;
  double max_cost_;
  double cost_threshold_;

  // Structure to store personal space data with timestamp
  struct PersonalSpaceData {
    rank_nav_msgs::msg::PeoplePersonalSpace::SharedPtr msg;
    rclcpp::Time last_update_time;
  };

  void personalSpaceCallback(const rank_nav_msgs::msg::PeoplePersonalSpace::SharedPtr msg);
  void removeExpiredPersonalSpaces(const rclcpp::Time & current_time);

  // Indicates that the entire costmap should be recalculated next time.
  bool need_recalculation_;

  std::string personal_space_topic_;
  rclcpp::Subscription<rank_nav_msgs::msg::PeoplePersonalSpace>::SharedPtr personal_space_sub_;
  
  // Map of person_id to personal space data with timestamp
  std::map<int32_t, PersonalSpaceData> personal_spaces_;
  double timeout_duration_;  // Timeout in seconds (default: 1.0)

  std::mutex mutex_;
};

}  // namespace cabot_navigation2

#endif  // CABOT_NAVIGATION2__PERSONAL_SPACE_LAYER_HPP_
