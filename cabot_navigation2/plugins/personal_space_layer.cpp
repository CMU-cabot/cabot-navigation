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
#include "cabot_navigation2/personal_space_layer.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cabot_navigation2
{

/**
 * Calculate asymmetric Gaussian value at a single point (x, y)
 * 
 * @param x X coordinate of the point
 * @param y Y coordinate of the point
 * @param xc X coordinate of the center
 * @param yc Y coordinate of the center
 * @param theta Orientation angle (radians)
 * @param sigma_h Sigma for heading direction (front/back)
 * @param sigma_s Sigma for side direction (perpendicular)
 * @param sigma_r Sigma for rear direction (behind)
 * @return Gaussian value at point (x, y)
 */
inline float asymmetric_gaussian(
    float x,
    float y,
    float xc,
    float yc,
    float theta,
    float sigma_h,
    float sigma_s,
    float sigma_r
) {
    // Calculate offset from center
    float dx = x - xc;
    float dy = y - yc;
    
    // Calculate angle from center to point, relative to theta
    // alpha represents the angle in the person's local coordinate frame
    float alpha = std::atan2(dy, dx) - theta + M_PI / 2.0f;
    
    // Normalize angle to [-pi, pi]
    alpha = std::fmod(alpha + M_PI, 2.0f * M_PI) - M_PI;
    
    // Select sigma based on whether point is in front (alpha > 0) or behind (alpha <= 0)
    float sigma = (alpha <= 0.0f) ? sigma_r : sigma_h;
    
    // Prevent division by zero
    float sigma_sq = std::max(sigma * sigma, 1e-6f);
    float sigma_s_sq = std::max(sigma_s * sigma_s, 1e-6f);
    
    // Precompute trigonometric values
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);
    float cos_t_sq = cos_t * cos_t;
    float sin_t_sq = sin_t * sin_t;
    float two_theta_sin = std::sin(2.0f * theta);
    
    // Calculate rotation matrix coefficients for the quadratic form
    float a = cos_t_sq / (2.0f * sigma_sq) + sin_t_sq / (2.0f * sigma_s_sq);
    float b = two_theta_sin / (4.0f * sigma_sq) - two_theta_sin / (4.0f * sigma_s_sq);
    float c = sin_t_sq / (2.0f * sigma_sq) + cos_t_sq / (2.0f * sigma_s_sq);
    
    // Calculate Mahalanobis-like distance
    float exponent = -(a * dx * dx + 2.0f * b * dx * dy + c * dy * dy);
    
    return std::exp(exponent);
}

PersonalSpaceLayer::PersonalSpaceLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

PersonalSpaceLayer::~PersonalSpaceLayer()
{
}

void PersonalSpaceLayer::activate()
{
  nav2_costmap_2d::Layer::activate();
}

void PersonalSpaceLayer::deactivate()
{
  nav2_costmap_2d::Layer::deactivate();
}

void PersonalSpaceLayer::reset()
{
  nav2_costmap_2d::Layer::reset();
}

bool PersonalSpaceLayer::isClearable()
{
  return true;
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
PersonalSpaceLayer::onInitialize()
{
  auto node = node_.lock();
  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("personal_space_topic", rclcpp::ParameterValue("/personal_space"));
  node->get_parameter(name_ + "." + "personal_space_topic", personal_space_topic_);

  declareParameter("update_width", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_width", update_width_);

  declareParameter("update_height", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_height", update_height_);

  declareParameter("min_cost", rclcpp::ParameterValue(25.0));
  node->get_parameter(name_ + "." + "min_cost", min_cost_);

  declareParameter("max_cost", rclcpp::ParameterValue(50.0));
  node->get_parameter(name_ + "." + "max_cost", max_cost_);

  declareParameter("cost_threshold", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "cost_threshold", cost_threshold_);

  declareParameter("timeout_duration", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "timeout_duration", timeout_duration_);

  // Time (seconds) without an update after which a tracked person is considered to have
  // lost tracking and is moved to "ghost" state.  Should be slightly larger than the
  // expected inter-message interval of the personal-space publisher.
  declareParameter("tracking_lost_threshold", rclcpp::ParameterValue(0.3));
  node->get_parameter(name_ + "." + "tracking_lost_threshold", tracking_lost_threshold_);

  personal_space_sub_ = node->create_subscription<rank_nav_msgs::msg::PeoplePersonalSpace>(
    personal_space_topic_, sensor_qos,
    std::bind(&PersonalSpaceLayer::personalSpaceCallback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
  matchSize();

  RCLCPP_INFO(node->get_logger(), "PersonalSpaceLayer initialized. Subscribing to: %s", personal_space_topic_.c_str());
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
PersonalSpaceLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;
  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
  if (need_recalculation_) {
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    *min_x = robot_x - update_width_ / 2.0;
    *min_y = robot_y - update_height_ / 2.0;
    *max_x = robot_x + update_width_ / 2.0;
    *max_y = robot_y + update_height_ / 2.0;
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
PersonalSpaceLayer::onFootprintChanged()
{
  if (!enabled_) {
    return;
  }
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "PersonalSpaceLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
PersonalSpaceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  auto node = node_.lock();
  std::chrono::milliseconds s = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

  if (!enabled_) {
    RCLCPP_DEBUG(node->get_logger(), "[PersonalSpaceLayer] Layer is disabled");
    return;
  }

  // Detect newly-lost tracking, then remove fully-expired entries.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateGhostState(node->now());
    removeExpiredPersonalSpaces(node->now());

    if (personal_spaces_.empty()) {
      RCLCPP_DEBUG(node->get_logger(), "[PersonalSpaceLayer] No personal space data available");
      return;
    }
  }

  RCLCPP_DEBUG(node->get_logger(), "[PersonalSpaceLayer] Starting updateCosts for %zu people", personal_spaces_.size());

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  int window_size_x = max_i - min_i;
  int window_size_y = max_j - min_j;

  if (window_size_x <= 0 || window_size_y <= 0) {
    return;
  }

  // Local buffer to accumulate Gaussian values within the update window
  std::vector<float> accumulation_buffer(window_size_x * window_size_y, 0.0f);

  // Copy personal spaces to avoid holding lock during computation
  std::map<int32_t, PersonalSpaceData> local_spaces;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_spaces = personal_spaces_;
  }
  
  RCLCPP_INFO(node->get_logger(), "[PersonalSpaceLayer] Processing %zu people", local_spaces.size());

  // Avoid log(0) or undefined behavior if cost_threshold_ is <= 0 or >= 1
  // If threshold is >= 1.0, nothing will be drawn anyway (since max gaussian is 1.0)
  float effective_threshold = std::max((float)cost_threshold_, 0.001f);
  if (effective_threshold >= 1.0f - 1e-6) {
    return; // No need to process
  }

  // Precompute -2 * ln(threshold)
  float log_factor = -2.0f * std::log(effective_threshold);

    for (const auto & person_pair : local_spaces) {
    const auto & msg = person_pair.second.msg;
    float xc = msg->x;
    float yc = msg->y;
    float theta = msg->yaw;
    
    // If theta is out of range (< -M_PI or > M_PI), calculate direction from person to robot
    if (std::abs(theta) > 6.0f) { // 6.0 is well outside [-PI, PI] but catches -10.0
      theta = std::atan2(robot_y_ - yc, robot_x_ - xc);
    }
    
    // Precompute trigonometric values for this person
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    float sigma_h = msg->param1;
    float sigma_s = msg->param2;
    float sigma_r = msg->param3;
    
    // Validate sigmas to prevent division by zero
    float sigma_h_sq = std::max(sigma_h * sigma_h, 1e-6f);
    float sigma_s_sq = std::max(sigma_s * sigma_s, 1e-6f);
    float sigma_r_sq = std::max(sigma_r * sigma_r, 1e-6f);

    // Precompute coefficients for the exponent calculation
    // Exponent = - ( lx^2 / (2*sigma_long^2) + ly^2 / (2*sigma_s^2) )
    float coeff_h = 1.0f / (2.0f * sigma_h_sq);
    float coeff_s = 1.0f / (2.0f * sigma_s_sq);
    float coeff_r = 1.0f / (2.0f * sigma_r_sq);

    // Calculate cutoff radius
    // We use the maximum sigma to determine a safe bounding box
    float max_sigma = std::max({sigma_h, sigma_s, sigma_r});
    float cutoff_radius = std::sqrt(log_factor) * max_sigma;

    // Calculate bounding box in map coordinates
    int p_min_i, p_min_j, p_max_i, p_max_j;
    
    // Using worldToMapNoBounds to get indices even if outside the map (will be clamped later)
    master_grid.worldToMapNoBounds(xc - cutoff_radius, yc - cutoff_radius, p_min_i, p_min_j);
    master_grid.worldToMapNoBounds(xc + cutoff_radius, yc + cutoff_radius, p_max_i, p_max_j);

    // Intersection of person's bounding box and the update window
    int i_start = std::max(min_i, p_min_i);
    int j_start = std::max(min_j, p_min_j);
    int i_end = std::min(max_i, p_max_i + 1); // +1 because loop is exclusive
    int j_end = std::min(max_j, p_max_j + 1);

    if (i_start >= i_end || j_start >= j_end) {
      continue; // No overlap
    }

    // Iterate only over the relevant area for this person
    for (int j = j_start; j < j_end; j++) {
      for (int i = i_start; i < i_end; i++) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        
        float dx = wx - xc;
        float dy = wy - yc;

        // Transform to local coordinates (lx: forward, ly: left)
        // lx = dx * cos(theta) + dy * sin(theta)
        // ly = -dx * sin(theta) + dy * cos(theta)
        float lx = dx * cos_t + dy * sin_t;
        float ly = -dx * sin_t + dy * cos_t;

        // Determine which sigma to use for the longitudinal component
        // lx > 0 corresponds to the front (alpha > 0 in original logic)
        float sigma_coeff = (lx > 0.0f) ? coeff_h : coeff_r;

        // Calculate Gaussian value
        // val = exp( - (lx^2 / 2sigma^2 + ly^2 / 2sigma_s^2) )
        float exponent = -(lx * lx * sigma_coeff + ly * ly * coeff_s);
        
        // Add to buffer
        // Map (i, j) -> Buffer (i - min_i, j - min_j)
        int buf_idx = (j - min_j) * window_size_x + (i - min_i);
        accumulation_buffer[buf_idx] += std::exp(exponent);
      }
    }
  }

  // Apply accumulated costs to the master grid
  int cells_updated = 0;
  int cells_with_cost = 0;
  unsigned char max_cost_applied = 0;

  for (int j = 0; j < window_size_y; j++) {
    for (int i = 0; i < window_size_x; i++) {
        float total_gaussian = accumulation_buffer[j * window_size_x + i];

        if (total_gaussian < cost_threshold_) {
          continue;
        }

        // Apply cost linearly between min_cost_ and max_cost_
        float normalized_val = std::min(total_gaussian, 1.0f);
        float denom = 1.0f - cost_threshold_;
        float f_cost;
        if (denom < 1e-6) {
            f_cost = max_cost_;
        } else {
            float ratio = (normalized_val - cost_threshold_) / denom;
            f_cost = min_cost_ + ratio * (max_cost_ - min_cost_);
        }
        unsigned char cost = static_cast<unsigned char>(f_cost);

        // Update costmap with overwrite (preserving lethal obstacles)
        // Buffer local (i, j) corresponds to global (min_i + i, min_j + j)
        unsigned int index = master_grid.getIndex(min_i + i, min_j + j);
        
        if (master_array[index] != LETHAL_OBSTACLE) {
          master_array[index] = cost;
          cells_updated++;
        }
        if (cost > 0) {
          cells_with_cost++;
          max_cost_applied = std::max(max_cost_applied, cost);
        }
    }
  }

  std::chrono::milliseconds e = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  RCLCPP_INFO(node->get_logger(), "[PersonalSpaceLayer] Update complete: %d cells updated, %d cells with cost, max_cost: %d, time: %ld ms",
    cells_updated, cells_with_cost, (int)max_cost_applied, (e - s).count());
}

void PersonalSpaceLayer::personalSpaceCallback(const rank_nav_msgs::msg::PeoplePersonalSpace::SharedPtr msg)
{
  auto node = node_.lock();
  std::lock_guard<std::mutex> lock(mutex_);

  // Detect tracking resumption: if this person was previously in ghost state, log the recovery.
  auto it = personal_spaces_.find(msg->person_id);
  const bool was_ghost = (it != personal_spaces_.end() && it->second.is_ghost);
  if (was_ghost) {
    RCLCPP_INFO(
      node->get_logger(),
      "[PersonalSpaceLayer] Tracking RESUMED for person_id: %d (was in ghost/buffer state)",
      msg->person_id);
  }

  // Store or update personal space data for this person_id.
  // Resetting is_ghost to false marks the entry as actively tracked again.
  PersonalSpaceData data;
  data.msg = msg;
  data.last_update_time = node->now();
  data.is_ghost = false;

  personal_spaces_[msg->person_id] = data;

  RCLCPP_INFO(
    node->get_logger(),
    "[PersonalSpaceLayer] Received personal space - person_id: %d, pos: (%.2f, %.2f),"
    " yaw: %.2f, params: (%.2f, %.2f, %.2f) [Total: %zu people]",
    msg->person_id, msg->x, msg->y, msg->yaw,
    msg->param1, msg->param2, msg->param3, personal_spaces_.size());
}

void PersonalSpaceLayer::updateGhostState(const rclcpp::Time & current_time)
{
  // Called under mutex_.
  // Detect persons whose tracking has been lost (no message for > tracking_lost_threshold_)
  // and mark them as ghosts so their last known personal space continues to be applied
  // to the costmap during the buffer period (up to timeout_duration_ from last message).
  auto node = node_.lock();
  for (auto & [id, data] : personal_spaces_) {
    if (!data.is_ghost) {
      double elapsed = (current_time - data.last_update_time).seconds();
      if (elapsed > tracking_lost_threshold_) {
        data.is_ghost = true;
        double remaining = timeout_duration_ - elapsed;
        RCLCPP_INFO(
          node->get_logger(),
          "[PersonalSpaceLayer] Tracking LOST for person_id: %d"
          " (%.2f s since last update, buffering last known position for up to %.2f s more)",
          id, elapsed, std::max(remaining, 0.0));
      }
    }
  }
}

void PersonalSpaceLayer::removeExpiredPersonalSpaces(const rclcpp::Time & current_time)
{
  // Called under mutex_.
  // A ghost entry is kept for timeout_duration_ seconds measured from the last received
  // message (last_update_time).  After that the entry is fully removed.
  auto node = node_.lock();
  auto it = personal_spaces_.begin();
  while (it != personal_spaces_.end()) {
    double elapsed = (current_time - it->second.last_update_time).seconds();
    if (elapsed > timeout_duration_) {
      RCLCPP_INFO(
        node->get_logger(),
        "[PersonalSpaceLayer] Buffer expired for person_id: %d"
        " (was %s, %.2f s since last message)",
        it->first, it->second.is_ghost ? "ghost" : "active", elapsed);
      it = personal_spaces_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace cabot_navigation2

// This is the macro allowing a cabot_navigation2::PersonalSpaceLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::PersonalSpaceLayer, nav2_costmap_2d::Layer)