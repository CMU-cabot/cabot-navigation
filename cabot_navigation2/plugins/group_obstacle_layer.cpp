#include "cabot_navigation2/group_obstacle_layer.hpp"

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

GroupObstacleLayer::GroupObstacleLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

GroupObstacleLayer::~GroupObstacleLayer()
{
}

void GroupObstacleLayer::activate()
{
  nav2_costmap_2d::ObstacleLayer::activate();
}

void GroupObstacleLayer::deactivate()
{
  nav2_costmap_2d::ObstacleLayer::deactivate();
}

void GroupObstacleLayer::reset()
{
  nav2_costmap_2d::ObstacleLayer::reset();
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void GroupObstacleLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("group_topic", rclcpp::ParameterValue("/group_predictions"));
  node->get_parameter(name_ + "." + "group_topic", group_topic_);

  declareParameter("update_width", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_width", update_width);

  declareParameter("update_height", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "update_height", update_height);

  group_sub_ = node->create_subscription<lidar_process_msgs::msg::GroupTimeArray>(
    group_topic_, 10,
    std::bind(&GroupObstacleLayer::groupCallBack, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void GroupObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  auto node = node_.lock();

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
    *min_x = robot_x - update_width / 2;
    *min_y = robot_y - update_height / 2;
    *max_x = robot_x + update_width / 2;
    *max_y = robot_y + update_height / 2;
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void GroupObstacleLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GroupObstacleLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void GroupObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, 
  int min_i, int min_j, int max_i, int max_j)
{

  auto node = node_.lock();

  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Simply computing one-by-one cost per each cell
  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = LETHAL_OBSTACLE;
      master_array[index] = 20.0;
    }
  }
}

void GroupObstacleLayer::groupCallBack(const lidar_process_msgs::msg::GroupTimeArray::SharedPtr group)
{
  auto node = node_.lock();
  last_group_ = group;
  RCLCPP_INFO(node->get_logger(), "======================Group Received======================");
}

}  // namespace cabot_navigation2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::GroupObstacleLayer, nav2_costmap_2d::Layer)