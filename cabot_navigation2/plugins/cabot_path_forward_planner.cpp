#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "cabot_navigation2/cabot_path_forward_planner.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace cabot_navigation2
{

void CabotPathForwardPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  declare_parameter_if_not_declared(node_, name_ + ".path_topic", rclcpp::ParameterValue("/path"));
  node_->get_parameter(name_ + "." + "path_topic", path_topic_);

  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10,
    std::bind(&CabotPathForwardPlanner::pathCallBack, this, std::placeholders::_1));
}

void CabotPathForwardPlanner::cleanup()
{
  //RCLCPP_INFO(
  //  node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
  //  name_.c_str());
}

void CabotPathForwardPlanner::activate()
{
  //RCLCPP_INFO(
  //  node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
  //  name_.c_str());
}

void CabotPathForwardPlanner::deactivate()
{
  //RCLCPP_INFO(
  //  node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
  //  name_.c_str());
}

nav_msgs::msg::Path CabotPathForwardPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto global_path = nav_msgs::msg::Path();

  if (!last_path_) {
    return global_path;
  }

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path = *last_path_;
  /*
  for (auto it = global_path.poses.begin(); it != global_path.poses.end(); it++) {
    auto pose = it;
    RCLCPP_INFO(node_->get_logger(), "Path point: %.2f, %.2f", pose->pose.position.x, pose->pose.position.y);
  }
  */

  return global_path;
}

void CabotPathForwardPlanner::pathCallBack(const nav_msgs::msg::Path::SharedPtr path)
{
  // Group sequences: First time, then groups
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  last_path_ = path;
  //RCLCPP_INFO(node_->get_logger(), "============Path Received==============");
}

}  // namespace cabot_navigation2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CabotPathForwardPlanner, nav2_core::GlobalPlanner)