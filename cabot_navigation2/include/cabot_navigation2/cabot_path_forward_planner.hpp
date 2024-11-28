#ifndef CABOT_NAVIGATION2__CABOT_PATH_FORWARD_PLANNER_HPP_
#define CABOT_NAVIGATION2__CABOT_PATH_FORWARD_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace cabot_navigation2
{

class CabotPathForwardPlanner : public nav2_core::GlobalPlanner
{
public:
  CabotPathForwardPlanner() = default;
  ~CabotPathForwardPlanner() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  std::recursive_mutex mutex_;

  std::string path_topic_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr last_path_;
  void pathCallBack(const nav_msgs::msg::Path::SharedPtr path);
};

}  // namespace cabot_navigation2

#endif  // CABOT_NAVIGATION2__CABOT_PATH_FORWARD_PLANNER_HPP_