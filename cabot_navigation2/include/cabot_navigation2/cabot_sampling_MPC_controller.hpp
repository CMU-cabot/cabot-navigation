#ifndef CABOT_NAVIGATION2__CABOT_SAMPLING_MPC_CONTROLLER_HPP_
#define CABOT_NAVIGATION2__CABOT_SAMPLING_MPC_CONTROLLER_HPP_

#include <memory>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "lidar_process_msgs/msg/group_time_array.hpp"
#include "lidar_process_msgs/msg/group_array.hpp"
#include "lidar_process_msgs/msg/group.hpp"

namespace cabot_navigation2
{

class CaBotSamplingMPCController : public nav2_core::Controller
{
public:
  CaBotSamplingMPCController() = default;
  ~CaBotSamplingMPCController() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav_msgs::msg::Path & global_plan) override;

private:
  rclcpp::Logger logger_;
  std::shared_ptr<rclcpp::Node> node_;
  nav2_costmap_2d::Costmap2DROS *costmap_ros_;  // Pointer to the costmap

  // Storage for the group trajectories
  std::vector<lidar_process_msgs::msg::GroupArray> group_trajectories_;

  // MPC-related variables
  double prediction_horizon_;
  double sampling_rate_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double lookahead_distance_;
  double discount_factor_;  // Discount factor for future time steps

  int last_visited_index_; // Keep track of the last visited point in the global plan

  std::string group_topic_;
  rclcpp::Subscription<lidar_process_msgs::msg::GroupTimeArray>::SharedPtr group_trajectory_sub_;  // group prediction subscriber
  void groupPredictionCallback(const lidar_process_msgs::msg::GroupTimeArray::SharedPtr group_series);

  geometry_msgs::msg::Twist computeMPCControl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav_msgs::msg::Path & global_plan);

  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & global_plan);

  bool hasReachedLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);

  double calculateCost(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const std::vector<geometry_msgs::msg::PoseStamped> & sampled_trajectory,
    const nav_msgs::msg::Path & global_plan,
    const geometry_msgs::msg::PoseStamped & local_goal);

  double getCostFromCostmap(
    const geometry_msgs::msg::Pose & pose);

  double calculateaGroupTrajectoryCost(
    const std::vector<geometry_msgs::msg::PoseStamped> & sampled_trajectory);
};

} // namespace cabot_navigation2

#endif // CABOT_NAVIGATION2__CABOT_SAMPLING_MPC_CONTROLLER_HPP_
