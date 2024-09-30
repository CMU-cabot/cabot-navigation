#ifndef CABOT_NAVIGATION2__CABOT_SAMPLING_MPC_CONTROLLER_HPP_
#define CABOT_NAVIGATION2__CABOT_SAMPLING_MPC_CONTROLLER_HPP_

#include <memory>
#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "lidar_process_msgs/msg/group_time_array.hpp"
#include "lidar_process_msgs/msg/group_array.hpp"
#include "lidar_process_msgs/msg/group.hpp"

#include <cabot_navigation2/util.hpp>

using nav2_util::declare_parameter_if_not_declared;

namespace cabot_navigation2
{

struct Trajectory
{
  geometry_msgs::msg::Twist control;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory;
  Trajectory(
    geometry_msgs::msg::Twist control,
    std::vector<geometry_msgs::msg::PoseStamped> trajectory
  );
};

class CaBotSamplingMPCController : public nav2_core::Controller     
{
public:
  CaBotSamplingMPCController();
  ~CaBotSamplingMPCController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  // bool cancel()
  // {
  //   return true;
  // }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // void reset() {}

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("CaBotSamplingMPCController");
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  nav2_costmap_2d::Costmap2DROS *costmap_ros_;  // Pointer to the costmap
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan;

  // Storage for the group trajectories
  std::vector<lidar_process_msgs::msg::GroupArray> group_trajectories_;

  // MPC-related variables
  double prediction_horizon_;
  double sampling_rate_;
  double max_linear_velocity_;
  double linear_sample_size_;
  double linear_sample_resolution_;
  double max_angular_velocity_;
  double angular_sample_size_;
  double angular_sample_resolution_;
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
  
  std::vector<Trajectory> generateTrajectoriesSimple(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & velocity);

  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & global_plan);

  bool hasReachedLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);

  double calculateCost(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const Trajectory sampled_trajectory,
    const nav_msgs::msg::Path & global_plan,
    const geometry_msgs::msg::PoseStamped & local_goal);

  double getCostFromCostmap(
    const geometry_msgs::msg::Pose & pose);

  double calculateGroupTrajectoryCost(
    const std::vector<geometry_msgs::msg::PoseStamped> & sampled_trajectory);

  double pointDist(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

  double pointDist(
    const Safety::Point p1,
    const Safety::Point p2);
};

} // namespace cabot_navigation2

#endif // CABOT_NAVIGATION2__CABOT_SAMPLING_MPC_CONTROLLER_HPP_
