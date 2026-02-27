#ifndef CABOT_DNN_CONTROLLER__DAGGER_CONTROLLER_HPP_
#define CABOT_DNN_CONTROLLER__DAGGER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace cabot_dnn_controller
{

class DaggerController : public nav2_core::Controller
{
public:
  DaggerController() = default;
  ~DaggerController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("dagger_controller")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string oracle_plugin_type_;
  std::string policy_plugin_type_;
  double alpha_;
  double policy_diff_threshold_;
  double policy_recovery_sec_;
  int rng_seed_;
  bool use_fixed_seed_;

  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  std::shared_ptr<nav2_core::Controller> oracle_;
  std::shared_ptr<nav2_core::Controller> policy_;
  std::mt19937 rng_;
  std::bernoulli_distribution bernoulli_dist_;
  rclcpp::Duration policy_diff_hold_duration_{0, 0};
  rclcpp::Duration policy_recovery_duration_{0, 0};
  rclcpp::Time policy_disabled_until_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_oracle_pub_;
};

}  // namespace cabot_dnn_controller

#endif  // CABOT_DNN_CONTROLLER__DAGGER_CONTROLLER_HPP_
