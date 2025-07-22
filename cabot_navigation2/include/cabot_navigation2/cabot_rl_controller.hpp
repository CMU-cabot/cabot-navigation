#ifndef CABOT_NAVIGATION2__CABOT_RL_CONTROLLER_HPP_
#define CABOT_NAVIGATION2__CABOT_RL_CONTROLLER_HPP_

#include <memory>
#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// #include "lidar_process_msgs/srv/rl_action.hpp"
#include "lidar_process_msgs/msg/robot_message.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cabot_navigation2/util.hpp>

using nav2_util::declare_parameter_if_not_declared;

namespace cabot_navigation2
{

class CaBotRLController : public nav2_core::Controller     
{
public:
  CaBotRLController();
  ~CaBotRLController() override;

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
  rclcpp::Logger logger_ = rclcpp::get_logger("CaBotRLController");
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string rl_cmd_topic_;
  std::string rl_info_topic_;
  nav2_costmap_2d::Costmap2DROS *costmap_ros_;  // Pointer to the costmap
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan;

  int last_visited_index_; // Keep track of the last visited point in the global plan
  geometry_msgs::msg::PoseStamped curr_local_goal_;
  geometry_msgs::msg::Twist current_command;
  lidar_process_msgs::msg::RobotMessage robot_info;

  double lookahead_distance_;
  double focus_goal_dist_;

  std::string loc_goal_vis_topic_;
  rclcpp::TimerBase::SharedPtr loc_goal_vis_timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_goal_visualization_pub_;
  void localGoalVisualizationCallback();

  // rclcpp::Client<lidar_process_msgs::srv::RlAction>::SharedPtr rl_client;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rl_cmd_sub_;
  void rlCommandCallback(const geometry_msgs::msg::Twist::SharedPtr rl_cmd);

  rclcpp::TimerBase::SharedPtr rl_info_pub_timer_;
  rclcpp::Publisher<lidar_process_msgs::msg::RobotMessage>::SharedPtr rl_info_pub_;
  void rlInfoCallback();

  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & global_plan);

  bool hasReachedLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);

  double pointDist(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);
};

} // namespace cabot_navigation2

#endif // CABOT_NAVIGATION2__CABOT_RL_CONTROLLER_HPP_
