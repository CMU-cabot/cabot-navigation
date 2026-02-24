#ifndef CABOT_NAVIGATION2__SOCIAL_CRITIC_HPP_
#define CABOT_NAVIGATION2__SOCIAL_CRITIC_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "social_nav_msgs/msg/social_nav_msg.hpp"

namespace cabot_navigation2
{

class SocialCritic : public dwb_core::TrajectoryCritic
{
public:
  SocialCritic() = default;

  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void reset() override;

private:
  void socialCallback(const social_nav_msgs::msg::SocialNavMsg::SharedPtr msg);

  rclcpp::Subscription<social_nav_msgs::msg::SocialNavMsg>::SharedPtr social_sub_;
  
  std::mutex social_mutex_;
  int head_dir_ = -1;
  int speed_ = -1;
  std::string social_nav_topic_;

  // Parameters
  double turn_cost_weight_;
  double speed_cost_weight_;
  double stop_cost_weight_;
  double turn_left_theta_;
  double turn_right_theta_;
  double straight_theta_;
  double speed_0_;
  double speed_1_;
  double speed_2_;
};

}  // namespace cabot_navigation2

#endif  // CABOT_NAVIGATION2__SOCIAL_CRITIC_HPP_
