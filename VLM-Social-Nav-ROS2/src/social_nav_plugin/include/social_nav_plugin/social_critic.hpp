#ifndef SOCIAL_NAV_PLUGIN__SOCIAL_CRITIC_HPP_
#define SOCIAL_NAV_PLUGIN__SOCIAL_CRITIC_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "vlm_social_nav/msg/social_nav_msg.hpp"

namespace social_nav_plugin
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
  void socialCallback(const vlm_social_nav::msg::SocialNavMsg::SharedPtr msg);

  rclcpp::Subscription<vlm_social_nav::msg::SocialNavMsg>::SharedPtr social_sub_;
  
  std::mutex social_mutex_;
  int head_dir_ = -1;
  int speed_ = -1;
};

}  // namespace social_nav_plugin

#endif  // SOCIAL_NAV_PLUGIN__SOCIAL_CRITIC_HPP_
