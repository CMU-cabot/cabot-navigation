#include "cabot_navigation2/social_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace cabot_navigation2
{

void SocialCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string name = getName();
  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "social_nav_topic", rclcpp::ParameterValue("openai_result"));
  node->get_parameter(name + "." + "social_nav_topic", social_nav_topic_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "turn_cost_weight", rclcpp::ParameterValue(10.0));
  node->get_parameter(name + "." + "turn_cost_weight", turn_cost_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "speed_cost_weight", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + "." + "speed_cost_weight", speed_cost_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "stop_cost_weight", rclcpp::ParameterValue(100.0));
  node->get_parameter(name + "." + "stop_cost_weight", stop_cost_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "turn_left_theta", rclcpp::ParameterValue(0.2));
  node->get_parameter(name + "." + "turn_left_theta", turn_left_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "turn_right_theta", rclcpp::ParameterValue(-0.2));
  node->get_parameter(name + "." + "turn_right_theta", turn_right_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "straight_theta", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + "." + "straight_theta", straight_theta_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "speed_0", rclcpp::ParameterValue(0.3));
  node->get_parameter(name + "." + "speed_0", speed_0_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "speed_1", rclcpp::ParameterValue(0.6));
  node->get_parameter(name + "." + "speed_1", speed_1_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + "." + "speed_2", rclcpp::ParameterValue(0.4));
  node->get_parameter(name + "." + "speed_2", speed_2_);

  // Subscribe to social_nav_topic
  social_sub_ = node->create_subscription<social_nav_msgs::msg::SocialNavMsg>(
    social_nav_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&SocialCritic::socialCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "SocialCritic initialized. Subscribing to %s", social_nav_topic_.c_str());
}

void SocialCritic::socialCallback(const social_nav_msgs::msg::SocialNavMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(social_mutex_);
  try {
    if (!msg->head_dir.empty()) {
      head_dir_ = std::stoi(msg->head_dir);
    } else {
      head_dir_ = -1;
    }

    if (!msg->speed.empty()) {
      speed_ = std::stoi(msg->speed);
    } else {
      speed_ = -1;
    }
    
    RCLCPP_INFO(node_.lock()->get_logger(), "Received Social Cmd: %d, %d", head_dir_, speed_);
  } catch (const std::exception & e) {
    // If parsing fails, reset to -1
    head_dir_ = -1;
    speed_ = -1;
    RCLCPP_WARN(node_.lock()->get_logger(), "Failed to parse social nav msg: %s", e.what());
  }
}

bool SocialCritic::prepare(const geometry_msgs::msg::Pose2D & /*pose*/, const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose2D & /*goal*/,
  const nav_2d_msgs::msg::Path2D & /*global_plan*/)
{
  return true;
}

void SocialCritic::reset()
{
  std::lock_guard<std::mutex> lock(social_mutex_);
  head_dir_ = -1;
  speed_ = -1;
}

double SocialCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  std::lock_guard<std::mutex> lock(social_mutex_);
  
  double cost = 0.0;

  //  no social cost recieved
  if (head_dir_ < 0 || speed_ < 0)
      return 0.0;

  double xv = traj.velocity.x;
  double thetav = traj.velocity.theta;
  
  if (speed_ == 3) {
      // STOP
      return std::abs(xv) * stop_cost_weight_;
  }

  int num_points = traj.poses.size();
  
  for (int i = 0; i < num_points; ++i) {
      double desired_theta = 0.0;
      double desired_speed = 0.0;
      
      // preference on the left
      // In original code: 1 is STRAIGHT, mapped to 0.2 (Turn Left)
      if (head_dir_ == 1) 
          desired_theta = turn_left_theta_;
      // preference on the straight
      // In original code: 0 is LEFT, mapped to 0.0 (Straight)
      else if (head_dir_ == 0) 
          desired_theta = straight_theta_;
      // preference on the right
      // In original code: 2 is RIGHT, mapped to -0.2 (Turn Right)
      else if (head_dir_ == 2) 
          desired_theta = turn_right_theta_;

      cost += turn_cost_weight_ * std::abs(desired_theta - thetav);

      // speed lookup
      if (speed_ == 0) 
          desired_speed = speed_0_;
      else if (speed_ == 1) 
          desired_speed = speed_1_;
      else if (speed_ == 2)
          desired_speed = speed_2_;
      // speed_ == 3 handled above
      
      cost += speed_cost_weight_ * std::abs(desired_speed - xv);
  }

  return cost;
}

}  // namespace cabot_navigation2

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::SocialCritic, dwb_core::TrajectoryCritic)
