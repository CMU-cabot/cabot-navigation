#include "cabot_dnn_controller/dagger_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cabot_dnn_controller
{

void DaggerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock node in DaggerController::configure");
  }
  name_ = std::move(name);
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  node_->declare_parameter(name_ + ".oracle.plugin", rclcpp::ParameterValue("dwb_core::DWBLocalPlanner"));
  node_->get_parameter(name_ + ".oracle.plugin", oracle_plugin_type_);
  node_->declare_parameter(name_ + ".policy.plugin", rclcpp::ParameterValue("cabot_dnn_controller::DnnController"));
  node_->get_parameter(name_ + ".policy.plugin", policy_plugin_type_);
  node_->declare_parameter(name_ + ".alpha", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".alpha", alpha_);
  node_->declare_parameter(name_ + ".policy_diff_threshold", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".policy_diff_threshold", policy_diff_threshold_);
  node_->declare_parameter(name_ + ".policy_recovery_sec", rclcpp::ParameterValue(5.0));
  node_->get_parameter(name_ + ".policy_recovery_sec", policy_recovery_sec_);
  node_->declare_parameter(name_ + ".use_fixed_seed", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".use_fixed_seed", use_fixed_seed_);
  node_->declare_parameter(name_ + ".rng_seed", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + ".rng_seed", rng_seed_);

  if (alpha_ < 0.0 || alpha_ > 1.0) {
    throw std::runtime_error("DaggerController: alpha must be in [0.0, 1.0]");
  }
  if (policy_diff_threshold_ < 0.0) {
    throw std::runtime_error("DaggerController: policy_diff_threshold must be >= 0.0");
  }
  if (policy_recovery_sec_ < 0.0) {
    throw std::runtime_error("DaggerController: policy_recovery_sec must be >= 0.0");
  }

  if (use_fixed_seed_) {
    rng_.seed(rng_seed_);
  } else {
    std::random_device rd;
    rng_.seed(rd());
  }
  bernoulli_dist_ = std::bernoulli_distribution(alpha_);
  policy_recovery_duration_ = rclcpp::Duration::from_seconds(policy_recovery_sec_);
  policy_disabled_until_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  RCLCPP_INFO(node_->get_logger(), "[%s] oracle=%s policy=%s alpha=%.3f diff_th=%.3f recovery=%.2fs", 
    name_.c_str(), oracle_plugin_type_.c_str(), policy_plugin_type_.c_str(), alpha_, policy_diff_threshold_, policy_recovery_sec_);

  cmd_oracle_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_oracle", rclcpp::SystemDefaultsQoS());

  try {
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core",
      "nav2_core::Controller",
      "plugin",
      std::vector<std::string>{}
    );
    oracle_ = loader_->createSharedInstance(oracle_plugin_type_);
    policy_ = loader_->createSharedInstance(policy_plugin_type_);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("DaggerController: failed to create internal controller: ") + e.what());
  }

  oracle_->configure(parent, name_ + ".oracle", tf_, costmap_ros_);
  policy_->configure(parent, name_ + ".policy", tf_, costmap_ros_);
}

void DaggerController::cleanup()
{
  if (oracle_) oracle_->cleanup();
  if (policy_) policy_->cleanup();

  oracle_.reset();
  policy_.reset();
}

void DaggerController::activate()
{
  if (oracle_) oracle_->activate();
  if (policy_) policy_->activate();
}

void DaggerController::deactivate()
{
  if (oracle_) oracle_->deactivate();
  if (policy_) policy_->deactivate();
}

void DaggerController::setPlan(const nav_msgs::msg::Path & path)
{
  if (oracle_) oracle_->setPlan(path);
  if (policy_) policy_->setPlan(path);
}

void DaggerController::setSpeedLimit(const double &, const bool &)
{
  // TODO: implement speed limiting for DaggerController.
}

geometry_msgs::msg::TwistStamped DaggerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto cmd_oracle = oracle_->computeVelocityCommands(pose, velocity, goal_checker);
  auto cmd_policy = policy_->computeVelocityCommands(pose, velocity, goal_checker);

  const auto now = clock_->now();
  cmd_oracle.header.stamp = now;
  cmd_policy.header.stamp = now;

  cmd_oracle_pub_->publish(cmd_oracle);

  const bool policy_disabled = now < policy_disabled_until_;
  const bool choose_oracle = policy_disabled || bernoulli_dist_(rng_);
  auto out = choose_oracle ? cmd_oracle : cmd_policy;
  RCLCPP_INFO(logger_, "[%s] time=%.3f: choose %s command, policy_disabled=%s",
    name_.c_str(), now.seconds(), choose_oracle ? "oracle" : "policy", policy_disabled ? "true" : "false");

  // If the selected policy output differs from the oracle by more than a certain threshold, disable selecting the policy for a while
  if (!choose_oracle) {
    const auto & oracle_twist = cmd_oracle.twist;
    const auto & policy_twist = cmd_policy.twist;
    const double dx = oracle_twist.linear.x - policy_twist.linear.x;
    const double dy = oracle_twist.linear.y - policy_twist.linear.y;
    const double dz = oracle_twist.linear.z - policy_twist.linear.z;
    const double dax = oracle_twist.angular.x - policy_twist.angular.x;
    const double day = oracle_twist.angular.y - policy_twist.angular.y;
    const double daz = oracle_twist.angular.z - policy_twist.angular.z;
    const double v_diff = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double w_diff = std::sqrt(dax * dax + day * day + daz * daz);
    if ((v_diff > policy_diff_threshold_) || (w_diff > policy_diff_threshold_)) {
      policy_disabled_until_ = now + policy_recovery_duration_;
    }
    RCLCPP_INFO(logger_, "[%s] time=%.3f: v_diff=%.3f, w_diff=%.3f", name_.c_str(), now.seconds(), v_diff, w_diff);
  }

  return out;
}

}  // namespace cabot_dnn_controller

PLUGINLIB_EXPORT_CLASS(cabot_dnn_controller::DaggerController, nav2_core::Controller)
