#include "cabot_navigation2/cabot_blind_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

namespace
{

double pointDist(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

// wrap to (-pi, pi]
double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle <= -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

}  // namespace

namespace cabot_navigation2
{

CaBotBlindController::CaBotBlindController() {}

CaBotBlindController::~CaBotBlindController() {}

void CaBotBlindController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // costmap_ros is intentionally unused: this controller looks at no perception
  // input. See the class comment for why that is safe here.
  (void)costmap_ros;

  node_ = parent;
  tf_ = tf;
  name_ = name;

  auto node = parent.lock();
  logger_ = node->get_logger();

  declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_lookahead", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + ".max_lookahead", max_lookahead_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_linear_velocity", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".max_linear_velocity", max_linear_velocity_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".max_angular_velocity", max_angular_velocity_);

  declare_parameter_if_not_declared(
    node, name_ + ".angular_gain", rclcpp::ParameterValue(1.5));
  node->get_parameter(name_ + ".angular_gain", angular_gain_);

  declare_parameter_if_not_declared(
    node, name_ + ".rotate_in_place_threshold", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".rotate_in_place_threshold", rotate_in_place_threshold_);

  declare_parameter_if_not_declared(
    node, name_ + ".goal_approach_distance", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".goal_approach_distance", goal_approach_distance_);

  declare_parameter_if_not_declared(
    node, name_ + ".local_goal_vis_topic", rclcpp::ParameterValue(std::string("/local_goal_vis")));
  node->get_parameter(name_ + ".local_goal_vis_topic", local_goal_vis_topic_);

  local_goal_vis_pub_ =
    node->create_publisher<visualization_msgs::msg::Marker>(local_goal_vis_topic_, 10);
  local_goal_vis_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CaBotBlindController::localGoalVisualizationCallback, this));

  RCLCPP_INFO(
    logger_,
    "CaBotBlindController configured: lookahead=%.2f max_lookahead=%.2f "
    "linear=%.2f angular_max=%.2f gain=%.2f rotate_in_place=%.2f goal_approach=%.2f",
    lookahead_distance_, max_lookahead_, max_linear_velocity_, max_angular_velocity_,
    angular_gain_, rotate_in_place_threshold_, goal_approach_distance_);
}

void CaBotBlindController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up blind controller");
  local_goal_vis_pub_.reset();
  local_goal_vis_timer_.reset();
}

void CaBotBlindController::activate()
{
  RCLCPP_INFO(logger_, "Activating blind controller");
}

void CaBotBlindController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating blind controller");
}

void CaBotBlindController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  last_visited_index_ = 0;
  has_local_goal_ = false;
}

void CaBotBlindController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped CaBotBlindController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // velocity and goal_checker are unused: the speed is constant and arrival is
  // decided by the goal checker the controller server runs for us.
  (void)velocity;
  (void)goal_checker;

  auto node = node_.lock();

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000, "blind controller has no plan, holding still");
    return cmd;
  }

  const geometry_msgs::msg::PoseStamped local_goal = getLookaheadPoint(pose, global_plan_);
  curr_local_goal_ = local_goal;
  has_local_goal_ = true;

  const double desired_heading = std::atan2(
    local_goal.pose.position.y - pose.pose.position.y,
    local_goal.pose.position.x - pose.pose.position.x);
  const double heading_error =
    normalizeAngle(desired_heading - tf2::getYaw(pose.pose.orientation));

  double angular = std::clamp(
    angular_gain_ * heading_error, -max_angular_velocity_, max_angular_velocity_);

  double linear = max_linear_velocity_;

  // a plan that doubles back would otherwise be driven into at full speed
  if (std::fabs(heading_error) > rotate_in_place_threshold_) {
    linear = 0.0;
  }

  // ease off near the end of the plan so we do not run past the goal before the
  // goal checker notices
  if (goal_approach_distance_ > 0.0) {
    const double dist_to_end =
      pointDist(pose.pose.position, global_plan_.poses.back().pose.position);
    if (dist_to_end < goal_approach_distance_) {
      linear *= std::max(0.0, dist_to_end / goal_approach_distance_);
    }
  }

  // honour a speed limit set by nav2 (speed filter / behavior tree)
  if (speed_limit_ > 0.0) {
    const double limit = speed_limit_is_percentage_ ?
      max_linear_velocity_ * speed_limit_ / 100.0 : speed_limit_;
    linear = std::min(linear, limit);
  }

  cmd.twist.linear.x = linear;
  cmd.twist.angular.z = angular;
  return cmd;
}

geometry_msgs::msg::PoseStamped CaBotBlindController::getLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const nav_msgs::msg::Path & global_plan)
{
  // first point at least lookahead_distance_ ahead of us, searching forward
  // from where we were last time so the robot cannot be pulled backwards by an
  // earlier part of a plan that loops near itself
  const size_t search_from = std::min(last_visited_index_, global_plan.poses.size() - 1);
  geometry_msgs::msg::PoseStamped lookahead_point = global_plan.poses.back();
  size_t chosen = global_plan.poses.size() - 1;

  for (size_t i = search_from; i < global_plan.poses.size(); ++i) {
    if (pointDist(current_pose.pose.position, global_plan.poses[i].pose.position) >=
      lookahead_distance_)
    {
      lookahead_point = global_plan.poses[i];
      chosen = i;
      break;
    }
  }
  last_visited_index_ = chosen;

  // keep the aim point close enough that the heading towards it still means
  // something on a long straight plan
  const double dist = pointDist(current_pose.pose.position, lookahead_point.pose.position);
  if (dist > max_lookahead_) {
    const double angle = std::atan2(
      lookahead_point.pose.position.y - current_pose.pose.position.y,
      lookahead_point.pose.position.x - current_pose.pose.position.x);
    lookahead_point.pose.position.x =
      current_pose.pose.position.x + max_lookahead_ * std::cos(angle);
    lookahead_point.pose.position.y =
      current_pose.pose.position.y + max_lookahead_ * std::sin(angle);
  }

  return lookahead_point;
}

void CaBotBlindController::localGoalVisualizationCallback()
{
  if (!has_local_goal_ || !local_goal_vis_pub_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = curr_local_goal_.header.frame_id.empty() ?
    "map" : curr_local_goal_.header.frame_id;
  marker.header.stamp = node->now();
  marker.ns = "blind_local_goal";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = curr_local_goal_.pose;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  local_goal_vis_pub_->publish(marker);
}

}  // namespace cabot_navigation2

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotBlindController, nav2_core::Controller)
