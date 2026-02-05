#include "cabot_navigation2/cabot_rl_controller.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>

using namespace std::chrono_literals;

namespace cabot_navigation2
{

CaBotRLController::CaBotRLController() {
}

CaBotRLController::~CaBotRLController() {
}

void CaBotRLController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  costmap_ros_ = costmap_ros.get();  // Get pointer to the costmap
  name_ = name;
  tf_ = tf;
  
  configure_count++;
  RCLCPP_INFO(logger_, "Configure called - count: %d", configure_count);

  // Load parameters
  // declare_parameter_if_not_declared(
  //   node, name_ + ".rl_topic", rclcpp::ParameterValue("/lidar/rl_action"));
  // node->get_parameter(name_ + ".rl_topic", rl_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".rl_cmd_topic", rclcpp::ParameterValue("/rl_robot_cmd"));
  node->get_parameter(name_ + ".rl_cmd_topic", rl_cmd_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".rl_info_topic", rclcpp::ParameterValue("/rl_robot_info"));
  node->get_parameter(name_ + ".rl_info_topic", rl_info_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".loc_goal_vis_topic", rclcpp::ParameterValue("/local_goal_vis"));
  node->get_parameter(name_ + ".loc_goal_vis_topic", loc_goal_vis_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance", rclcpp::ParameterValue(0.5)); // meters
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_lookahead", rclcpp::ParameterValue(10.0)); // meters
  node->get_parameter(name_ + ".max_lookahead", max_lookahead_);

  declare_parameter_if_not_declared(
    node, name_ + ".focus_goal_dist", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".focus_goal_dist", focus_goal_dist_);

  last_visited_index_ = 0; // Initialize the last visited index to the start of the path

  // rl_client = node->create_client<lidar_process_msgs::srv::RlAction>(rl_topic_);
  rl_cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      rl_cmd_topic_, 10, std::bind(&CaBotRLController::rlCommandCallback, this, std::placeholders::_1));

  rl_info_pub_ = node->create_publisher<lidar_process_msgs::msg::RobotMessage>(rl_info_topic_, 10);
  rl_info_pub_timer_ = node->create_wall_timer(100ms, std::bind(&CaBotRLController::rlInfoCallback, this));


  // Publish current local goal for visualization purposes
  local_goal_visualization_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(loc_goal_vis_topic_, 10);
  loc_goal_vis_timer_ = node->create_wall_timer(100ms, std::bind(&CaBotRLController::localGoalVisualizationCallback, this));

  current_command = geometry_msgs::msg::Twist();
  robot_info = lidar_process_msgs::msg::RobotMessage();
}

void CaBotRLController::localGoalVisualizationCallback()
{
  auto node = node_.lock();
  auto vis_msg = visualization_msgs::msg::Marker();

  double marker_size = 0.75;

  vis_msg.header.stamp = node->now();
  vis_msg.header.frame_id = "map";
  vis_msg.ns = "cabot_navigation2";
  vis_msg.id = 0;
  vis_msg.type = 2;
  vis_msg.action = 0;
  vis_msg.pose = curr_local_goal_.pose;
  vis_msg.scale.x = marker_size;
  vis_msg.scale.y = marker_size;
  vis_msg.scale.z = marker_size;
  vis_msg.color.r = 1.0;
  vis_msg.color.g = 0.0;
  vis_msg.color.b = 0.0;
  vis_msg.color.a = 1.0;

  local_goal_visualization_pub_->publish(vis_msg);
}

void CaBotRLController::rlCommandCallback(const geometry_msgs::msg::Twist::SharedPtr rl_cmd)
{
  // Group sequences: First time, then groups
  auto node = node_.lock();
  current_command.linear.x = rl_cmd->linear.x;
  current_command.angular.z = rl_cmd->angular.z;
}

void CaBotRLController::rlInfoCallback()
{
  auto node = node_.lock();

  // if (robot_info.robot_pos.x == 0) {
  //   robot_info.robot_pos.x = 0.0;
  //   robot_info.robot_pos.y = 0.0;
  //   robot_info.robot_vel.x = 0.0;
  //   robot_info.robot_vel.y = 0.0;
  //   robot_info.robot_goal.x = 0.0;
  //   robot_info.robot_goal.y = 0.0;
  //   robot_info.robot_th = 0.0;
  // }
  RCLCPP_INFO(logger_, "Publishing RL Info: Pos(%.2f, %.2f), Vel(%.2f, %.2f), Goal(%.2f, %.2f), Th(%.2f)",
    robot_info.robot_pos.x, robot_info.robot_pos.y,
    robot_info.robot_vel.linear.x, robot_info.robot_vel.angular.z,
    robot_info.robot_goal.x, robot_info.robot_goal.y,
    robot_info.robot_th);
  rl_info_pub_->publish(robot_info);
}

void CaBotRLController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up RL controller");
}

void CaBotRLController::activate()
{
  RCLCPP_INFO(logger_, "Activating RL controller");
}

void CaBotRLController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating RL controller");
}

void CaBotRLController::setPlan(const nav_msgs::msg::Path & path)
{
  auto node = node_.lock();
  // Transform global path into the robot's frame
  global_plan = path;
  last_visited_index_ = 0;
}

void CaBotRLController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
}

geometry_msgs::msg::TwistStamped CaBotRLController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // This wrapper fucntion calls the function that computes the velocity commands

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request Sent A");

  auto node = node_.lock();

  geometry_msgs::msg::TwistStamped velocity_cmd;
  velocity_cmd.header.stamp = node->now();
  velocity_cmd.header.frame_id = "base_link";

  if (global_plan.poses.size() == 0) {
    return velocity_cmd;
  }

  // Call your RL function to compute the optimal control action
  geometry_msgs::msg::PoseStamped local_goal = getLookaheadPoint(pose, global_plan);
  curr_local_goal_ = local_goal;

   // temporary code for goal handling! (DANGER!)
  double goal_dist = pointDist(pose.pose.position, local_goal.pose.position);
  if (goal_dist < focus_goal_dist_) {
    double desired_heading = std::atan2(local_goal.pose.position.y - pose.pose.position.y, local_goal.pose.position.x - pose.pose.position.x);
    double current_heading = tf2::getYaw(pose.pose.orientation);
    velocity_cmd.twist.linear.x = 1.0;
    velocity_cmd.twist.angular.z = std::min(1.0, desired_heading - current_heading);
    return velocity_cmd;
  }

  robot_info.robot_pos.x = pose.pose.position.x;
  robot_info.robot_pos.y = pose.pose.position.y;
  robot_info.robot_th = tf2::getYaw(pose.pose.orientation);
  robot_info.robot_vel.linear.x = velocity.linear.x;
  robot_info.robot_vel.angular.z = velocity.angular.z;
  robot_info.robot_goal.x = local_goal.pose.position.x;
  robot_info.robot_goal.y = local_goal.pose.position.y;

  // auto request = std::make_shared<lidar_process_msgs::srv::RlAction::Request>();
  // request->robot_pos.x = pose.pose.position.x;
  // request->robot_pos.y = pose.pose.position.y;
  // request->robot_th = tf2::getYaw(pose.pose.orientation);
  // request->robot_vel.x = velocity.linear.x * std::cos(velocity.angular.z);
  // request->robot_vel.y = velocity.linear.x * std::sin(velocity.angular.z);
  // request->robot_goal.x = local_goal.pose.position.x;
  // request->robot_goal.y = local_goal.pose.position.y;

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request Sent B");

  // while (!rl_client->wait_for_service(1s)) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  // }

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request Sent C");

  // auto result = rl_client->async_send_request(request);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request Sent D");
  // // Wait for the result.
  // if (rclcpp::spin_until_future_complete(node, result) ==
  //   rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call success");
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  // }
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request Sent E");

  if (current_command.linear.x == 0) {
      current_command.linear.x = 0.0;
      current_command.angular.z = 0.0;
  }

  velocity_cmd.twist.linear.x = current_command.linear.x;
  velocity_cmd.twist.angular.z= current_command.angular.z;

  return velocity_cmd;
}

geometry_msgs::msg::PoseStamped CaBotRLController::getLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const nav_msgs::msg::Path & global_plan)
{
  // This function gets the immediate next point outside of threhsold as the goal point
  // on the global plan

  geometry_msgs::msg::PoseStamped lookahead_point;

  double current_x = current_pose.pose.position.x;
  double current_y = current_pose.pose.position.y;

  bool found_point = false;

  for (size_t i = last_visited_index_; i < global_plan.poses.size(); ++i)
  {
    double dx = global_plan.poses[i].pose.position.x - current_x;
    double dy = global_plan.poses[i].pose.position.y - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance >= lookahead_distance_)
    {
      lookahead_point = global_plan.poses[i];
      last_visited_index_ = i;  // Update last visited index
      found_point = true;
      break;
    }
  }

  // If no point is found beyond the lookahead distance, use the last point
  if (!found_point)
  {
    lookahead_point = global_plan.poses.back();
    last_visited_index_ = global_plan.poses.size() - 1;
  }

  // Clamp the lookahead point to be within max_lookahead_
  if (pointDist(current_pose.pose.position, lookahead_point.pose.position) > max_lookahead_) {
    double angle_to_goal = std::atan2(lookahead_point.pose.position.y - current_y, lookahead_point.pose.position.x - current_x);
    lookahead_point.pose.position.x = current_x + max_lookahead_ * std::cos(angle_to_goal);
    lookahead_point.pose.position.y = current_y + max_lookahead_ * std::sin(angle_to_goal);
  }

  return lookahead_point;
}

bool CaBotRLController::hasReachedLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & lookahead_point)
{
  // This function checks if the robot's pose is within threshold dist of a point

  double dx = current_pose.pose.position.x - lookahead_point.pose.position.x;
  double dy = current_pose.pose.position.y - lookahead_point.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  return distance <= lookahead_distance_;
}

double CaBotRLController::pointDist(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace cabot_navigation2

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotRLController, nav2_core::Controller)
