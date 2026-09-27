#include "cabot_navigation2/cabot_hybrid_rl_controller.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <map>
#include <algorithm>
#include <stdexcept>
#include "angles/angles.h"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

using namespace std::chrono_literals;

namespace cabot_navigation2
{

CaBotHybridRLController::CaBotHybridRLController() {
}

CaBotHybridRLController::~CaBotHybridRLController() {
  parameter_callback_.reset();
}

void CaBotHybridRLController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  node_ = parent;
  auto node = node_.lock();
  costmap_ros_ = costmap_ros.get();  // Get pointer to the costmap
  name_ = name;
  tf_ = tf;

  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
  
  configure_count++;
  RCLCPP_INFO(logger_, "Configure called - count: %d", configure_count);

  // Load parameters
  // declare_parameter_if_not_declared(
  //   node, name_ + ".rl_topic", rclcpp::ParameterValue("/lidar/rl_action"));
  // node->get_parameter(name_ + ".rl_topic", rl_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".rl_people_topic", rclcpp::ParameterValue("/rl_people"));
  node->get_parameter(name_ + ".rl_people_topic", rl_people_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".rl_subgoal_topic", rclcpp::ParameterValue("/rl_subgoal"));
  node->get_parameter(name_ + ".rl_subgoal_topic", rl_subgoal_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".rl_info_topic", rclcpp::ParameterValue("/rl_robot_info"));
  node->get_parameter(name_ + ".rl_info_topic", rl_info_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".loc_goal_vis_topic", rclcpp::ParameterValue("/local_goal_vis"));
  node->get_parameter(name_ + ".loc_goal_vis_topic", loc_goal_vis_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".traj_vis_topic", rclcpp::ParameterValue("/control_output_vis"));
  node->get_parameter(name_ + ".traj_vis_topic", traj_vis_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".prediction_horizon", rclcpp::ParameterValue(1.0)); // seconds
  node->get_parameter(name_ + ".prediction_horizon", prediction_horizon_);

  declare_parameter_if_not_declared(
    node, name_ + ".sampling_rate", rclcpp::ParameterValue(0.1)); // seconds
  node->get_parameter(name_ + ".sampling_rate", sampling_rate_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_linear_velocity", rclcpp::ParameterValue(1.0)); // m/s
  node->get_parameter(name_ + ".max_linear_velocity", max_linear_velocity_);

  declare_parameter_if_not_declared(
    node, name_ + ".linear_sample_size", rclcpp::ParameterValue(3.0)); 
  node->get_parameter(name_ + ".linear_sample_size", linear_sample_size_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity", rclcpp::ParameterValue(0.785)); // rad/s
  node->get_parameter(name_ + ".max_angular_velocity", max_angular_velocity_);

  declare_parameter_if_not_declared(
    node, name_ + ".angular_sample_size", rclcpp::ParameterValue(10.0)); 
  node->get_parameter(name_ + ".angular_sample_size", angular_sample_size_);

  declare_parameter_if_not_declared(
    node, name_ + ".discount_factor", rclcpp::ParameterValue(0.9)); // Discount factor for future time steps
  node->get_parameter(name_ + ".discount_factor", discount_factor_);

  declare_parameter_if_not_declared(
    node, name_ + ".obstacle_costval", rclcpp::ParameterValue(250.0));
  node->get_parameter(name_ + ".obstacle_costval", obstacle_costval_);

  declare_parameter_if_not_declared(
    node, name_ + ".collision_radius", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".collision_radius", collision_radius_);

  declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance", rclcpp::ParameterValue(0.5)); // meters
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_lookahead", rclcpp::ParameterValue(10.0)); // meters
  node->get_parameter(name_ + ".max_lookahead", max_lookahead_);

  declare_parameter_if_not_declared(
    node, name_ + ".focus_goal_dist", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".focus_goal_dist", focus_goal_dist_);

  declare_parameter_if_not_declared(
    node, name_ + ".goal_cost_wt", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".goal_cost_wt", goal_cost_wt_);

  declare_parameter_if_not_declared(
    node, name_ + ".people_cost_wt", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".people_cost_wt", people_cost_wt_);

  declare_parameter_if_not_declared(
    node, name_ + ".heading_cost_wt", rclcpp::ParameterValue(0.3));
  node->get_parameter(name_ + ".heading_cost_wt", heading_cost_wt_);
  declare_parameter_if_not_declared(
    node, name_ + ".angular_cost_wt", rclcpp::ParameterValue(0.05));
  node->get_parameter(name_ + ".angular_cost_wt", angular_cost_wt_);

  for (const auto & parameter : std::vector<std::pair<std::string, double>>{
    {"avoidance_max_angle", 0.7}, {"avoidance_angular_velocity", 0.25},
    {"avoidance_probe_distance", 1.5}, {"avoidance_min_clearance", 0.65}})
  {
    declare_parameter_if_not_declared(node, name_ + "." + parameter.first,
      rclcpp::ParameterValue(parameter.second));
  }
  applyParameters(readParameters());

  const auto validation = validateParameters({});
  if (!validation.successful) {
    throw std::invalid_argument(validation.reason);
  }
  parameter_callback_ = node->add_on_set_parameters_callback(
    std::bind(&CaBotHybridRLController::validateParameters, this, std::placeholders::_1));

  last_visited_index_ = 0; // Initialize the last visited index to the start of the path

  // rl_client = node->create_client<lidar_process_msgs::srv::RlAction>(rl_topic_);
  rl_subgoal_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
      rl_subgoal_topic_, sensor_qos, std::bind(&CaBotHybridRLController::rlSubgoalCallback, this, std::placeholders::_1));

  rl_people_sub_ = node->create_subscription<lidar_process_msgs::msg::PositionHistoryArray>(
      rl_people_topic_, sensor_qos, std::bind(&CaBotHybridRLController::rlPeopleCallback, this, std::placeholders::_1));

  rl_info_pub_ = node->create_publisher<lidar_process_msgs::msg::RobotMessage>(rl_info_topic_, 10);

  // Publish selected trajectory for visualization purposes
  trajectory_visualization_pub_ = node->create_publisher<nav_msgs::msg::Path>(traj_vis_topic_, 10);

  // Publish current local goal for visualization purposes
  local_goal_visualization_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(loc_goal_vis_topic_, 10);

  current_command = geometry_msgs::msg::Twist();
  robot_info = lidar_process_msgs::msg::RobotMessage();
  
  rl_people_.clear();
  RCLCPP_INFO(logger_, "CaBotHybridRLController configured");
}

std::vector<std::pair<std::string, double *>> CaBotHybridRLController::parameterBindings()
{
  return {
    {"prediction_horizon", &prediction_horizon_}, {"sampling_rate", &sampling_rate_},
    {"max_linear_velocity", &max_linear_velocity_}, {"linear_sample_size", &linear_sample_size_},
    {"max_angular_velocity", &max_angular_velocity_}, {"angular_sample_size", &angular_sample_size_},
    {"discount_factor", &discount_factor_}, {"obstacle_costval", &obstacle_costval_},
    {"collision_radius", &collision_radius_}, {"lookahead_distance", &lookahead_distance_},
    {"max_lookahead", &max_lookahead_}, {"focus_goal_dist", &focus_goal_dist_},
    {"goal_cost_wt", &goal_cost_wt_}, {"people_cost_wt", &people_cost_wt_},
    {"heading_cost_wt", &heading_cost_wt_}, {"angular_cost_wt", &angular_cost_wt_},
    {"avoidance_max_angle", &avoidance_max_angle_},
    {"avoidance_angular_velocity", &avoidance_angular_velocity_},
    {"avoidance_probe_distance", &avoidance_probe_distance_},
    {"avoidance_min_clearance", &avoidance_min_clearance_}
  };
}

std::vector<rclcpp::Parameter> CaBotHybridRLController::readParameters()
{
  std::vector<std::string> names;
  for (const auto & binding : parameterBindings()) {
    names.push_back(name_ + "." + binding.first);
  }
  // One committed snapshot, including when several values are set atomically.
  return node_.lock()->get_parameters(names);
}

void CaBotHybridRLController::applyParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  const auto bindings = parameterBindings();
  for (size_t i = 0; i < bindings.size(); ++i) {
    *bindings[i].second = parameters.at(i).as_double();
  }
}

rcl_interfaces::msg::SetParametersResult CaBotHybridRLController::validateParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  std::map<std::string, double> values;
  for (const auto & parameter : readParameters()) {
    values[parameter.get_name()] = parameter.as_double();
  }
  for (const auto & parameter : parameters) {
    if (parameter.get_name().rfind(name_ + ".", 0) != 0) {
      continue;
    }
    const auto found = values.find(parameter.get_name());
    if (found == values.end()) {
      result.reason = parameter.get_name() + " requires controller reconfiguration";
      return result;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.reason = parameter.get_name() + " must be a double";
      return result;
    }
    found->second = parameter.as_double();
  }
  for (const auto & entry : values) {
    if (!std::isfinite(entry.second) || entry.second < 0.0) {
      result.reason = entry.first + " must be finite and nonnegative";
      return result;
    }
  }
  const auto value = [&](const char * key) {return values.at(name_ + "." + key);};
  if (value("sampling_rate") <= 0.0 || value("prediction_horizon") < value("sampling_rate")) {
    result.reason = "Require 0 < sampling_rate <= prediction_horizon";
    return result;
  }
  if (value("lookahead_distance") <= 0.0 || value("max_lookahead") < value("lookahead_distance")) {
    result.reason = "Require 0 < lookahead_distance <= max_lookahead";
    return result;
  }
  for (const auto key : {"linear_sample_size", "angular_sample_size"}) {
    const double count = value(key);
    if (count < 1.0 || count > 200.0 || std::floor(count) != count) {
      result.reason = std::string(key) + " must be a whole number from 1 to 200 (double)";
      return result;
    }
  }
  if (value("discount_factor") > 1.0 || value("obstacle_costval") < 1.0 || value("obstacle_costval") > 255.0) {
    result.reason = "Require discount_factor in [0, 1] and obstacle_costval in [1, 255]";
    return result;
  }
  if (value("avoidance_max_angle") > M_PI / 3.0 ||
    value("avoidance_angular_velocity") > 0.5 ||
    value("avoidance_probe_distance") < 0.3 || value("avoidance_probe_distance") > 3.0 ||
    value("avoidance_min_clearance") < 0.3)
  {
    result.reason = "Avoidance requires angle <= pi/3, angular velocity <= 0.5, "
      "probe distance in [0.3, 3.0], and clearance >= 0.3";
    return result;
  }
  const double steps = std::ceil(value("prediction_horizon") / value("sampling_rate"));
  if (steps * (value("linear_sample_size") + 1.0) * (value("angular_sample_size") + 2.0) > 200000.0) {
    result.reason = "Trajectory sampling budget exceeded (200000 predicted poses per cycle)";
    return result;
  }
  // Validate only. Another callback may still reject this transaction. The control
  // loop reads committed parameters rather than mutating state before acceptance.
  result.successful = true;
  return result;
}

void CaBotHybridRLController::localGoalVisualizationCallback()
{
  auto node = node_.lock();
  auto vis_msg = visualization_msgs::msg::Marker();

  double marker_size = 0.75;

  vis_msg.header.stamp = node->now();
  vis_msg.header.frame_id = "map";
  vis_msg.ns = name_ + "/local_goal";
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

void CaBotHybridRLController::rlPeopleCallback(const lidar_process_msgs::msg::PositionHistoryArray::SharedPtr rl_people_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto node = node_.lock();
  rl_people_.clear();
  horizon_people_ = rl_people_msg->positions_history.size();
  rl_people_tmp_ = rl_people_msg->positions_history;
  if (horizon_people_ == 0) {
    num_people_ = 0;
    return;
  }
  try {
    for (size_t i = 0; i < horizon_people_; ++i) {
      const auto & hist = rl_people_tmp_.at(i);  // at() for bounds check

      const size_t pos_sz = hist.positions.size();
      const size_t ids_sz = hist.ids.size();
      const size_t count  = std::min(pos_sz, ids_sz);  // clamp by both vectors

      lidar_process_msgs::msg::PositionArray people_array;
      people_array.quantity = static_cast<uint32_t>(count);
      num_people_ = static_cast<uint32_t>(count);

      people_array.positions.reserve(count);
      people_array.ids.reserve(count);

      for (size_t j = 0; j < count; ++j) {
        geometry_msgs::msg::Point pos;
        pos.x = hist.positions.at(j).x;  // at() for bounds check
        pos.y = hist.positions.at(j).y;
        people_array.positions.push_back(pos);
        people_array.ids.push_back(hist.ids.at(j));  // at() for bounds check
      }
      rl_people_.push_back(std::move(people_array));
    }
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(logger_, "rlPeopleCallback out_of_range: history=%zu horizon=%zu what=%s",
      rl_people_tmp_.size(), horizon_people_, e.what());
    // Leave rl_people_ as-is (cleared) to fail-safe
    num_people_ = 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "rlPeopleCallback exception: %s", e.what());
    num_people_ = 0;
  }
}

void CaBotHybridRLController::rlSubgoalCallback(const geometry_msgs::msg::Point::SharedPtr rl_subgoal)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!std::isfinite(rl_subgoal->x) || !std::isfinite(rl_subgoal->y)) {
    rl_ready_ = false;
    return;
  }
  // Copying the subgoal over
  auto node = node_.lock();
  rl_subgoal_.x = rl_subgoal->x;
  rl_subgoal_.y = rl_subgoal->y;
  rl_ready_ = true;
}

void CaBotHybridRLController::rlInfoCallback()
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

void CaBotHybridRLController::cleanup()
{
  parameter_callback_.reset();
  std::lock_guard<std::mutex> lock(state_mutex_);
  rl_ready_ = false;
  rl_people_.clear();
  num_people_ = 0;
  rl_subgoal_sub_.reset();
  rl_people_sub_.reset();
  rl_info_pub_.reset();
  trajectory_visualization_pub_.reset();
  local_goal_visualization_pub_.reset();
  RCLCPP_INFO(logger_, "Cleaning up RL controller");
}

void CaBotHybridRLController::activate()
{
  RCLCPP_INFO(logger_, "Activating RL controller");
}

void CaBotHybridRLController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating RL controller");
}

void CaBotHybridRLController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto node = node_.lock();
  // Check if path's positions are the same as the current global plan
  bool same = true;
  if (path.poses.size() == global_plan_.poses.size()) {
    for (size_t i = 0; i < path.poses.size(); ++i) {
      if (abs(path.poses[i].pose.position.x - global_plan_.poses[i].pose.position.x) > 0.0001 ||
          abs(path.poses[i].pose.position.y - global_plan_.poses[i].pose.position.y) > 0.0001) {
        same = false;
        break;
      }
    }
  } else {
    same = false;
  }
  if (same) {
    RCLCPP_INFO(logger_, "Received same global plan, ignoring.");
    return;
  } else {
    global_plan_ = path;
    last_visited_index_ = 0;
    avoidance_side_ = 0;
    // Do not steer toward the previous destination while waiting for new RL input.
    rl_ready_ = false;
    num_people_ = 0;
    rl_people_.clear();
    RCLCPP_INFO(logger_, "Received new global plan with %zu points.", global_plan_.poses.size());
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
      auto pose = global_plan_.poses[i];
      RCLCPP_INFO(logger_, "Path point %zu: (%.2f, %.2f)", i, pose.pose.position.x, pose.pose.position.y);
    }
    return;
  }
}

void CaBotHybridRLController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
}

geometry_msgs::msg::TwistStamped CaBotHybridRLController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // This wrapper fucntion calls the function that computes the velocity commands

  const auto parameters = readParameters();
  std::lock_guard<std::mutex> lock(state_mutex_);
  applyParameters(parameters);

  RCLCPP_INFO(logger_, "Request Sent A");

  auto node = node_.lock();

  geometry_msgs::msg::TwistStamped velocity_cmd;
  velocity_cmd.header.stamp = node->now();
  velocity_cmd.header.frame_id = "base_link";

  if (global_plan_.poses.size() == 0) {
    return velocity_cmd;
  }

  // Call your RL function to compute the optimal control action
  geometry_msgs::msg::PoseStamped  local_goal= getLookaheadPoint(pose, global_plan_);
  curr_local_goal_ = local_goal;
  localGoalVisualizationCallback();

  //  // temporary code for goal handling! (DANGER!)
  // double goal_dist = pointDist(pose.pose.position, local_goal.pose.position);
  // if (goal_dist < focus_goal_dist_) {
  //   double desired_heading = std::atan2(local_goal.pose.position.y - pose.pose.position.y, local_goal.pose.position.x - pose.pose.position.x);
  //   double current_heading = tf2::getYaw(pose.pose.orientation);
  //   velocity_cmd.twist.linear.x = 1.0;
  //   velocity_cmd.twist.angular.z = std::min(1.0, desired_heading - current_heading);
  //   return velocity_cmd;
  // }

  robot_info.robot_pos.x = pose.pose.position.x;
  robot_info.robot_pos.y = pose.pose.position.y;
  robot_info.robot_th = tf2::getYaw(pose.pose.orientation);
  robot_info.robot_vel.linear.x = velocity.linear.x;
  robot_info.robot_vel.angular.z = velocity.angular.z;
  robot_info.robot_goal.x = local_goal.pose.position.x;
  robot_info.robot_goal.y = local_goal.pose.position.y;
  // Only the controller computing this command may supply the shared RL input.
  rlInfoCallback();
  if (!rl_ready_) {
    return velocity_cmd;
  }

  // Call your MPC function to compute the optimal control action
  geometry_msgs::msg::Twist control_cmd = computeMPCControl(pose, velocity);
  velocity_cmd.twist = control_cmd;

  return velocity_cmd;
}

geometry_msgs::msg::Twist CaBotHybridRLController::computeMPCControl(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // This function samples the MPC trajectories and computes the costs of the trajectories
  // The cost with the lowest trajectory will be selected and the associated velocities returned.

  geometry_msgs::msg::Twist best_control;
  double min_cost = std::numeric_limits<double>::infinity();

  nav_msgs::msg::Path best_trajectory;

  // Generate all the trajectories based on sampled velocities
  std::vector<Trajectory> trajectories = generateTrajectoriesSimple(pose, velocity);

  // Loop over the generated trajectories and calculate their costs
  for (const auto & trajectory : trajectories)
  {
    double cost = calculateCost(pose, trajectory);

    // Update the best control if this trajectory has a lower cost
    const bool tied = std::isfinite(cost) && std::abs(cost - min_cost) <= 1e-9;
    const bool less_turn = std::abs(trajectory.control.angular.z) < std::abs(best_control.angular.z) - 1e-9;
    const bool same_turn = std::abs(trajectory.control.angular.z - best_control.angular.z) <= 1e-9;
    if (std::isfinite(cost) && (cost < min_cost - 1e-9 ||
      (tied && (less_turn || (same_turn && trajectory.control.linear.x < best_control.linear.x)))))
    {
      min_cost = cost;
      best_control = trajectory.control;  // Assuming we store the control input corresponding to each trajectory
      best_trajectory.header = pose.header;
      best_trajectory.poses = trajectory.trajectory;
    }
  }
  if (std::isfinite(min_cost) && best_control.linear.x < 1e-6) {
    chooseAvoidanceTurn(pose, best_control, best_trajectory);
  } else if (best_control.linear.x > 0.05) {
    avoidance_side_ = 0;
  }
  trajectory_visualization_pub_->publish(best_trajectory);

  // No admissible trajectory: stop.
  if (min_cost >= std::numeric_limits<double>::infinity() - 1){
    best_control.linear.x = 0.0;
    best_control.angular.z = 0.0;
  }

  return best_control;
}

bool CaBotHybridRLController::chooseAvoidanceTurn(
  const geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::Twist & control, nav_msgs::msg::Path & trajectory)
{
  const auto & position = pose.pose.position;
  const auto & goal = curr_local_goal_.pose.position;
  const double distance = pointDist(position, goal);
  const double route_heading = std::atan2(goal.y - position.y, goal.x - position.x);
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double angular_limit = std::min(avoidance_angular_velocity_, max_angular_velocity_);
  // Bound the absolute deviation from the route, not a new turn relative to the
  // body on every cycle. Repeated attempts therefore cannot turn us backwards.
  if (avoidance_max_angle_ <= 0.0 || angular_limit <= 0.0 || max_linear_velocity_ <= 0.0 ||
    distance < std::max(focus_goal_dist_, 0.3) ||
    std::abs(angles::shortest_angular_distance(route_heading, yaw)) > avoidance_max_angle_ + 0.02)
  {
    return false;
  }

  auto * map = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
  const auto footprint = costmap_ros_->getRobotFootprint();
  const double probe_distance = std::min(distance, avoidance_probe_distance_);
  const int steps = static_cast<int>(std::ceil(probe_distance / 0.05));
  double best_score = -std::numeric_limits<double>::infinity();
  double best_heading = route_heading;
  int best_side = 0;
  const double rl_heading = std::atan2(rl_subgoal_.y - position.y, rl_subgoal_.x - position.x);

  for (int index = 0; index <= 12; ++index) {
    const int side = index == 0 ? 0 : (index % 2 ? 1 : -1);
    const double offset = side * ((index + 1) / 2) * avoidance_max_angle_ / 6.0;
    const double heading = route_heading + offset;
    const double turn = angles::shortest_angular_distance(yaw, heading);
    bool clear = true;
    double clearance = 2.0;
    double obstacle_cost = 0.0;
    const auto people_clear = [&](double x, double y, double time) {
        if (rl_people_.empty()) {return true;}
        const size_t t = std::min(rl_people_.size() - 1,
          static_cast<size_t>(std::min(1000000.0, time / sampling_rate_)));
        for (const auto & person : rl_people_[t].positions) {
          const double separation = std::hypot(x - person.x, y - person.y);
          if (!std::isfinite(separation) || separation < avoidance_min_clearance_) {return false;}
          clearance = std::min(clearance, separation);
        }
        return true;
      };
    // Check the footprint swept by the rotation before evaluating the exit ray.
    const int rotation_steps = std::max(1, static_cast<int>(std::ceil(std::abs(turn) / 0.05)));
    for (int i = 0; i <= rotation_steps; ++i) {
      if (checker.footprintCostAtPose(position.x, position.y,
        yaw + turn * i / rotation_steps, footprint) >= nav2_costmap_2d::LETHAL_OBSTACLE ||
        !people_clear(position.x, position.y, std::abs(turn) / angular_limit * i / rotation_steps))
      {
        clear = false;
        break;
      }
    }
    for (int i = 0; clear && i <= steps; ++i) {
      const double travel = probe_distance * i / steps;
      auto probe = pose.pose;
      probe.position.x += travel * std::cos(heading);
      probe.position.y += travel * std::sin(heading);
      const double cell_cost = getCostFromCostmap(probe);
      if (cell_cost >= obstacle_costval_ || checker.footprintCostAtPose(
        probe.position.x, probe.position.y, heading, footprint) >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        clear = false;
        break;
      }
      obstacle_cost += cell_cost / 255.0 / (steps + 1);
      // Predicted people are sampled at the expected exit time. Hold the final
      // prediction if the short prediction horizon ends before the exit ray.
      const double arrival_time = std::abs(turn) / angular_limit +
        travel / std::max(0.1, max_linear_velocity_);
      clear = people_clear(probe.position.x, probe.position.y, arrival_time);
    }
    if (!clear) {continue;}
    const double rl_error = num_people_ > 0 ?
      std::abs(angles::shortest_angular_distance(heading, rl_heading)) : 0.0;
    const double score = clearance - 0.5 * std::abs(offset) - 0.25 * rl_error -
      0.5 * obstacle_cost + (side != 0 && side == avoidance_side_ ? 0.1 : 0.0);
    if (score > best_score + 1e-9) {
      best_score = score;
      best_heading = heading;
      best_side = side;
    }
  }
  if (!std::isfinite(best_score)) {return false;}
  const double error = angles::shortest_angular_distance(yaw, best_heading);
  control.linear.x = 0.0;
  control.angular.z = std::abs(error) < 0.02 ? 0.0 :
    std::clamp(error / prediction_horizon_, -angular_limit, angular_limit);
  avoidance_side_ = best_side;
  trajectory.header = pose.header;
  trajectory.poses.clear();
  for (int i = 0; i <= 10; ++i) {
    auto predicted = pose;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw + control.angular.z * prediction_horizon_ * i / 10.0);
    predicted.pose.orientation = tf2::toMsg(q);
    trajectory.poses.push_back(predicted);
  }
  return true;
}

std::vector<Trajectory> CaBotHybridRLController::generateTrajectoriesSimple(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & /* velocity */)
{
  std::vector<Trajectory> trajectories;
  const int linear_samples = static_cast<int>(linear_sample_size_);
  const int angular_samples = static_cast<int>(angular_sample_size_);
  const int steps = static_cast<int>(std::ceil(prediction_horizon_ / sampling_rate_));
  double max_linear = max_linear_velocity_;
  const double goal_distance = pointDist(current_pose.pose.position, curr_local_goal_.pose.position);
  if (goal_distance < focus_goal_dist_) {
    // Slow down near the local goal, but still evaluate obstacles and all turns.
    max_linear = std::min(max_linear, goal_distance / prediction_horizon_);
  }
  std::vector<double> angular_velocities;
  for (int i = 0; i <= angular_samples; ++i) {
    angular_velocities.push_back(max_angular_velocity_ * (2.0 * i / angular_samples - 1.0));
  }
  if (angular_samples % 2 != 0) {
    angular_velocities.push_back(0.0);  // Always include a true stop and straight motion.
  }
  for (int i = 0; i <= linear_samples; ++i) {
    const double linear_vel = max_linear * i / linear_samples;
    for (const double angular_vel : angular_velocities) {
      geometry_msgs::msg::Twist control;
      control.linear.x = linear_vel;
      control.angular.z = angular_vel;
      auto predicted = current_pose;
      double theta = tf2::getYaw(predicted.pose.orientation);
      std::vector<geometry_msgs::msg::PoseStamped> trajectory;
      trajectory.reserve(steps);
      for (int step = 0; step < steps; ++step) {
        const double dt = std::min(sampling_rate_, prediction_horizon_ - step * sampling_rate_);
        if (std::abs(angular_vel) < 1e-9) {
          predicted.pose.position.x += linear_vel * dt * std::cos(theta);
          predicted.pose.position.y += linear_vel * dt * std::sin(theta);
        } else {
          predicted.pose.position.x += linear_vel / angular_vel *
            (std::sin(theta + angular_vel * dt) - std::sin(theta));
          predicted.pose.position.y += linear_vel / angular_vel *
            (std::cos(theta) - std::cos(theta + angular_vel * dt));
        }
        theta += angular_vel * dt;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        predicted.pose.orientation = tf2::toMsg(q);
        trajectory.push_back(predicted);
      }
      trajectories.emplace_back(control, trajectory);
    }
  }
  return trajectories;
}

std::vector<Trajectory> CaBotHybridRLController::generateTrajectoriesImproved(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // This function samples trajectories that follow a fixed linear velocity
  // But the angular velocity can change in the middle of the duration
  std::vector<Trajectory> trajectories;

  double linear_sample_resolution = max_linear_velocity_ / linear_sample_size_;
  double angular_vel_lim = max_angular_velocity_;
  double angular_sample_resolution = angular_vel_lim / angular_sample_size_;

  // Sample a set of linear velocities
  for (double initial_linear_vel = 0.0; initial_linear_vel <= max_linear_velocity_; initial_linear_vel += linear_sample_resolution)
  {
    double secondary_max_linear_velocity;
    if (abs(initial_linear_vel) < 0.001) {
      secondary_max_linear_velocity = 0.001;
    } else {
      secondary_max_linear_velocity = max_linear_velocity_;
    }
    for (double secondary_linear_vel = 0.0; secondary_linear_vel <= secondary_max_linear_velocity; secondary_linear_vel += linear_sample_resolution)
    {
      // Sample initial and secondary angular velocities
      for (double initial_angular_vel = -angular_vel_lim; initial_angular_vel <= angular_vel_lim; initial_angular_vel += angular_sample_resolution)
      {
        for (double secondary_angular_vel = -angular_vel_lim; secondary_angular_vel <= angular_vel_lim; secondary_angular_vel += angular_sample_resolution)
        {
          // Start with the current pose and initial control
          geometry_msgs::msg::PoseStamped current_pose_copy = current_pose;
          double current_x = current_pose_copy.pose.position.x;
          double current_y = current_pose_copy.pose.position.y;
          double current_theta = tf2::getYaw(current_pose_copy.pose.orientation);

          std::vector<geometry_msgs::msg::PoseStamped> trajectory;
          geometry_msgs::msg::Twist initial_control;
          initial_control.linear.x = initial_linear_vel;
          initial_control.angular.z = initial_angular_vel;

          // Determine the time at which to switch to the secondary angular velocity
          double switch_time = prediction_horizon_ / 2.0;

          // Predict the trajectory over the prediction horizon
          for (double t = sampling_rate_; t <= prediction_horizon_; t += sampling_rate_)
          {
            // Use initial angular velocity before switch time, secondary after
            double angular_vel;
            double linear_vel;
            if (t < switch_time) {
              angular_vel = initial_angular_vel;
              linear_vel = initial_linear_vel;
            } else {
              angular_vel = secondary_angular_vel;
              linear_vel = secondary_linear_vel;
            }

            // Simulate robot dynamics
            if (abs(angular_vel) < 0.0001)
            {
              current_x += linear_vel * sampling_rate_ * cos(current_theta);
              current_y += linear_vel * sampling_rate_ * sin(current_theta);
            } else{
              current_x += linear_vel / angular_vel * (sin(current_theta + angular_vel * sampling_rate_) - sin(current_theta));
              current_y -= linear_vel / angular_vel * (cos(current_theta + angular_vel * sampling_rate_) - cos(current_theta));
            }
            current_theta += angular_vel * sampling_rate_;
            

            geometry_msgs::msg::PoseStamped predicted_pose;
            predicted_pose.pose.position.x = current_x;
            predicted_pose.pose.position.y = current_y;
            tf2::Quaternion q;
            q.setRPY(0, 0, current_theta);
            predicted_pose.pose.orientation = tf2::toMsg(q);

            trajectory.push_back(predicted_pose);
          }

          // Store this trajectory with its initial control
          trajectories.push_back(Trajectory(initial_control, trajectory));
        }
      }
    }
  }

  return trajectories;
}

geometry_msgs::msg::PoseStamped CaBotHybridRLController::getLookaheadPoint(
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

bool CaBotHybridRLController::hasReachedLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & lookahead_point)
{
  // This function checks if the robot's pose is within threshold dist of a point

  double dx = current_pose.pose.position.x - lookahead_point.pose.position.x;
  double dy = current_pose.pose.position.y - lookahead_point.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  return distance <= lookahead_distance_;
}

double CaBotHybridRLController::calculateCost(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const Trajectory trajectory)
{
  // Define a cost function to evaluate how good the trajectory is
  // This includes distances to the local goal, costmap information, and people trajectory cost

  double cost = 0.0;

  std::vector<geometry_msgs::msg::PoseStamped> sampled_trajectory = trajectory.trajectory;

  // Add costmap-related cost
  double discount = 1.0;
  double cumulative_discount = 0.0;
  double step_cost = 0.0;
  double people_cost = 0.0;
  double goal_cost = 0.0;

  double goal_dist;
  double min_goal_dist = std::numeric_limits<double>::infinity();

    // Add people trajectory-related cost
  people_cost = calculatePeopleCost(sampled_trajectory);

  cost += people_cost_wt_ * people_cost;

  for (const auto & pose : sampled_trajectory)
  {

    step_cost = getCostFromCostmap(pose.pose);
    if (step_cost >= obstacle_costval_)
    {
      cost = std::numeric_limits<double>::infinity();
      return cost;
    }
    
    if (num_people_ > 0) {
      goal_dist = pointDist(pose.pose.position, rl_subgoal_);
    } else {
      goal_dist = pointDist(pose.pose.position, curr_local_goal_.pose.position);
    }
    if (goal_dist < min_goal_dist) {
      min_goal_dist = goal_dist;
    }

  }
  goal_cost = min_goal_dist;
  
  cost += goal_cost_wt_ * goal_cost;

  // A pure rotation does not change any position cost. Assess orientation toward
  // the route goal so waiting and turning away are no longer indistinguishable.
  // Keep this anchored to the route even when RL proposes a retreating subgoal.
  const auto & end_pose = sampled_trajectory.back().pose;
  const double dx = curr_local_goal_.pose.position.x - end_pose.position.x;
  const double dy = curr_local_goal_.pose.position.y - end_pose.position.y;
  if (std::hypot(dx, dy) > 1e-6) {
    const double error = angles::shortest_angular_distance(
      tf2::getYaw(end_pose.orientation), std::atan2(dy, dx));
    cost += heading_cost_wt_ * std::abs(error);
  }
  cost += angular_cost_wt_ * std::abs(trajectory.control.angular.z);
  return cost;
}

double CaBotHybridRLController::getCostFromCostmap(const geometry_msgs::msg::Pose & pose)
{
  // This function returns the cost information from the costmap

  unsigned int mx, my;
  double wx = pose.position.x;
  double wy = pose.position.y;

  // Convert world coordinates to map coordinates
  if (costmap_ros_->getCostmap()->worldToMap(wx, wy, mx, my))
  {
    // Get cost at map coordinates
    return static_cast<double>(costmap_ros_->getCostmap()->getCost(mx, my));
  }
  else
  {
    // Return high cost if the position is out of bounds or in an unknown area
    return 255.0;  // Maximum cost in costmap is typically 255 for obstacles
  }
}

double CaBotHybridRLController::calculatePeopleCost(
  const std::vector<geometry_msgs::msg::PoseStamped> & sampled_trajectory)
{
  double discount = 1.0;
  double people_cost = 0.0;
  const size_t num_time_steps = sampled_trajectory.size();

  double min_dist;
  for (size_t t = 0; t < num_time_steps; ++t)
  {
    try {
      if (t < rl_people_.size())
      {
        discount = std::pow(discount_factor_, static_cast<double>(t));
        const auto & current_people = rl_people_.at(t);  // at() for bounds check

        min_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < current_people.positions.size(); ++i)
        {
          const auto & robot_pose = sampled_trajectory.at(t).pose;          // at() for bounds check
          const auto & person     = current_people.positions.at(i);          // at() for bounds check
          double dx = robot_pose.position.x - person.x;
          double dy = robot_pose.position.y - person.y;
          double dist = std::sqrt(dx * dx + dy * dy);
          if (dist < 0.0001) dist = 0.0001;
          if (dist < min_dist) min_dist = dist;
        }
        people_cost += discount * std::exp(collision_radius_ - min_dist);
      }
    } catch (const std::out_of_range &e) {
      RCLCPP_ERROR(logger_, "calculatePeopleCost out_of_range: t=%zu traj=%zu people=%zu what=%s",
        t, num_time_steps, rl_people_.size(), e.what());
      // Fail-safe: stop accumulating and return what we have
      break;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger_, "calculatePeopleCost exception at t=%zu: %s", t, e.what());
      break;
    }
  }

  return people_cost;
}

double CaBotHybridRLController::pointDist(
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
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotHybridRLController, nav2_core::Controller)
