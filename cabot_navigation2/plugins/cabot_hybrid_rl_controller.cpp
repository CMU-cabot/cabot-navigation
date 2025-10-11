#include "cabot_navigation2/cabot_hybrid_rl_controller.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>

using namespace std::chrono_literals;

namespace cabot_navigation2
{

CaBotHybridRLController::CaBotHybridRLController() {
}

CaBotHybridRLController::~CaBotHybridRLController() {
}

void CaBotHybridRLController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
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

  last_visited_index_ = 0; // Initialize the last visited index to the start of the path

  // rl_client = node->create_client<lidar_process_msgs::srv::RlAction>(rl_topic_);
  rl_subgoal_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
      rl_subgoal_topic_, sensor_qos, std::bind(&CaBotHybridRLController::rlSubgoalCallback, this, std::placeholders::_1));

  rl_people_sub_ = node->create_subscription<lidar_process_msgs::msg::PositionHistoryArray>(
      rl_people_topic_, sensor_qos, std::bind(&CaBotHybridRLController::rlPeopleCallback, this, std::placeholders::_1));

  rl_info_pub_ = node->create_publisher<lidar_process_msgs::msg::RobotMessage>(rl_info_topic_, 10);
  rl_info_pub_timer_ = node->create_wall_timer(100ms, std::bind(&CaBotHybridRLController::rlInfoCallback, this));

  // Publish selected trajectory for visualization purposes
  trajectory_visualization_pub_ = node->create_publisher<nav_msgs::msg::Path>(traj_vis_topic_, 10);

  // Publish current local goal for visualization purposes
  local_goal_visualization_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(loc_goal_vis_topic_, 10);
  loc_goal_vis_timer_ = node->create_wall_timer(100ms, std::bind(&CaBotHybridRLController::localGoalVisualizationCallback, this));

  current_command = geometry_msgs::msg::Twist();
  robot_info = lidar_process_msgs::msg::RobotMessage();
  
  rl_people_.clear();
  RCLCPP_INFO(logger_, "CaBotHybridRLController configured");
}

void CaBotHybridRLController::localGoalVisualizationCallback()
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

void CaBotHybridRLController::rlPeopleCallback(const lidar_process_msgs::msg::PositionHistoryArray::SharedPtr rl_people)
{
  auto node = node_.lock();
  rl_people_.clear();
  horizon_people_ = rl_people->positions_history.size();
  if (horizon_people_ == 0) {
    num_people_ = 0;
    return;
  }
  try {
    for (size_t i = 0; i < horizon_people_; ++i) {
      const auto & hist = rl_people->positions_history.at(i);  // at() for bounds check

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
      rl_people->positions_history.size(), horizon_people_, e.what());
    // Leave rl_people_ as-is (cleared) to fail-safe
    num_people_ = 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "rlPeopleCallback exception: %s", e.what());
    num_people_ = 0;
  }
}

void CaBotHybridRLController::rlSubgoalCallback(const geometry_msgs::msg::Point::SharedPtr rl_subgoal)
{
  // Copying the subgoal over
  auto node = node_.lock();
  rl_subgoal_.x = rl_subgoal->x;
  rl_subgoal_.y = rl_subgoal->y;
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
    RCLCPP_INFO(logger_, "Received new global plan with %zu points.", global_plan_.poses.size());
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

  RCLCPP_INFO(logger_, "Request Sent A");

  auto node = node_.lock();

  geometry_msgs::msg::TwistStamped velocity_cmd;
  velocity_cmd.header.stamp = node->now();
  velocity_cmd.header.frame_id = "base_link";

  if (global_plan_.poses.size() == 0) {
    return velocity_cmd;
  }

  // Call your RL function to compute the optimal control action
  geometry_msgs::msg::PoseStamped local_goal = getLookaheadPoint(pose, global_plan_);
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

  // temporary code for goal handling! (DANGER!)
  double goal_dist = pointDist(pose.pose.position, curr_local_goal_.pose.position);
  if (goal_dist < focus_goal_dist_) {
    double desired_heading = std::atan2(curr_local_goal_.pose.position.y - pose.pose.position.y, 
                                        curr_local_goal_.pose.position.x - pose.pose.position.x);
    double current_heading = tf2::getYaw(pose.pose.orientation);
    best_control.linear.x = 1.0;
    best_control.angular.z = std::max(-1.0, std::min(1.0, desired_heading - current_heading));
    return best_control;
  }

  // Generate all the trajectories based on sampled velocities
  std::vector<Trajectory> trajectories = generateTrajectoriesSimple(pose, velocity);

  // Loop over the generated trajectories and calculate their costs
  for (const auto & trajectory : trajectories)
  {
    double cost = calculateCost(pose, trajectory);

    // Update the best control if this trajectory has a lower cost
    if (cost < min_cost)
    {
      min_cost = cost;
      best_control = trajectory.control;  // Assuming we store the control input corresponding to each trajectory
      best_trajectory.header = pose.header;
      best_trajectory.poses = trajectory.trajectory;
    }
  }
  trajectory_visualization_pub_->publish(best_trajectory);

  // smooth the control with prior control
  if (min_cost >= std::numeric_limits<double>::infinity() - 1){
    best_control.linear.x = 0.0;
    best_control.angular.z = 0.0;
  }

  return best_control;
}

std::vector<Trajectory> CaBotHybridRLController::generateTrajectoriesSimple(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // This function samples trajectories that follow a fixed linear and angular velocities
  std::vector<Trajectory> trajectories;

  double linear_sample_resolution = max_linear_velocity_ / linear_sample_size_;

  // Sample a set of velocities and predict the corresponding trajectories
  for (double linear_vel = 0.0; linear_vel <= max_linear_velocity_; linear_vel += linear_sample_resolution)
  {
    double linear_portion = linear_vel / max_linear_velocity_;
    // double angular_vel_lim = (1 - linear_portion) * max_angular_velocity_;
    double angular_vel_lim = max_angular_velocity_;
    double angular_sample_resolution = angular_vel_lim * 2.0 / angular_sample_size_;
    for (double angular_vel = -angular_vel_lim; angular_vel <= angular_vel_lim; angular_vel += angular_sample_resolution)
    {
      // Generate a single trajectory
      std::vector<geometry_msgs::msg::PoseStamped> trajectory;
      geometry_msgs::msg::Twist control;
      control.linear.x = linear_vel;
      control.angular.z = angular_vel;

      // Start with the current pose
      geometry_msgs::msg::PoseStamped current_pose_copy = current_pose;
      double current_x = current_pose_copy.pose.position.x;
      double current_y = current_pose_copy.pose.position.y;
      double current_theta = tf2::getYaw(current_pose_copy.pose.orientation);

      // Predict the trajectory over the prediction horizon
      for (double t = sampling_rate_; t <= prediction_horizon_; t += sampling_rate_)
      {
        // Simulate robot dynamics
        if (abs(angular_vel) < 0.0001)
        {
          current_x += linear_vel * sampling_rate_ * cos(current_theta);
          current_y += linear_vel * sampling_rate_ * sin(current_theta);
        } else{
          current_x += linear_vel / angular_vel * (sin(current_theta + angular_vel * sampling_rate_) - sin(current_theta));
          current_y += linear_vel / angular_vel * (-cos(current_theta + angular_vel * sampling_rate_) + cos(current_theta));
        }
        // current_x += linear_vel * sampling_rate_ * cos(current_theta);
        // current_y += linear_vel * sampling_rate_ * sin(current_theta);
        current_theta += angular_vel * sampling_rate_;

        geometry_msgs::msg::PoseStamped predicted_pose;
        predicted_pose.pose.position.x = current_x;
        predicted_pose.pose.position.y = current_y;
        tf2::Quaternion q;
        q.setRPY(0, 0, current_theta);
        predicted_pose.pose.orientation = tf2::toMsg(q);

        trajectory.push_back(predicted_pose);
      }

      // Store this trajectory
      trajectories.push_back(Trajectory(control, trajectory));
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
  lookahead_point = global_plan.poses.back();

  double current_x = current_pose.pose.position.x;
  double current_y = current_pose.pose.position.y;

  for (size_t i = last_visited_index_; i < global_plan.poses.size(); ++i)
  {
    double dx = global_plan.poses[i].pose.position.x - current_x;
    double dy = global_plan.poses[i].pose.position.y - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance >= lookahead_distance_)
    {
      lookahead_point = global_plan.poses[i];
      last_visited_index_ = i;  // Update last visited index
      break;
    }
  }

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
