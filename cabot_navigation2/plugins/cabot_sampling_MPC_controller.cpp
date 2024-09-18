#include "cabot_navigation2/cabot_sampling_MPC_controller.hpp"
#include <vector>
#include <cmath>

namespace cabot_navigation2
{

void CaBotSamplingMPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = parent.lock();
  logger_ = node_->get_logger();
  costmap_ros_ = costmap_ros.get();  // Get pointer to the costmap

  // Load parameters
  declareParameter("group_topic", rclcpp::ParameterValue("/group_predictions"));
  node->get_parameter(name_ + "." + "group_topic", group_topic_);

  declareParameter("prediction_horizon", rclcpp::ParameterValue(4.8)); // seconds
  node->get_parameter(name_ + "." + "prediction_horizon", prediction_horizon_);

  declareParameter("sampling_rate", rclcpp::ParameterValue(0.4)); // seconds
  node->get_parameter(name_ + "." + "sampling_rate", sampling_rate_);

  declareParameter("max_linear_velocity", rclcpp::ParameterValue(1.0)); // m/s
  node->get_parameter(name_ + "." + "max_linear_velocity", max_linear_velocity_);

  declareParameter("max_angular_velocity", rclcpp::ParameterValue(M_PI / 6)); // rad/s
  node->get_parameter(name_ + "." + "max_angular_velocity", max_angular_velocity_);

  declareParameter("lookahead_distance", rclcpp::ParameterValue(0.5)); // meters
  node->get_parameter(name_ + "." + "lookahead_distance", lookahead_distance_);

  declareParameter("discount_factor", rclcpp::ParameterValue(0.9)); // Discount factor for future time steps
  node->get_parameter(name_ + "." + "discount_factor", discount_factor_);

  last_visited_index_ = 0; // Initialize the last visited index to the start of the path

  // Subscribe to group trajectory prediction topic
  group_trajectory_sub_ = node_->create_subscription<lidar_process_msgs::msg::GroupTimeArray>(
      group_topic_, 10, std::bind(&CaBotSamplingMPCController::groupPredictionCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "MPC controller configured with prediction horizon: %.2f, sampling rate: %.2f",
    prediction_horizon_, sampling_rate_);
}

void CaBotSamplingMPCController::groupPredictionCallback(const lidar_process_msgs::msg::GroupTimeArray::SharedPtr group)
{
  // Group sequences: First time, then groups
  auto node = node_.lock();
  group_trajectories_ = group->group_sequences;
}

void CaBotSamplingMPCController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up MPC controller");
}

void CaBotSamplingMPCController::activate()
{
  RCLCPP_INFO(logger_, "Activating MPC controller");
}

void CaBotSamplingMPCController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating MPC controller");
}

geometry_msgs::msg::TwistStamped CaBotSamplingMPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav_msgs::msg::Path & global_plan)
{
  // This wrapper fucntion calls the function that computes the velocity commands

  auto node = node_.lock();

  // Call your MPC function to compute the optimal control action
  geometry_msgs::msg::Twist control_cmd = computeMPCControl(pose, velocity, global_plan);

  geometry_msgs::msg::TwistStamped velocity_cmd;
  velocity_cmd.header.stamp = node->now();
  velocity_cmd.header.frame_id = "base_link";
  velocity_cmd.twist = control_cmd;

  return velocity_cmd;
}

geometry_msgs::msg::Twist CaBotSamplingMPCController::computeMPCControl(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav_msgs::msg::Path & global_plan)
{
  // This function samples the MPC trajectories and computes the costs of the trajectories
  // The cost with the lowest trajectory will be selected and the associated velocities returned.

  geometry_msgs::msg::Twist best_control;
  double min_cost = std::numeric_limits<double>::max();

  // Get the local goal
  geometry_msgs::msg::PoseStamped local_goal = getLookaheadPoint(pose, global_plan);

  // Sample a set of velocities and predict the corresponding trajectories
  for (double linear_vel = 0.0; linear_vel <= max_linear_velocity_; linear_vel += 0.1)
  {
    for (double angular_vel = -max_angular_velocity_; angular_vel <= max_angular_velocity_; angular_vel += 0.1)
    {
      // Generate a predicted trajectory
      std::vector<geometry_msgs::msg::PoseStamped> trajectory;

      // Start with the current pose
      geometry_msgs::msg::PoseStamped current_pose = pose;
      double current_x = current_pose.pose.position.x;
      double current_y = current_pose.pose.position.y;
      double current_theta = tf2::getYaw(current_pose.pose.orientation);

      // Predict the trajectory over the prediction horizon
      for (double t = 0; t <= prediction_horizon_; t += sampling_rate_)
      {
        // Simulate robot dynamics
        current_x += linear_vel * sampling_rate_ * cos(current_theta);
        current_y += linear_vel * sampling_rate_ * sin(current_theta);
        current_theta += angular_vel * sampling_rate_;

        geometry_msgs::msg::PoseStamped predicted_pose;
        predicted_pose.pose.position.x = current_x;
        predicted_pose.pose.position.y = current_y;
        tf2::Quaternion q;
        q.setRPY(0, 0, current_theta);
        predicted_pose.pose.orientation = tf2::toMsg(q);

        trajectory.push_back(predicted_pose);
      }

      // Calculate the cost for this trajectory
      double cost = calculateCost(pose, trajectory, global_plan, local_goal);

      // Update the best control if this trajectory has a lower cost
      if (cost < min_cost)
      {
        min_cost = cost;
        best_control.linear.x = linear_vel;
        best_control.angular.z = angular_vel;
      }
    }
  }

  return best_control;
}

geometry_msgs::msg::PoseStamped CaBotSamplingMPCController::getLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const nav_msgs::msg::Path & global_plan)
{
  // This function gets the immediate next point outside of threhsold as the goal point
  // on the global plan

  geometry_msgs::msg::PoseStamped lookahead_point;

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

  return lookahead_point;
}

bool CaBotSamplingMPCController::hasReachedLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & lookahead_point)
{
  // This function checks if the robot's pose is within threshold dist of a point

  double dx = current_pose.pose.position.x - lookahead_point.pose.position.x;
  double dy = current_pose.pose.position.y - lookahead_point.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  return distance <= lookahead_distance_;
}

double CaBotSamplingMPCController::calculateCost(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::vector<geometry_msgs::msg::PoseStamped> & sampled_trajectory,
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & local_goal)
{
  // Define a cost function to evaluate how good the trajectory is
  // This includes distances to the local goal, costmap information, and group trajectory cost

  double cost = 0.0;

  // Calculate the distance from the end of the sampled trajectory to the local goal
  if (!sampled_trajectory.empty())
  {
    const auto & final_pose = sampled_trajectory.back();

    double dx = final_pose.pose.position.x - local_goal.pose.position.x;
    double dy = final_pose.pose.position.y - local_goal.pose.position.y;
    double goal_dist = std::sqrt(dx * dx + dy * dy);

    cost += goal_dist;  // Accumulate the cost based on distance to the local goal
  }

  // Add costmap-related cost
  for (const auto & pose : sampled_trajectory)
  {
    cost += getCostFromCostmap(pose.pose);
  }

  // Add group trajectory-related cost
  cost += calculateGroupTrajectoryCost(sampled_trajectory);

  return cost;
}

double CaBotSamplingMPCController::getCostFromCostmap(const geometry_msgs::msg::Pose & pose)
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

double CaBotSamplingMPCController::calculateGroupTrajectoryCost(
  const std::vector<geometry_msgs::msg::PoseStamped> & sampled_trajectory)
{
  // This function estimates the cost of a trajectory against the predicted group trajectories

  double group_cost = 0.0;
  size_t num_time_steps = sampled_trajectory.size();

  // Iterate over each time step in the sampled trajectory
  for (size_t t = 0; t < num_time_steps; ++t)
  {
    const geometry_msgs::msg::PoseStamped & robot_pose = sampled_trajectory[t];
    double discount = std::pow(discount_factor_, t);  // Apply discount factor for future time steps

    // Compare the robot trajectory at time t with group trajectories at the same time
    for (const auto & group_traj : group_trajectories_)
    {
      if (t < group_traj.poses.size())  // Make sure the group trajectory has enough points
      {
        const geometry_msgs::msg::Pose & group_pose = group_traj.poses[t];

        double dx = robot_pose.pose.position.x - group_pose.position.x;
        double dy = robot_pose.pose.position.y - group_pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Add the discounted distance to the group trajectory to the cost
        group_cost += discount * distance;
      }
    }
  }

  return group_cost;
}

}  // namespace cabot_navigation2

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotSamplingMPCController, nav2_core::Controller)
