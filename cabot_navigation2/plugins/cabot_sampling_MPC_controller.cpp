#include "cabot_navigation2/cabot_sampling_MPC_controller.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <vector>
#include <cmath>
#include <limits>

namespace cabot_navigation2
{

Trajectory::Trajectory(
    geometry_msgs::msg::Twist control_,
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_)
: control(control_),
  trajectory(trajectory_)
{
}

CaBotSamplingMPCController::CaBotSamplingMPCController() {

}
CaBotSamplingMPCController::~CaBotSamplingMPCController() {
  
}

void CaBotSamplingMPCController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  costmap_ros_ = costmap_ros.get();  // Get pointer to the costmap
  name_ = name;
  tf_ = tf;

  // Load parameters
  declare_parameter_if_not_declared(
    node, name_ + ".group_topic", rclcpp::ParameterValue("/group_predictions"));
  node->get_parameter(name_ + ".group_topic", group_topic_);

  declare_parameter_if_not_declared(
    node, name_ + ".prediction_horizon", rclcpp::ParameterValue(4.8)); // seconds
  node->get_parameter(name_ + ".prediction_horizon", prediction_horizon_);

  declare_parameter_if_not_declared(
    node, name_ + ".sampling_rate", rclcpp::ParameterValue(0.4)); // seconds
  node->get_parameter(name_ + ".sampling_rate", sampling_rate_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_linear_velocity", rclcpp::ParameterValue(1.0)); // m/s
  node->get_parameter(name_ + ".max_linear_velocity", max_linear_velocity_);

  declare_parameter_if_not_declared(
    node, name_ + ".linear_sample_size", rclcpp::ParameterValue(3.0)); 
  node->get_parameter(name_ + ".linear_sample_size", linear_sample_size_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity", rclcpp::ParameterValue(M_PI / 6)); // rad/s
  node->get_parameter(name_ + ".max_angular_velocity", max_angular_velocity_);

  declare_parameter_if_not_declared(
    node, name_ + ".angular_sample_size", rclcpp::ParameterValue(3.0)); 
  node->get_parameter(name_ + ".angular_sample_size", angular_sample_size_);

  declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance", rclcpp::ParameterValue(0.5)); // meters
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);

  declare_parameter_if_not_declared(
    node, name_ + ".discount_factor", rclcpp::ParameterValue(0.9)); // Discount factor for future time steps
  node->get_parameter(name_ + ".discount_factor", discount_factor_);

  declare_parameter_if_not_declared(
    node, name_ + ".obstacle_costval", rclcpp::ParameterValue(250.0));
  node->get_parameter(name_ + ".obstacle_costval", obstacle_costval_);

  declare_parameter_if_not_declared(
    node, name_ + ".time_cost", rclcpp::ParameterValue(50.0));
  node->get_parameter(name_ + ".time_cost", time_cost_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_goal_dist", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + ".max_goal_dist", max_goal_dist_);

  linear_sample_resolution_ = max_linear_velocity_ / linear_sample_size_;
  angular_sample_resolution_ = max_angular_velocity_ * 2.0 / angular_sample_size_;

  last_visited_index_ = 0; // Initialize the last visited index to the start of the path

  // Subscribe to group trajectory prediction topic
  group_trajectory_sub_ = node->create_subscription<lidar_process_msgs::msg::GroupTimeArray>(
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

void CaBotSamplingMPCController::setPlan(const nav_msgs::msg::Path & path)
{
  auto node = node_.lock();
  // Transform global path into the robot's frame
  global_plan = path;
}

void CaBotSamplingMPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage)
  {
    max_linear_velocity_ *= speed_limit;
    max_angular_velocity_ *= speed_limit;
  } else {
    double ratio = speed_limit / max_linear_velocity_;
    max_linear_velocity_ = speed_limit;
    max_angular_velocity_ *= ratio;
  }
}

geometry_msgs::msg::TwistStamped CaBotSamplingMPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // This wrapper fucntion calls the function that computes the velocity commands

  auto node = node_.lock();

  geometry_msgs::msg::TwistStamped velocity_cmd;
  velocity_cmd.header.stamp = node->now();
  velocity_cmd.header.frame_id = "base_link";

  if (global_plan.poses.size() == 0)
  {
    return velocity_cmd;
  }

  // Call your MPC function to compute the optimal control action
  geometry_msgs::msg::Twist control_cmd = computeMPCControl(pose, velocity, global_plan);
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

  // Generate all the trajectories based on sampled velocities
  std::vector<Trajectory> trajectories = generateTrajectoriesSimple(pose, velocity);

  // Loop over the generated trajectories and calculate their costs
  for (const auto & trajectory : trajectories)
  {
    double cost = calculateCost(pose, trajectory, global_plan, local_goal);

    // Update the best control if this trajectory has a lower cost
    if (cost < min_cost)
    {
      min_cost = cost;
      best_control = trajectory.control;  // Assuming we store the control input corresponding to each trajectory
    }
  }

  return best_control;
}

std::vector<Trajectory> CaBotSamplingMPCController::generateTrajectoriesSimple(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  std::vector<Trajectory> trajectories;

  // Sample a set of velocities and predict the corresponding trajectories
  for (double linear_vel = 0.0; linear_vel <= max_linear_velocity_; linear_vel += linear_sample_resolution_)
  {
    for (double angular_vel = -max_angular_velocity_; angular_vel <= max_angular_velocity_; angular_vel += angular_sample_resolution_)
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

      // Store this trajectory
      trajectories.push_back(Trajectory(control, trajectory));
    }
  }

  return trajectories;
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
  const Trajectory trajectory,
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & local_goal)
{
  // Define a cost function to evaluate how good the trajectory is
  // This includes distances to the local goal, costmap information, and group trajectory cost

  double cost = 0.0;

  std::vector<geometry_msgs::msg::PoseStamped> sampled_trajectory = trajectory.trajectory;

  // Add costmap-related cost
  double discount = 1.0;
  double step_cost;
  double goal_dist;
  geometry_msgs::msg::PoseStamped final_pose;
  // if the trajectory pass through obstacle then it is invalid
  // if the trajectory pass through goal then trajectory is cut short.
  int trunc_length = 0;
  for (const auto & pose : sampled_trajectory)
  {
    step_cost = getCostFromCostmap(pose.pose);
    if (step_cost >= obstacle_costval_)
    {
      cost = std::numeric_limits<double>::infinity();
      // break;
      return cost;
    }
    cost += step_cost * discount;
    discount *= discount_factor_;
    final_pose = pose;
    // calculate goal dist at the same time
    goal_dist = pointDist(final_pose.pose.position, local_goal.pose.position);
    trunc_length += 1;
    cost += time_cost_;
    if (goal_dist <= lookahead_distance_)
    {
      break;
    }
  }
  cost += std::min(goal_dist / max_goal_dist_, 1.0) * 255.0;  // 255 if goal_dist > max_goal_dist_
  sampled_trajectory.resize(trunc_length);

  // Calculate the distance from the end of the sampled trajectory to the local goal
  // if (!sampled_trajectory.empty())
  // {
  //   // const auto & final_pose = sampled_trajectory.back();

  //   double goal_dist = pointDist(final_pose.pose.position, local_goal.pose.position);
  //   double goal_cost = std::min(goal_dist / 10.0, 1.0) * 255.0;  // 255 if goal_dist > 10

  //   cost += goal_cost;  // Accumulate the cost based on distance to the local goal
  // }

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
    if (t < group_trajectories_.size())
    {
      const geometry_msgs::msg::PoseStamped & robot_pose = sampled_trajectory[t];
      double discount = std::pow(discount_factor_, t);  // Apply discount factor for future time steps
      
      lidar_process_msgs::msg::GroupArray current_groups = group_trajectories_[t];

      // Compare the robot trajectory at time t with group trajectories at the same time
      for (const auto & group : current_groups.groups)
      {
        Safety::Point p1(group.left.x, group.left.y);
        Safety::Point p2(group.center.x, group.center.y);
        Safety::Point p3(group.right.x, group.right.y);
        Safety::Point p4(group.right_offset.x, group.right_offset.y);
        Safety::Point p5(group.left_offset.x, group.left_offset.y);

        Safety::Line l1(p1, p2);
        Safety::Line l2(p2, p3);
        Safety::Line l3(p3, p4);
        Safety::Line l4(p4, p5);
        Safety::Line l5(p5, p1);

        Safety::Point robot_point(robot_pose.pose.position.x, robot_pose.pose.position.y);
        Safety::Point closest_p1 = l1.closestPoint(robot_point);
        Safety::Point closest_p2 = l2.closestPoint(robot_point);
        Safety::Point closest_p3 = l3.closestPoint(robot_point);
        Safety::Point closest_p4 = l4.closestPoint(robot_point);
        Safety::Point closest_p5 = l5.closestPoint(robot_point);

        double d1 = pointDist(robot_point, closest_p1);
        double d2 = pointDist(robot_point, closest_p2);
        double d3 = pointDist(robot_point, closest_p3);
        double d4 = pointDist(robot_point, closest_p4);
        double d5 = pointDist(robot_point, closest_p5);

        double min_dist = std::min({d1, d2, d3, d4, d5});
        if (pointInPentagon(robot_point, p1, p2, p3, p4, p5))
        {
          min_dist = -min_dist;
        }

        // Add the discounted distance to the group trajectory to the cost
        group_cost += discount * std::exp(-min_dist) * 255.0;
      }
    }
  }

  return group_cost;
}

double CaBotSamplingMPCController::pointDist(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double CaBotSamplingMPCController::pointDist(
    const Safety::Point p1,
    const Safety::Point p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool CaBotSamplingMPCController::pointInPentagon(
  Safety::Point robot_point,
  Safety::Point p1,
  Safety::Point p2,
  Safety::Point p3,
  Safety::Point p4,
  Safety::Point p5)
{
  Safety::Line l1_1(robot_point, p1);
  Safety::Line l1_2(robot_point, p2);
  double cross_1 = l1_1.cross(l1_2);

  Safety::Line l2_1(robot_point, p2);
  Safety::Line l2_2(robot_point, p3);
  double cross_2 = l2_1.cross(l2_2);

  Safety::Line l3_1(robot_point, p3);
  Safety::Line l3_2(robot_point, p4);
  double cross_3 = l3_1.cross(l3_2);

  Safety::Line l4_1(robot_point, p4);
  Safety::Line l4_2(robot_point, p5);
  double cross_4 = l4_1.cross(l4_2);

  Safety::Line l5_1(robot_point, p5);
  Safety::Line l5_2(robot_point, p1);
  double cross_5 = l5_1.cross(l5_2);

  // if all the cross products have the same sign
  if (((cross_1 > 0) && (cross_2 > 0) && (cross_3 > 0) && (cross_4 > 0) && (cross_5 > 0)) ||
      ((cross_1 < 0) && (cross_2 < 0) && (cross_3 < 0) && (cross_4 < 0) && (cross_5 < 0)))
    {
      return true;
    } else {
      return false;
    }
}

}  // namespace cabot_navigation2

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotSamplingMPCController, nav2_core::Controller)
