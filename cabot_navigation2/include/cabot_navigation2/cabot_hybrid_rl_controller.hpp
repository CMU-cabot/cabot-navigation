#ifndef CABOT_NAVIGATION2__CABOT_HYBRID_RL_CONTROLLER_HPP_
#define CABOT_NAVIGATION2__CABOT_HYBRID_RL_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// #include "lidar_process_msgs/srv/rl_action.hpp"
#include "lidar_process_msgs/msg/robot_message.hpp"
#include "lidar_process_msgs/msg/position_array.hpp"
#include "lidar_process_msgs/msg/position_history_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cabot_navigation2/util.hpp>

using nav2_util::declare_parameter_if_not_declared;

namespace cabot_navigation2
{

struct Trajectory
{
  bool initialized;
  geometry_msgs::msg::Twist control;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory;
  Trajectory();
  Trajectory(
    geometry_msgs::msg::Twist control,
    std::vector<geometry_msgs::msg::PoseStamped> trajectory
  );
};

class CaBotHybridRLController : public nav2_core::Controller     
{
public:
  CaBotHybridRLController();
  ~CaBotHybridRLController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  // bool cancel()
  // {
  //   return true;
  // }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // void reset() {}

private:
  friend class HybridControllerTestPeer;
  std::mutex state_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::vector<std::pair<std::string, double *>> parameterBindings();
  std::vector<rclcpp::Parameter> readParameters();
  void applyParameters(const std::vector<rclcpp::Parameter> & parameters);
  rcl_interfaces::msg::SetParametersResult validateParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  int configure_count = 0;
  rclcpp::Logger logger_ = rclcpp::get_logger("CaBotHybridRLController");
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string rl_people_topic_;
  std::string rl_subgoal_topic_;
  std::string rl_info_topic_;
  double prediction_horizon_;
  double sampling_rate_;
  double max_linear_velocity_;
  double linear_sample_size_;
  double max_angular_velocity_;
  double angular_sample_size_;
  double discount_factor_;  // Discount factor for future time steps
  double obstacle_costval_;
  double collision_radius_;
  nav2_costmap_2d::Costmap2DROS *costmap_ros_;  // Pointer to the costmap
  rclcpp::Clock::SharedPtr clock_;

  double goal_cost_wt_;
  double people_cost_wt_;
  double heading_cost_wt_;
  double angular_cost_wt_;
  double avoidance_max_angle_;
  double avoidance_angular_velocity_;
  double avoidance_probe_distance_;
  double avoidance_min_clearance_;
  int avoidance_side_ = 0;

  nav_msgs::msg::Path global_plan_;

  int last_visited_index_; // Keep track of the last visited point in the global plan
  geometry_msgs::msg::PoseStamped curr_local_goal_;
  lidar_process_msgs::msg::RobotMessage robot_info;

  double lookahead_distance_;
  double max_lookahead_;
  double focus_goal_dist_;

  std::string loc_goal_vis_topic_;
  rclcpp::TimerBase::SharedPtr loc_goal_vis_timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_goal_visualization_pub_;
  void localGoalVisualizationCallback();

  std::string traj_vis_topic_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_visualization_pub_;

  // rclcpp::Client<lidar_process_msgs::srv::RlAction>::SharedPtr rl_client;
  geometry_msgs::msg::Twist current_command;
  std::atomic<bool> rl_ready_{false};
  geometry_msgs::msg::Point rl_subgoal_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rl_subgoal_sub_;
  void rlSubgoalCallback(const geometry_msgs::msg::Point::SharedPtr rl_subgoal);

  int horizon_people_ = 0;
  int num_people_ = 0;
  std::vector<lidar_process_msgs::msg::PositionArray> rl_people_;
  std::vector<lidar_process_msgs::msg::PositionArray> rl_people_tmp_;
  rclcpp::Subscription<lidar_process_msgs::msg::PositionHistoryArray>::SharedPtr rl_people_sub_;  // group prediction subscriber
  void rlPeopleCallback(const lidar_process_msgs::msg::PositionHistoryArray::SharedPtr rl_people);

  rclcpp::TimerBase::SharedPtr rl_info_pub_timer_;
  rclcpp::Publisher<lidar_process_msgs::msg::RobotMessage>::SharedPtr rl_info_pub_;
  void rlInfoCallback();

  geometry_msgs::msg::Twist computeMPCControl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  bool chooseAvoidanceTurn(
    const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::Twist & control, nav_msgs::msg::Path & trajectory);
  
  std::vector<Trajectory> generateTrajectoriesSimple(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & velocity);

  std::vector<Trajectory> generateTrajectoriesImproved(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & velocity);

  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & global_plan);

  bool hasReachedLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);

  double calculateCost(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const Trajectory sampled_trajectory);

  double getCostFromCostmap(
    const geometry_msgs::msg::Pose & pose);

  double calculatePeopleCost(
    const std::vector<geometry_msgs::msg::PoseStamped> & sampled_trajectory);

  double pointDist(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

};

} // namespace cabot_navigation2

#endif // CABOT_NAVIGATION2__CABOT_HYBRID_RL_CONTROLLER_HPP_
