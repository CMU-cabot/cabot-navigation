#ifndef CABOT_NAVIGATION2__CABOT_BLIND_CONTROLLER_HPP_
#define CABOT_NAVIGATION2__CABOT_BLIND_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "visualization_msgs/msg/marker.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace cabot_navigation2
{

/**
 * @brief Pure pursuit controller that follows the global plan and nothing else.
 *
 * This is the baseline for comparing the people-aware controllers against. It
 * reads no perception input at all: no costmap, no /people, no lidar_process.
 * It takes a lookahead point on the plan the planner gave it, drives at a
 * constant speed and steers towards that point.
 *
 * Not looking at obstacles is safe here only because stopping is somebody
 * else's job. cabot's safety chain sits downstream of every nav2 controller:
 *
 *   /cabot/cmd_vel_adapter -> speed_control_node -> /cabot/cmd_vel
 *
 * and lidar_speed_control_node publishes /cabot/lidar_speed (0.0 within
 * min_distance, i.e. a full stop) from /scan, low_lidar_speed_control_node the
 * same from the Livox scan. Those clamp whatever this controller emits. Do not
 * add obstacle handling here: it would be a second, competing source of truth.
 */
class CaBotBlindController : public nav2_core::Controller
{
public:
  CaBotBlindController();
  ~CaBotBlindController() override;

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

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & global_plan);

  void localGoalVisualizationCallback();

  rclcpp::Logger logger_ = rclcpp::get_logger("CaBotBlindController");
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;

  nav_msgs::msg::Path global_plan_;
  size_t last_visited_index_ = 0;

  // how far along the plan to aim
  double lookahead_distance_;
  // never aim further than this, so a long straight plan does not make the
  // heading error meaningless
  double max_lookahead_;
  // constant travel speed
  double max_linear_velocity_;
  // ceiling on the turn rate. With a constant linear speed this sets the
  // tightest turn the robot will make: r = max_linear_velocity / max_angular_velocity
  double max_angular_velocity_;
  // proportional gain from heading error to angular velocity
  double angular_gain_;
  // above this heading error, stop and turn on the spot instead of driving.
  // The rotation shim only does this once, before the plan starts; a plan that
  // doubles back would otherwise be driven forwards at full speed.
  double rotate_in_place_threshold_;
  // slow down linearly within this distance of the last point of the plan, so
  // the robot does not overshoot before the goal checker fires. Set to 0.0 to
  // keep the speed constant all the way.
  double goal_approach_distance_;

  // set by nav2 when a speed filter or the behavior tree limits the speed
  double speed_limit_ = 0.0;
  bool speed_limit_is_percentage_ = false;

  std::string local_goal_vis_topic_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_goal_vis_pub_;
  rclcpp::TimerBase::SharedPtr local_goal_vis_timer_;
  geometry_msgs::msg::PoseStamped curr_local_goal_;
  bool has_local_goal_ = false;
};

}  // namespace cabot_navigation2

#endif  // CABOT_NAVIGATION2__CABOT_BLIND_CONTROLLER_HPP_
