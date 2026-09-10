#ifndef CABOT_DNN_CONTROLLER__DNN_CONTROLLER_HPP_
#define CABOT_DNN_CONTROLLER__DNN_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <NvInfer.h>

#include "cabot_dnn_controller/dnn_controller_constants.hpp"
#include "cabot_dnn_controller/msg/attention_weights.hpp"
#include "cabot_dnn_controller/tensorrt_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "people_msgs/msg/people.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_event_handler.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

namespace cabot_dnn_controller
{

class TrtLogger : public nvinfer1::ILogger
{
public:
  explicit TrtLogger(rclcpp::Logger ros_logger)
  : ros_logger_(ros_logger) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity == Severity::kVERBOSE) return;

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
      case Severity::kERROR:
        RCLCPP_ERROR(ros_logger_, "[TensorRT] %s", msg);
        break;
      case Severity::kWARNING:
        RCLCPP_WARN(ros_logger_, "[TensorRT] %s", msg);
        break;
      case Severity::kINFO:
        RCLCPP_INFO(ros_logger_, "[TensorRT] %s", msg);
        break;
      default:
        RCLCPP_DEBUG(ros_logger_, "[TensorRT] %s", msg);
        break;
    }
  }

private:
  rclcpp::Logger ros_logger_;
};

struct TrtDeleter
{
  void operator()(nvinfer1::IRuntime * runtime) const
  {
    if (runtime) {
      delete runtime;
    }
  }

  void operator()(nvinfer1::ICudaEngine * engine) const
  {
    if (engine) {
      delete engine;
    }
  }

  void operator()(nvinfer1::IExecutionContext * engine) const
  {
    if (engine) {
      delete engine;
    }
  }
};

class DnnController : public nav2_core::Controller
{
public:
  DnnController() = default;
  ~DnnController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
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
  struct PlanarVelocity
  {
    float vx{0.0f};
    float wz{0.0f};
  };

  struct PeopleHistoryRecord
  {
    std::int64_t stamp_ns{0};
    std::array<float, 2> position{0.0f, 0.0f};
    std::array<float, 2> velocity{0.0f, 0.0f};
    float presence{0.0f};
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void peopleCallback(const people_msgs::msg::People::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult param_set_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  void updatePeopleHistoryLocked(std::int64_t stamp_ns);
  std::vector<float> buildPeopleInput(
    const geometry_msgs::msg::TransformStamped & tf_base_link_map,
    const rclcpp::Time & current_time,
    const std::unordered_map<std::string, std::deque<PeopleHistoryRecord>> &
    people_history);
  std::vector<std::array<float, 2>> transformPoints2D(
    const std::vector<std::array<float, 2>> & points,
    const geometry_msgs::msg::TransformStamped & tf) const;
  void publishAttention(
    const std::vector<float> & people_attention,
    const std::vector<float> & robot_people_attention,
    const std_msgs::msg::Header & header) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("dnn_controller")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<PlanarVelocity> odom_history_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  std::unordered_map<std::string, PeopleHistoryRecord> latest_people_;
  bool has_people_observation_{false};
  std::unordered_map<std::string, std::deque<PeopleHistoryRecord>> people_history_;
  rclcpp::AsyncParametersClient::SharedPtr offset_sign_client_;
  std::shared_ptr<rclcpp::ParameterEventHandler> offset_sign_event_handler_;
  rclcpp::ParameterCallbackHandle::SharedPtr offset_sign_callback_handle_;
  rclcpp::TimerBase::SharedPtr offset_sign_request_timer_;
  std::atomic<float> offset_sign_{0.0f};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<cabot_dnn_controller::msg::AttentionWeights>::SharedPtr people_attention_pub_;
  rclcpp::Publisher<cabot_dnn_controller::msg::AttentionWeights>::SharedPtr
    robot_people_attention_pub_;

  std::string trt_model_;
  double max_linear_vel_;
  double max_angular_vel_;
  std::atomic<bool> velocity_parameters_dirty_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_change_callback_handle_;
  double transform_tolerance_;
  std::string base_link_frame_;
  std::string map_frame_;
  std::string scan_frame_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string people_topic_;
  std::string debug_image_topic_;
  std::string people_attention_topic_;
  std::string robot_people_attention_topic_;
  std::unique_ptr<TrtLogger> trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDeleter> trt_runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter> trt_engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter> trt_context_;
  cudaStream_t trt_stream_{nullptr};
  bool trt_ready_{false};
  void* d_odom_{nullptr};
  void* d_plan_{nullptr};
  void* d_scan_{nullptr};
  void* d_offset_sign_{nullptr};
  void* d_people_{nullptr};
  void* d_cmd_{nullptr};
  void* d_v_logits_{nullptr};
  void* d_w_logits_{nullptr};
  void* d_people_attention_{nullptr};
  void* d_robot_people_attention_{nullptr};
  dnn_controller_constants::ActionMode action_mode_;
  int odom_length_;
  int plan_length_;
  bool people_encoder_enabled_{false};
  bool input_velocity_{false};
  int num_people_{0};
  int people_history_length_{0};
  int people_dim_{dnn_controller_constants::kPeopleDim};
  int presence_index_{dnn_controller_constants::kPeopleDim - 1};
  int num_attention_heads_{0};
  int v_num_bins_;
  int w_num_bins_;
  std::mutex trt_mutex_;
  std::mutex plan_mutex_;
  std::mutex odom_mutex_;
  std::mutex scan_mutex_;
  std::mutex people_mutex_;
};

}  // namespace cabot_dnn_controller

#endif  // CABOT_DNN_CONTROLLER__DNN_CONTROLLER_HPP_
