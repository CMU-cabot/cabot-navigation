#include "cabot_dnn_controller/dnn_controller.hpp"

#include <filesystem>
#include <fstream>
#include <cstdio>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
  #include <cv_bridge/cv_bridge.h>
#else
  #error "cv_bridge header not found: expected cv_bridge/cv_bridge.hpp or cv_bridge/cv_bridge.h"
#endif
#include <NvInferPlugin.h>
#include <yaml-cpp/yaml.h>

#include "cabot_dnn_controller/classify_utils.hpp"
#include "cabot_dnn_controller/dnn_controller_constants.hpp"
#include "cabot_dnn_controller/tensorrt_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cabot_dnn_controller
{

namespace
{

using cabot_dnn_controller::dnn_controller_constants::ActionMode;

using cabot_dnn_controller::dnn_controller_constants::kScanLength;
using cabot_dnn_controller::dnn_controller_constants::kScanRangeMax;
using cabot_dnn_controller::dnn_controller_constants::kPeopleDim;

using cabot_dnn_controller::dnn_controller_constants::kVMin;
using cabot_dnn_controller::dnn_controller_constants::kVMax;
using cabot_dnn_controller::dnn_controller_constants::kWMin;
using cabot_dnn_controller::dnn_controller_constants::kWMax;

using cabot_dnn_controller::dnn_controller_constants::kInputOdomName;
using cabot_dnn_controller::dnn_controller_constants::kInputPlanName;
using cabot_dnn_controller::dnn_controller_constants::kInputScanName;
using cabot_dnn_controller::dnn_controller_constants::kInputPeopleName;
using cabot_dnn_controller::dnn_controller_constants::kOutputCmdName;
using cabot_dnn_controller::dnn_controller_constants::kOutputVLogitsName;
using cabot_dnn_controller::dnn_controller_constants::kOutputWLogitsName;

void initTensorRTPluginsOnce(nvinfer1::ILogger & logger)
{
  static std::once_flag init_flag;
  static bool initialized = false;

  std::call_once(init_flag, [&]() {
    // Required for TensorRT builtin plugins
    initialized = initLibNvInferPlugins(&logger, "");
  });

  if (!initialized) {
    throw std::runtime_error("Failed to initialize TensorRT plugins");
  }
}

}  // namespace

void DnnController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock node in DnnController::configure");
  }
  name_ = std::move(name);
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  const std::string prefix = name_ + ".";
  node_->declare_parameter<std::string>(prefix + "trt_model", "");
  node_->get_parameter(prefix + "trt_model", trt_model_);
  node_->declare_parameter<double>(prefix + "max_linear_vel", 1.25);
  node_->get_parameter(prefix + "max_linear_vel", max_linear_vel_);
  node_->declare_parameter<double>(prefix + "max_angular_vel", 1.6);
  node_->get_parameter(prefix + "max_angular_vel", max_angular_vel_);
  node_->declare_parameter<double>(prefix + "transform_tolerance", 0.1);
  node_->get_parameter(prefix + "transform_tolerance", transform_tolerance_);
  node_->declare_parameter<std::string>(prefix + "base_link_frame", "");
  node_->get_parameter(prefix + "base_link_frame", base_link_frame_);
  node_->declare_parameter<std::string>(prefix + "map_frame", "");
  node_->get_parameter(prefix + "map_frame", map_frame_);
  node_->declare_parameter<std::string>(prefix + "scan_frame", "");
  node_->get_parameter(prefix + "scan_frame", scan_frame_);
  node_->declare_parameter<std::string>(prefix + "odom_topic", "");
  node_->get_parameter(prefix + "odom_topic", odom_topic_);
  node_->declare_parameter<std::string>(prefix + "scan_topic", "");
  node_->get_parameter(prefix + "scan_topic", scan_topic_);
  node_->declare_parameter<std::string>(prefix + "people_topic", "people");
  node_->get_parameter(prefix + "people_topic", people_topic_);
  node_->declare_parameter<std::string>(prefix + "debug_image_topic", "");
  node_->get_parameter(prefix + "debug_image_topic", debug_image_topic_);

  if (!trt_model_.empty()) {
    const std::filesystem::path model_path(trt_model_);
    if (!std::filesystem::exists(model_path)) {
      throw std::runtime_error("TensorRT model file not found: " + trt_model_);
    }

    const std::filesystem::path config_path = model_path.parent_path() / "config.yaml";
    if (!std::filesystem::exists(config_path)) {
      throw std::runtime_error("Model config file not found: " + config_path.string());
    }
    const YAML::Node config = YAML::LoadFile(config_path.string());
    odom_length_ = config["odom_encoder"]["odom_length"].as<int>();
    plan_length_ = config["plan_encoder"]["plan_length"].as<int>();
    people_encoder_enabled_ = false;
    num_people_ = 0;
    const YAML::Node people_config = config["people_encoder"];
    if (people_config && people_config["enabled"]) {
      people_encoder_enabled_ = people_config["enabled"].as<bool>();
    }
    if (people_encoder_enabled_) {
      if (!people_config["num_people"]) {
        throw std::runtime_error("people_encoder.num_people is required when people_encoder is enabled");
      }
      num_people_ = people_config["num_people"].as<int>();
      if (num_people_ <= 0) {
        throw std::runtime_error("people_encoder.num_people must be > 0");
      }
    }
    std::string action_mode_str = config["action"]["mode"].as<std::string>();
    if (action_mode_str == "reg") {
      action_mode_ = ActionMode::kReg;
    } else if (action_mode_str == "cls") {
      action_mode_ = ActionMode::kCls;
    } else if (action_mode_str == "cls-reg") {
      action_mode_ = ActionMode::kClsReg;
    } else if (action_mode_str == "mdn-reg") {
      action_mode_ = ActionMode::kMdnReg;
    } else {
      throw std::runtime_error("Invalid action mode: " + action_mode_str);
    }
    if ((action_mode_ == ActionMode::kCls) || (action_mode_ == ActionMode::kClsReg)) {
      v_num_bins_ = config["action"]["v_num_bins"].as<int>();
      w_num_bins_ = config["action"]["w_num_bins"].as<int>();
    }

    std::ifstream ifs(trt_model_, std::ios::binary);
    if (!ifs) {
      throw std::runtime_error("Failed to open TensorRT model file: " + trt_model_);
    }
    std::vector<char> trt_engine_data((std::istreambuf_iterator<char>(ifs)), {});
    if (trt_engine_data.empty()) {
      throw std::runtime_error("TensorRT model file is empty: " + trt_model_);
    }
    trt_logger_ = std::make_unique<TrtLogger>(logger_);
    // Required before deserializing engines that contain TensorRT plugins
    initTensorRTPluginsOnce(*trt_logger_);
    trt_runtime_.reset(nvinfer1::createInferRuntime(*trt_logger_));
    if (!trt_runtime_) {
      throw std::runtime_error("Failed to create TensorRT runtime");
    }
    trt_engine_.reset(trt_runtime_->deserializeCudaEngine(trt_engine_data.data(), trt_engine_data.size()));
    if (!trt_engine_) {
      throw std::runtime_error("Failed to deserialize TensorRT engine: " + trt_model_);
    }
    RCLCPP_INFO(logger_, "Loaded TensorRT engine: %s (%zu bytes, %d io tensors)",
      trt_model_.c_str(), trt_engine_data.size(), trt_engine_->getNbIOTensors());

    trt_context_.reset(trt_engine_->createExecutionContext());
    if (!trt_context_) {
      throw std::runtime_error("Failed to create TensorRT execution context");
    }

    CUDA_CHECK(cudaStreamCreate(&trt_stream_));

    bool input_shapes_set =
      trt_context_->setInputShape(kInputOdomName, nvinfer1::Dims3{1, odom_length_, 2}) &&
      trt_context_->setInputShape(kInputPlanName, nvinfer1::Dims3{1, plan_length_, 2}) &&
      trt_context_->setInputShape(kInputScanName, nvinfer1::Dims2{1, kScanLength});
    if (people_encoder_enabled_) {
      input_shapes_set = input_shapes_set &&
        trt_context_->setInputShape(kInputPeopleName, nvinfer1::Dims3{1, num_people_, kPeopleDim});
    }
    if (!input_shapes_set) {
      throw std::runtime_error("Failed to set TensorRT input shapes");
    }

    auto allocTensor = [&](const char * name) -> void * {
      const nvinfer1::Dims dims = trt_context_->getTensorShape(name);
      const nvinfer1::DataType dt = trt_engine_->getTensorDataType(name);
      const size_t nbytes = tensorrt_utils::bytesFor(dims, dt);
      void * ptr = nullptr;
      cudaError_t status = cudaMalloc(&ptr, nbytes);
      if (status != cudaSuccess) {
        RCLCPP_ERROR(logger_, "cudaMalloc failed for %s: %s", name, cudaGetErrorString(status));
        std::fprintf(stderr, "[dnn_controller] cudaMalloc failed for %s: %s\n", name, cudaGetErrorString(status));
        std::fflush(stderr);
        ptr = nullptr;
      } else {
        RCLCPP_INFO(logger_, "Allocated %s: shape=[%d dims], bytes=%zu", name, dims.nbDims, nbytes);
      }
      return ptr;
    };

    d_odom_ = allocTensor(kInputOdomName);
    d_plan_ = allocTensor(kInputPlanName);
    d_scan_ = allocTensor(kInputScanName);
    if (people_encoder_enabled_) {
      d_people_ = allocTensor(kInputPeopleName);
    }
    d_cmd_  = allocTensor(kOutputCmdName);
    d_v_logits_  = allocTensor(kOutputVLogitsName);
    d_w_logits_  = allocTensor(kOutputWLogitsName);

    bool tensor_addresses_set =
      trt_context_->setTensorAddress(kInputOdomName, d_odom_) &&
      trt_context_->setTensorAddress(kInputPlanName, d_plan_) &&
      trt_context_->setTensorAddress(kInputScanName, d_scan_) &&
      trt_context_->setTensorAddress(kOutputCmdName, d_cmd_) &&
      trt_context_->setTensorAddress(kOutputVLogitsName, d_v_logits_) &&
      trt_context_->setTensorAddress(kOutputWLogitsName, d_w_logits_);
    if (people_encoder_enabled_) {
      tensor_addresses_set = tensor_addresses_set &&
        trt_context_->setTensorAddress(kInputPeopleName, d_people_);
    }
    if (!tensor_addresses_set) {
      throw std::runtime_error("Failed to set TensorRT tensor addresses");
    }

    trt_ready_ = true;
    RCLCPP_INFO(logger_, "TensorRT engine is ready");
  } else {
    trt_ready_ = false;
    RCLCPP_WARN(logger_, "trt_model is empty; skipping TensorRT engine load");
  }

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DnnController::scanCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DnnController::odomCallback, this, std::placeholders::_1));
  if (people_encoder_enabled_) {
    people_sub_ = node_->create_subscription<people_msgs::msg::People>(
      people_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DnnController::peopleCallback, this, std::placeholders::_1));
  }
  debug_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SystemDefaultsQoS());
}

void DnnController::cleanup()
{
  {
    std::lock_guard<std::mutex> plan_lock(plan_mutex_);
    global_plan_.poses.clear();
  }
  odom_sub_.reset();
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    odom_history_.clear();
  }
  scan_sub_.reset();
  {
    std::lock_guard<std::mutex> scan_lock(scan_mutex_);
    last_scan_.reset();
  }
  people_sub_.reset();
  {
    std::lock_guard<std::mutex> people_lock(people_mutex_);
    last_people_.reset();
  }

  trt_ready_ = false;
  if (d_odom_) { CUDA_CHECK(cudaFree(d_odom_)); d_odom_ = nullptr; }
  if (d_plan_) { CUDA_CHECK(cudaFree(d_plan_)); d_plan_ = nullptr; }
  if (d_scan_) { CUDA_CHECK(cudaFree(d_scan_)); d_scan_ = nullptr; }
  if (d_people_) { CUDA_CHECK(cudaFree(d_people_)); d_people_ = nullptr; }
  if (d_cmd_)  { CUDA_CHECK(cudaFree(d_cmd_));  d_cmd_  = nullptr; }
  if (d_w_logits_)  { CUDA_CHECK(cudaFree(d_w_logits_));  d_w_logits_  = nullptr; }
  if (d_v_logits_)  { CUDA_CHECK(cudaFree(d_v_logits_));  d_v_logits_  = nullptr; }
  if (trt_stream_) { CUDA_CHECK(cudaStreamDestroy(trt_stream_)); trt_stream_ = nullptr; }
  trt_context_.reset();
  trt_engine_.reset();
  trt_runtime_.reset();
  trt_logger_.reset();

  costmap_ros_.reset();
  tf_.reset();
  clock_.reset();
  node_.reset();
  people_encoder_enabled_ = false;
  num_people_ = 0;
}

void DnnController::activate()
{
  // No publishers to activate in skeleton.
}

void DnnController::deactivate()
{
  // No publishers to deactivate in skeleton.
}

std::vector<std::array<float, 2>> DnnController::transformPoints2D(
  const std::vector<std::array<float, 2>> & points,
  const geometry_msgs::msg::TransformStamped & tf) const
{
  const auto & t = tf.transform.translation;
  const auto & r = tf.transform.rotation;
  const double yaw = std::atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z));
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  std::vector<std::array<float, 2>> output_points;
  output_points.reserve(points.size());
  for (const auto & point : points) {
    output_points.push_back({
      static_cast<float>(t.x + cy * point[0] - sy * point[1]),
      static_cast<float>(t.y + sy * point[0] + cy * point[1]),
    });
  }
  return output_points;
}

std::vector<float> DnnController::buildPeopleInput()
{
  std::vector<float> h_people(static_cast<size_t>(num_people_) * kPeopleDim, 0.0f);
  if (!people_encoder_enabled_ || num_people_ <= 0) {
    return h_people;
  }

  people_msgs::msg::People::SharedPtr people_msg;
  {
    std::lock_guard<std::mutex> people_lock(people_mutex_);
    people_msg = last_people_;
  }
  if (!people_msg || people_msg->people.empty()) {
    return h_people;
  }

  std::string source_frame = people_msg->header.frame_id;
  if (source_frame.empty()) {
    source_frame = map_frame_;
  }
  if (source_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "Ignoring people message because header.frame_id is empty and map_frame is not set");
    return h_people;
  }

  std::vector<std::array<float, 2>> source_people;
  source_people.reserve(people_msg->people.size());
  for (const auto & person : people_msg->people) {
    const float x = static_cast<float>(person.position.x);
    const float y = static_cast<float>(person.position.y);
    if (std::isfinite(x) && std::isfinite(y)) {
      source_people.push_back({x, y});
    }
  }
  if (source_people.empty()) {
    return h_people;
  }

  std::vector<std::array<float, 2>> base_people;
  if (source_frame == base_link_frame_) {
    base_people = std::move(source_people);
  } else {
    geometry_msgs::msg::TransformStamped tf_base_link_people;
    try {
      const rclcpp::Duration tf_timeout = rclcpp::Duration::from_seconds(transform_tolerance_);
      const bool has_stamp =
        people_msg->header.stamp.sec != 0 || people_msg->header.stamp.nanosec != 0;
      const rclcpp::Time lookup_time = has_stamp ?
        rclcpp::Time(people_msg->header.stamp) :
        rclcpp::Time(0, 0, clock_->get_clock_type());
      tf_base_link_people = tf_->lookupTransform(
        base_link_frame_, source_frame, lookup_time, tf_timeout);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "Ignoring people message because transform from '%s' to '%s' is unavailable: %s",
        source_frame.c_str(), base_link_frame_.c_str(), ex.what());
      return h_people;
    }
    base_people = transformPoints2D(source_people, tf_base_link_people);
  }

  std::vector<size_t> order(base_people.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](size_t lhs, size_t rhs) {
    const auto & l = base_people[lhs];
    const auto & r = base_people[rhs];
    return (l[0] * l[0] + l[1] * l[1]) < (r[0] * r[0] + r[1] * r[1]);
  });

  const size_t output_count = std::min(order.size(), static_cast<size_t>(num_people_));
  for (size_t i = 0; i < output_count; ++i) {
    const auto & person = base_people[order[i]];
    h_people[i * kPeopleDim + 0] = person[0];
    h_people[i * kPeopleDim + 1] = person[1];
    h_people[i * kPeopleDim + 2] = 1.0f;
  }

  return h_people;
}

geometry_msgs::msg::TwistStamped DnnController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped &,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = base_link_frame_;
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;

  if (!trt_ready_) {
    RCLCPP_INFO(logger_, "TensorRT is not ready, return 0 velocity command");
    return cmd;
  }

  std::vector<std::array<float, 2>> odom_history;
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    odom_history = odom_history_;
  }

  sensor_msgs::msg::LaserScan::SharedPtr scan_msg;
  {
    std::lock_guard<std::mutex> scan_lock(scan_mutex_);
    scan_msg = last_scan_;
  }

  if (odom_history.empty() || !scan_msg) {
    RCLCPP_INFO(logger_, "odom or scan is not ready, return 0 velocity command");
    return cmd;
  }

  if (odom_history.size() != odom_length_) {
    RCLCPP_ERROR(logger_, "odom size is not correct, return 0 velocity command, input size=%ld, expected size=%ld",
      odom_history.size(), odom_length_);
    return cmd;
  }
  if (scan_msg->ranges.size() != kScanLength) {
    RCLCPP_ERROR(logger_, "scan size is not correct, return 0 velocity command, input size=%ld, expected size=%ld",
      scan_msg->ranges.size(), kScanLength);
    return cmd;
  }

  std::vector<float> h_odom(1 * odom_length_ * 2, 0.0f);
  for (size_t i = 0; i < odom_length_; i++) {
    h_odom[i * 2 + 0] = odom_history[i][0] / static_cast<float>(max_linear_vel_);
    h_odom[i * 2 + 1] = odom_history[i][1] / static_cast<float>(max_angular_vel_);
  }

  if (!tf_) {
    RCLCPP_ERROR(logger_, "tf buffer is not available, return 0 velocity command");
    return cmd;
  }

  nav_msgs::msg::Path global_plan;
  {
    std::lock_guard<std::mutex> plan_lock(plan_mutex_);
    global_plan = global_plan_;
  }
  if (global_plan.poses.empty()) {
    RCLCPP_WARN(logger_, "global plan is empty, return 0 velocity command");
    return cmd;
  }

  geometry_msgs::msg::TransformStamped tf_base_link_map;
  geometry_msgs::msg::TransformStamped tf_base_link_scan;
  try {
    const rclcpp::Duration tf_timeout = rclcpp::Duration::from_seconds(transform_tolerance_);
    tf_base_link_map = tf_->lookupTransform(base_link_frame_, map_frame_, cmd.header.stamp, tf_timeout);
    tf_base_link_scan = tf_->lookupTransform(base_link_frame_, scan_frame_, cmd.header.stamp, tf_timeout);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Failed to lookup transform: %s", ex.what());
    return cmd;
  }

  std::vector<std::array<float, 2>> map_plan_poses(global_plan.poses.size());
  for (size_t i = 0; i < global_plan.poses.size(); i++) {
    map_plan_poses[i][0] = static_cast<float>(global_plan.poses[i].pose.position.x);
    map_plan_poses[i][1] = static_cast<float>(global_plan.poses[i].pose.position.y);
  }

  const std::vector<std::array<float, 2>> base_plan_poses = transformPoints2D(map_plan_poses, tf_base_link_map);
  size_t nearest_plan_index = 0;
  float nearest_plan_distance_sq = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < base_plan_poses.size(); i++) {
    const float x = base_plan_poses[i][0];
    const float y = base_plan_poses[i][1];
    const float distance_sq = x * x + y * y;
    if (distance_sq < nearest_plan_distance_sq) {
      nearest_plan_distance_sq = distance_sq;
      nearest_plan_index = i;
    }
  }
  // select only the plan points located ahead of the robot
  const std::vector<std::array<float, 2>> base_plan_poses_from_nearest(
    base_plan_poses.begin() + nearest_plan_index,
    base_plan_poses.end());

  std::vector<float> h_plan(1 * plan_length_ * 2, 0.0f);
  for (size_t i = 0; i < plan_length_; i++) {
    const size_t plan_index =
      (i < base_plan_poses_from_nearest.size()) ? i : (base_plan_poses_from_nearest.size() - 1);
    h_plan[i * 2 + 0] = base_plan_poses_from_nearest[plan_index][0];
    h_plan[i * 2 + 1] = base_plan_poses_from_nearest[plan_index][1];
  }

  std::vector<float> h_scan(1 * kScanLength, 0.0f);
  for (size_t i = 0; i < kScanLength; i++) {
    float r = scan_msg->ranges[i];
    if (!std::isfinite(r)) {
      r = kScanRangeMax;
    }
    h_scan[i] = r;
  }

  std::vector<float> h_people;
  if (people_encoder_enabled_) {
    h_people = buildPeopleInput();
  }

  float v_pred = 0.0;
  float w_pred = 0.0;
  {
    std::lock_guard<std::mutex> lk(trt_mutex_);

    cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
      d_odom_, h_odom.data(), h_odom.size(),
      trt_engine_->getTensorDataType(kInputOdomName), trt_stream_);
    cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
      d_plan_, h_plan.data(), h_plan.size(),
      trt_engine_->getTensorDataType(kInputPlanName), trt_stream_);
    cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
      d_scan_, h_scan.data(), h_scan.size(),
      trt_engine_->getTensorDataType(kInputScanName), trt_stream_);
    if (people_encoder_enabled_) {
      cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
        d_people_, h_people.data(), h_people.size(),
        trt_engine_->getTensorDataType(kInputPeopleName), trt_stream_);
    }

    if (!trt_context_->enqueueV3(trt_stream_)) {
      RCLCPP_ERROR(logger_, "TensorRT enqueueV3 failed");
      CUDA_CHECK(cudaStreamSynchronize(trt_stream_));
      return cmd;
    }

    if ((action_mode_ == ActionMode::kReg) || (action_mode_ == ActionMode::kMdnReg)) {
      std::vector<float> h_cmd(2, 0.0f);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        h_cmd.data(), d_cmd_, h_cmd.size(),
        trt_engine_->getTensorDataType(kOutputCmdName), trt_stream_);
      CUDA_CHECK(cudaStreamSynchronize(trt_stream_));

      v_pred = h_cmd[0];
      w_pred = h_cmd[1];
    } else if (action_mode_ == ActionMode::kCls) {
      std::vector<float> v_logits(v_num_bins_, 0.0f);
      std::vector<float> w_logits(w_num_bins_, 0.0f);

      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        v_logits.data(), d_v_logits_, v_logits.size(),
        trt_engine_->getTensorDataType(kOutputVLogitsName), trt_stream_);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        w_logits.data(), d_w_logits_, w_logits.size(),
        trt_engine_->getTensorDataType(kOutputWLogitsName), trt_stream_);
      CUDA_CHECK(cudaStreamSynchronize(trt_stream_));

      std::vector<float> v_logits_first(v_logits.begin(), v_logits.begin() + v_num_bins_);
      std::vector<float> w_logits_first(w_logits.begin(), w_logits.begin() + w_num_bins_);
      v_pred = cabot_dnn_controller::classify_utils::valueFromArgmaxLogits(v_logits_first, kVMin, kVMax);
      w_pred = cabot_dnn_controller::classify_utils::valueFromArgmaxLogits(w_logits_first, kWMin, kWMax);
    } else {
      std::vector<float> h_cmd(static_cast<size_t>(v_num_bins_) * static_cast<size_t>(w_num_bins_) * 2, 0.0f);
      std::vector<float> v_logits(v_num_bins_, 0.0f);
      std::vector<float> w_logits(w_num_bins_, 0.0f);

      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        h_cmd.data(), d_cmd_, h_cmd.size(),
        trt_engine_->getTensorDataType(kOutputCmdName), trt_stream_);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        v_logits.data(), d_v_logits_, v_logits.size(),
        trt_engine_->getTensorDataType(kOutputVLogitsName), trt_stream_);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        w_logits.data(), d_w_logits_, w_logits.size(),
        trt_engine_->getTensorDataType(kOutputWLogitsName), trt_stream_);
      CUDA_CHECK(cudaStreamSynchronize(trt_stream_));

      std::vector<float> v_logits_first(v_logits.begin(), v_logits.begin() + v_num_bins_);
      std::vector<float> w_logits_first(w_logits.begin(), w_logits.begin() + w_num_bins_);
      auto v_it = std::max_element(v_logits_first.begin(), v_logits_first.end());
      auto w_it = std::max_element(w_logits_first.begin(), w_logits_first.end());
      const size_t v_idx = std::distance(v_logits_first.begin(), v_it);
      const size_t w_idx = std::distance(w_logits_first.begin(), w_it);
      const size_t cmd_idx = v_idx * static_cast<size_t>(w_num_bins_) + w_idx;
      v_pred = h_cmd[2 * cmd_idx];
      w_pred = h_cmd[2 * cmd_idx + 1];
    }
  }

  cmd.twist.linear.x  = v_pred * max_linear_vel_;
  cmd.twist.angular.z = w_pred * max_angular_vel_;

  if (debug_image_pub_ && debug_image_pub_->get_subscription_count() > 0) {
    constexpr int kImageSize = 400;
    constexpr float kMetersPerPixel = 0.1f;
    cv::Mat image(kImageSize, kImageSize, CV_8UC3, cv::Scalar(20, 20, 20));
    const int cx = kImageSize / 2;
    const int cy = kImageSize / 2;
    const auto toPixel = [&](float x, float y) -> cv::Point {
      const int px = cx + static_cast<int>(x / kMetersPerPixel);
      const int py = cy - static_cast<int>(y / kMetersPerPixel);
      return cv::Point(px, py);
    };

    {
      const float segment_offset = -0.3f;
      const float line_len = 5.0f;
      cv::line(image, toPixel(0.0f, -segment_offset), toPixel(line_len * h_odom[0], -segment_offset), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

      const float odom_yaw_vel = h_odom[1];
      const float arc_span = std::max(20.0f, std::min(140.0f, std::abs(odom_yaw_vel) * 60.0f));
      const float start_deg = (odom_yaw_vel >= 0.0f) ? 0.0f : -arc_span;
      const float end_deg = (odom_yaw_vel >= 0.0f) ? arc_span : 0.0f;
      const float radius = 0.8f;
      const int steps = 32;
      std::vector<cv::Point> arc_points;
      arc_points.reserve(steps + 1);
      for (int i = 0; i <= steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const float deg = start_deg + (end_deg - start_deg) * t;
        const float rad = deg * static_cast<float>(M_PI) / 180.0f;
        arc_points.push_back(toPixel(radius * std::cos(rad), radius * std::sin(rad)));
      }
      if (arc_points.size() >= 2) {
        cv::polylines(image, arc_points, false, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      }
    }
    {
      const float segment_offset = 0.3f;
      const float line_len = 5.0f;
      cv::line(image, toPixel(0.0f, segment_offset), toPixel(line_len * v_pred, segment_offset), cv::Scalar(255, 0, 255), 1, cv::LINE_AA);

      const float arc_span = std::max(20.0f, std::min(140.0f, std::abs(w_pred) * 60.0f));
      const float start_deg = (w_pred >= 0.0f) ? 0.0f : -arc_span;
      const float end_deg = (w_pred >= 0.0f) ? arc_span : 0.0f;
      const float radius = 1.2f;
      const int steps = 32;
      std::vector<cv::Point> arc_points;
      arc_points.reserve(steps + 1);
      for (int i = 0; i <= steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const float deg = start_deg + (end_deg - start_deg) * t;
        const float rad = deg * static_cast<float>(M_PI) / 180.0f;
        arc_points.push_back(toPixel(radius * std::cos(rad), radius * std::sin(rad)));
      }
      if (arc_points.size() >= 2) {
        cv::polylines(image, arc_points, false, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
      }
    }

    for (size_t i = 0; i + 1 < h_plan.size(); i += 2) {
      const cv::Point px = toPixel(h_plan[i], h_plan[i + 1]);
      if (px.x >= 0 && px.x < kImageSize && px.y >= 0 && px.y < kImageSize) {
        image.at<cv::Vec3b>(px.y, px.x) = cv::Vec3b(0, 200, 255);
      }
    }

    if (people_encoder_enabled_) {
      for (size_t i = 0; i < static_cast<size_t>(num_people_); ++i) {
        const size_t offset = i * kPeopleDim;
        if (h_people[offset + 2] <= 0.0f) {
          continue;
        }
        const cv::Point px = toPixel(h_people[offset + 0], h_people[offset + 1]);
        if (px.x >= 0 && px.x < kImageSize && px.y >= 0 && px.y < kImageSize) {
          cv::circle(image, px, 4, cv::Scalar(255, 80, 80), -1, cv::LINE_AA);
        }
      }
    }

    const auto & t_scan = tf_base_link_scan.transform.translation;
    const auto & r_scan = tf_base_link_scan.transform.rotation;
    const double scan_yaw = std::atan2(2.0 * (r_scan.w * r_scan.z + r_scan.x * r_scan.y), 1.0 - 2.0 * (r_scan.y * r_scan.y + r_scan.z * r_scan.z));
    std::vector<std::array<float, 2>> base_scan_points;
    base_scan_points.reserve(kScanLength);
    for (size_t i = 0; i < kScanLength; i++) {
      const float r = h_scan[i];
      const float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

      const float local_x = r * std::cos(angle);
      const float local_y = r * std::sin(angle);
      const float base_x = t_scan.x + std::cos(scan_yaw) * local_x - std::sin(scan_yaw) * local_y;
      const float base_y = t_scan.y + std::sin(scan_yaw) * local_x + std::cos(scan_yaw) * local_y;

      base_scan_points.push_back({
        static_cast<float>(base_x),
        static_cast<float>(base_y),
      });
    }
    for (const auto & point : base_scan_points) {
      const cv::Point px = toPixel(point[0], point[1]);
      if (px.x >= 0 && px.x < kImageSize && px.y >= 0 && px.y < kImageSize) {
        image.at<cv::Vec3b>(px.y, px.x) = cv::Vec3b(0, 255, 0);
      }
    }

    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = cmd.header.stamp;
    cv_img.header.frame_id = base_link_frame_;
    cv_img.encoding = "bgr8";
    cv_img.image = image;
    debug_image_pub_->publish(*cv_img.toImageMsg());
  }

  return cmd;
}

void DnnController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  global_plan_ = path;
}

void DnnController::setSpeedLimit(const double &, const bool &)
{
  // TODO: implement speed limiting for DnnController.
}

void DnnController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!trt_ready_) {
    RCLCPP_INFO(logger_, "TensorRT is not ready");
    return;
  }

  std::lock_guard<std::mutex> odom_lock(odom_mutex_);
  odom_history_.push_back({
    static_cast<float>(msg->twist.twist.linear.x),
    static_cast<float>(msg->twist.twist.angular.z),
  });
  if (odom_history_.size() > static_cast<size_t>(odom_length_)) {
    // if history is longer than input odom length, discard old data
    while (odom_history_.size() > static_cast<size_t>(odom_length_)) {
      odom_history_.erase(odom_history_.begin());
    }
  } else if (odom_history_.size() < static_cast<size_t>(odom_length_)) {
    // if history is shorter than input odom length, fill with the initial value
    odom_history_.insert(odom_history_.begin(), static_cast<size_t>(odom_length_) - odom_history_.size(), odom_history_.front());
  }
}

void DnnController::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> scan_lock(scan_mutex_);
  last_scan_ = msg;
}

void DnnController::peopleCallback(const people_msgs::msg::People::SharedPtr msg)
{
  std::lock_guard<std::mutex> people_lock(people_mutex_);
  last_people_ = msg;
}

}  // namespace cabot_dnn_controller

PLUGINLIB_EXPORT_CLASS(cabot_dnn_controller::DnnController, nav2_core::Controller)
