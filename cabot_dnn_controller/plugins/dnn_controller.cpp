#include "cabot_dnn_controller/dnn_controller.hpp"

#include <filesystem>
#include <fstream>
#include <cstdio>
#include <array>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
  #include <cv_bridge/cv_bridge.h>
#else
  #error "cv_bridge header not found: expected cv_bridge/cv_bridge.hpp or cv_bridge/cv_bridge.h"
#endif
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

using cabot_dnn_controller::dnn_controller_constants::kVMin;
using cabot_dnn_controller::dnn_controller_constants::kVMax;
using cabot_dnn_controller::dnn_controller_constants::kWMin;
using cabot_dnn_controller::dnn_controller_constants::kWMax;

using cabot_dnn_controller::dnn_controller_constants::kInputOdomName;
using cabot_dnn_controller::dnn_controller_constants::kInputPlanName;
using cabot_dnn_controller::dnn_controller_constants::kInputScanName;
using cabot_dnn_controller::dnn_controller_constants::kOutputCmdName;
using cabot_dnn_controller::dnn_controller_constants::kOutputVLogitsName;
using cabot_dnn_controller::dnn_controller_constants::kOutputWLogitsName;

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

    trt_context_->setInputShape(kInputOdomName, nvinfer1::Dims3{1, odom_length_, 2});
    trt_context_->setInputShape(kInputPlanName, nvinfer1::Dims3{1, plan_length_, 2});
    trt_context_->setInputShape(kInputScanName, nvinfer1::Dims2{1, kScanLength});

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
    d_cmd_  = allocTensor(kOutputCmdName);
    d_v_logits_  = allocTensor(kOutputVLogitsName);
    d_w_logits_  = allocTensor(kOutputWLogitsName);

    if (!trt_context_->setTensorAddress(kInputOdomName, d_odom_) ||
        !trt_context_->setTensorAddress(kInputPlanName, d_plan_) ||
        !trt_context_->setTensorAddress(kInputScanName, d_scan_) ||
        !trt_context_->setTensorAddress(kOutputCmdName, d_cmd_) ||
        !trt_context_->setTensorAddress(kOutputVLogitsName, d_v_logits_) ||
        !trt_context_->setTensorAddress(kOutputWLogitsName, d_w_logits_))
    {
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
  debug_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SystemDefaultsQoS());
}

void DnnController::cleanup()
{
  global_plan_.poses.clear();
  odom_sub_.reset();
  odom_history_.clear();
  scan_sub_.reset();
  last_scan_.reset();

  trt_ready_ = false;
  if (d_odom_) { CUDA_CHECK(cudaFree(d_odom_)); d_odom_ = nullptr; }
  if (d_plan_) { CUDA_CHECK(cudaFree(d_plan_)); d_plan_ = nullptr; }
  if (d_scan_) { CUDA_CHECK(cudaFree(d_scan_)); d_scan_ = nullptr; }
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

  if (odom_history_.empty() || !last_scan_) {
    RCLCPP_INFO(logger_, "odom or scan is not ready, return 0 velocity command");
    return cmd;
  }

  if (odom_history_.size() != odom_length_) {
    RCLCPP_ERROR(logger_, "odom size is not correct, return 0 velocity command, input size=%ld, expected size=%ld",
      odom_history_.size(), odom_length_);
    return cmd;
  }
  if (last_scan_->ranges.size() != kScanLength) {
    RCLCPP_ERROR(logger_, "scan size is not correct, return 0 velocity command, input size=%ld, expected size=%ld",
      last_scan_->ranges.size(), kScanLength);
    return cmd;
  }

  std::vector<float> h_odom(1 * odom_length_ * 2, 0.0f);
  for (size_t i = 0; i < odom_length_; i++) {
    h_odom[i * 2 + 0] = odom_history_[i][0] / static_cast<float>(max_linear_vel_);
    h_odom[i * 2 + 1] = odom_history_[i][1] / static_cast<float>(max_angular_vel_);
  }

  if (!tf_) {
    RCLCPP_ERROR(logger_, "tf buffer is not available, return 0 velocity command");
    return cmd;
  }
  if (global_plan_.poses.empty()) {
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

  std::vector<std::array<float, 2>> map_plan_poses(global_plan_.poses.size());
  for (size_t i = 0; i < global_plan_.poses.size(); i++) {
    map_plan_poses[i][0] = static_cast<float>(global_plan_.poses[i].pose.position.x);
    map_plan_poses[i][1] = static_cast<float>(global_plan_.poses[i].pose.position.y);
  }

  const std::vector<std::array<float, 2>> base_plan_poses = transformPoints2D(map_plan_poses, tf_base_link_map);

  std::vector<float> h_plan(1 * plan_length_ * 2, 0.0f);
  for (size_t i = 0; i < plan_length_; i++) {
    const size_t plan_index = (i < base_plan_poses.size()) ? i : (base_plan_poses.size() - 1);
    h_plan[i * 2 + 0] = base_plan_poses[plan_index][0];
    h_plan[i * 2 + 1] = base_plan_poses[plan_index][1];
  }

  std::vector<float> h_scan(1 * kScanLength, 0.0f);
  for (size_t i = 0; i < kScanLength; i++) {
    float r = last_scan_->ranges[i];
    if (!std::isfinite(r)) {
      r = kScanRangeMax;
    }
    h_scan[i] = r;
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

    for (const auto & point : base_plan_poses) {
      const cv::Point px = toPixel(point[0], point[1]);
      if (px.x >= 0 && px.x < kImageSize && px.y >= 0 && px.y < kImageSize) {
        image.at<cv::Vec3b>(px.y, px.x) = cv::Vec3b(0, 200, 255);
      }
    }

    const auto & t_scan = tf_base_link_scan.transform.translation;
    const auto & r_scan = tf_base_link_scan.transform.rotation;
    const double scan_yaw = std::atan2(2.0 * (r_scan.w * r_scan.z + r_scan.x * r_scan.y), 1.0 - 2.0 * (r_scan.y * r_scan.y + r_scan.z * r_scan.z));
    std::vector<std::array<float, 2>> base_scan_points;
    base_scan_points.reserve(kScanLength);
    for (size_t i = 0; i < kScanLength; i++) {
      const float r = h_scan[i];
      const float angle = last_scan_->angle_min + i * last_scan_->angle_increment;

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
  last_scan_ = msg;
}

}  // namespace cabot_dnn_controller

PLUGINLIB_EXPORT_CLASS(cabot_dnn_controller::DnnController, nav2_core::Controller)
