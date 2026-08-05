#include "cabot_dnn_controller/dnn_controller.hpp"

#include <filesystem>
#include <fstream>
#include <cstdio>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
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
using cabot_dnn_controller::dnn_controller_constants::kPeopleDimWithVelocity;

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
using cabot_dnn_controller::dnn_controller_constants::kOutputPeopleAttentionName;
using cabot_dnn_controller::dnn_controller_constants::kOutputRobotPeopleAttentionName;

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

bool hasIOTensor(const nvinfer1::ICudaEngine & engine, const char * tensor_name)
{
  for (int i = 0; i < engine.getNbIOTensors(); ++i) {
    if (std::string(engine.getIOTensorName(i)) == tensor_name) {
      return true;
    }
  }
  return false;
}

double yawFromTransform(const geometry_msgs::msg::TransformStamped & tf)
{
  const auto & r = tf.transform.rotation;
  return std::atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z));
}

std::array<float, 2> transformPoint2D(
  const std::array<float, 2> & point,
  const geometry_msgs::msg::TransformStamped & tf)
{
  const auto & t = tf.transform.translation;
  const double yaw = yawFromTransform(tf);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  return {
    static_cast<float>(t.x + cy * point[0] - sy * point[1]),
    static_cast<float>(t.y + sy * point[0] + cy * point[1]),
  };
}

std::array<float, 2> transformVector2D(
  const std::array<float, 2> & vector,
  const geometry_msgs::msg::TransformStamped & tf)
{
  const double yaw = yawFromTransform(tf);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  return {
    static_cast<float>(cy * vector[0] - sy * vector[1]),
    static_cast<float>(sy * vector[0] + cy * vector[1]),
  };
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
  node_->declare_parameter<std::string>(
    prefix + "people_attention_topic", "debug/people_attention");
  node_->get_parameter(prefix + "people_attention_topic", people_attention_topic_);
  node_->declare_parameter<std::string>(
    prefix + "robot_people_attention_topic", "debug/robot_people_attention");
  node_->get_parameter(prefix + "robot_people_attention_topic", robot_people_attention_topic_);

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
    input_velocity_ = false;
    num_people_ = 0;
    people_history_length_ = 0;
    people_dim_ = kPeopleDim;
    presence_index_ = kPeopleDim - 1;
    num_attention_heads_ = 0;
    const YAML::Node people_config = config["people_encoder"];
    if (people_config && people_config["enabled"]) {
      people_encoder_enabled_ = people_config["enabled"].as<bool>();
    }
    if (people_encoder_enabled_) {
      if (people_config["input_velocity"]) {
        input_velocity_ = people_config["input_velocity"].as<bool>();
      }
      people_dim_ = input_velocity_ ?
        kPeopleDimWithVelocity : kPeopleDim;
      presence_index_ = people_dim_ - 1;
      if (!people_config["num_people"]) {
        throw std::runtime_error("people_encoder.num_people is required when people_encoder is enabled");
      }
      num_people_ = people_config["num_people"].as<int>();
      if (num_people_ <= 0) {
        throw std::runtime_error("people_encoder.num_people must be > 0");
      }
      if (!people_config["history_length"]) {
        throw std::runtime_error("people_encoder.history_length is required when people_encoder is enabled");
      }
      people_history_length_ = people_config["history_length"].as<int>();
      if (people_history_length_ <= 0) {
        throw std::runtime_error("people_encoder.history_length must be > 0");
      }
      num_attention_heads_ = people_config["num_attention_heads"].as<int>();
      if (num_attention_heads_ <= 0) {
        throw std::runtime_error("people_encoder.num_attention_heads must be > 0");
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
    const bool engine_has_people = hasIOTensor(*trt_engine_, kInputPeopleName);
    if (people_encoder_enabled_ != engine_has_people) {
      throw std::runtime_error(
              people_encoder_enabled_ ?
              "people_encoder is enabled, but TensorRT engine has no people input" :
              "TensorRT engine has a people input, but people_encoder is disabled");
    }
    if (people_encoder_enabled_ &&
      (!hasIOTensor(*trt_engine_, kOutputPeopleAttentionName) ||
      !hasIOTensor(*trt_engine_, kOutputRobotPeopleAttentionName)))
    {
      throw std::runtime_error(
              "TensorRT engine has no attention outputs; re-export it with export_tensorrt.py");
    }

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
        trt_context_->setInputShape(
          kInputPeopleName,
          nvinfer1::Dims4{1, num_people_, people_history_length_, people_dim_});
    }
    if (!input_shapes_set) {
      throw std::runtime_error("Failed to set TensorRT input shapes");
    }
    if (people_encoder_enabled_) {
      const nvinfer1::Dims people_dims =
        trt_context_->getTensorShape(kInputPeopleName);
      if (people_dims.nbDims != 4 || people_dims.d[0] != 1 ||
        people_dims.d[1] != num_people_ ||
        people_dims.d[2] != people_history_length_ ||
        people_dims.d[3] != people_dim_)
      {
        throw std::runtime_error(
                "TensorRT people input shape does not match model config");
      }
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
    if (people_encoder_enabled_) {
      d_people_attention_ = allocTensor(kOutputPeopleAttentionName);
      d_robot_people_attention_ = allocTensor(kOutputRobotPeopleAttentionName);
    }

    bool tensor_addresses_set =
      trt_context_->setTensorAddress(kInputOdomName, d_odom_) &&
      trt_context_->setTensorAddress(kInputPlanName, d_plan_) &&
      trt_context_->setTensorAddress(kInputScanName, d_scan_) &&
      trt_context_->setTensorAddress(kOutputCmdName, d_cmd_) &&
      trt_context_->setTensorAddress(kOutputVLogitsName, d_v_logits_) &&
      trt_context_->setTensorAddress(kOutputWLogitsName, d_w_logits_);
    if (people_encoder_enabled_) {
      tensor_addresses_set = tensor_addresses_set &&
        trt_context_->setTensorAddress(kInputPeopleName, d_people_) &&
        trt_context_->setTensorAddress(kOutputPeopleAttentionName, d_people_attention_) &&
        trt_context_->setTensorAddress(kOutputRobotPeopleAttentionName, d_robot_people_attention_);
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
  if (people_encoder_enabled_) {
    people_attention_pub_ = node_->create_publisher<cabot_dnn_controller::msg::AttentionWeights>(
      people_attention_topic_, rclcpp::SystemDefaultsQoS());
    robot_people_attention_pub_ = node_->create_publisher<cabot_dnn_controller::msg::AttentionWeights>(
      robot_people_attention_topic_, rclcpp::SystemDefaultsQoS());
  }
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
  people_attention_pub_.reset();
  robot_people_attention_pub_.reset();
  {
    std::lock_guard<std::mutex> people_lock(people_mutex_);
    latest_people_.clear();
    has_people_observation_ = false;
    people_history_.clear();
  }

  trt_ready_ = false;
  if (d_odom_) { CUDA_CHECK(cudaFree(d_odom_)); d_odom_ = nullptr; }
  if (d_plan_) { CUDA_CHECK(cudaFree(d_plan_)); d_plan_ = nullptr; }
  if (d_scan_) { CUDA_CHECK(cudaFree(d_scan_)); d_scan_ = nullptr; }
  if (d_people_) { CUDA_CHECK(cudaFree(d_people_)); d_people_ = nullptr; }
  if (d_cmd_)  { CUDA_CHECK(cudaFree(d_cmd_));  d_cmd_  = nullptr; }
  if (d_w_logits_)  { CUDA_CHECK(cudaFree(d_w_logits_));  d_w_logits_  = nullptr; }
  if (d_v_logits_)  { CUDA_CHECK(cudaFree(d_v_logits_));  d_v_logits_  = nullptr; }
  if (d_people_attention_) { CUDA_CHECK(cudaFree(d_people_attention_)); d_people_attention_ = nullptr; }
  if (d_robot_people_attention_) { CUDA_CHECK(cudaFree(d_robot_people_attention_)); d_robot_people_attention_ = nullptr; }
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
  input_velocity_ = false;
  num_people_ = 0;
  people_history_length_ = 0;
  people_dim_ = kPeopleDim;
  presence_index_ = kPeopleDim - 1;
  num_attention_heads_ = 0;
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

std::vector<float> DnnController::buildPeopleInput(
  const geometry_msgs::msg::TransformStamped & tf_base_link_map,
  const rclcpp::Time & current_time,
  const std::unordered_map<std::string, std::deque<PeopleHistoryRecord>> &
  people_history)
{
  const size_t people_count = static_cast<size_t>(num_people_);
  const size_t history_count = static_cast<size_t>(people_history_length_);
  std::vector<float> h_people(
    people_count * history_count * static_cast<size_t>(people_dim_), 0.0f);
  if (!people_encoder_enabled_ || num_people_ <= 0 || people_history_length_ <= 0) {
    return h_people;
  }

  if (people_history.empty()) {
    return h_people;
  }

  struct PeopleCandidate
  {
    float distance_sq{0.0f};
    std::vector<PeopleHistoryRecord> records;
  };

  const std::int64_t current_time_ns = current_time.nanoseconds();
  std::vector<PeopleCandidate> candidates;
  candidates.reserve(people_history.size());
  for (const auto & entry : people_history) {
    const auto & records = entry.second;
    std::vector<PeopleHistoryRecord> valid_records;
    valid_records.reserve(records.size());
    for (const auto & record : records) {
      if (record.stamp_ns <= current_time_ns) {
        valid_records.push_back(record);
      }
    }
    if (valid_records.empty()) {
      continue;
    }

    const size_t start_index =
      valid_records.size() > history_count ? valid_records.size() - history_count : 0;
    std::vector<PeopleHistoryRecord> history_records(valid_records.begin() + start_index, valid_records.end());
    if (history_records.empty() || history_records.back().presence <= 0.0f) {
      continue;
    }

    const std::array<float, 2> latest_position =
      transformPoint2D(history_records.back().position, tf_base_link_map);
    if (!std::isfinite(latest_position[0]) || !std::isfinite(latest_position[1])) {
      continue;
    }

    PeopleCandidate candidate;
    candidate.distance_sq =
      latest_position[0] * latest_position[0] + latest_position[1] * latest_position[1];
    candidate.records = std::move(history_records);
    candidates.push_back(std::move(candidate));
  }

  if (candidates.empty()) {
    return h_people;
  }

  std::sort(candidates.begin(), candidates.end(), [](const PeopleCandidate & lhs, const PeopleCandidate & rhs) {
    return lhs.distance_sq < rhs.distance_sq;
  });

  const size_t output_count = std::min(candidates.size(), people_count);
  for (size_t person_index = 0; person_index < output_count; ++person_index) {
    const auto & records = candidates[person_index].records;
    const size_t history_offset = history_count - records.size();
    for (size_t record_index = 0; record_index < records.size(); ++record_index) {
      const auto & record = records[record_index];
      const size_t output_history_index = history_offset + record_index;
      const size_t output_offset =
        (person_index * history_count + output_history_index) *
        static_cast<size_t>(people_dim_);

      if (record.presence <= 0.0f) {
        continue;
      }

      const std::array<float, 2> position = transformPoint2D(record.position, tf_base_link_map);
      if (!std::isfinite(position[0]) || !std::isfinite(position[1])) {
        continue;
      }

      std::array<float, 2> velocity{0.0f, 0.0f};
      if (input_velocity_) {
        velocity = transformVector2D(record.velocity, tf_base_link_map);
        if (!std::isfinite(velocity[0]) || !std::isfinite(velocity[1])) {
          continue;
        }
      }

      h_people[output_offset + 0] = position[0];
      h_people[output_offset + 1] = position[1];
      if (input_velocity_) {
        h_people[output_offset + 2] = velocity[0];
        h_people[output_offset + 3] = velocity[1];
      }
      h_people[output_offset + static_cast<size_t>(presence_index_)] = 1.0f;
    }
  }

  return h_people;
}

void DnnController::publishAttention(
  const std::vector<float> & people_attention,
  const std::vector<float> & robot_people_attention,
  const std_msgs::msg::Header & header) const
{
  const int people_rows = num_attention_heads_ * num_people_;
  const size_t expected_people =
    static_cast<size_t>(people_rows) * static_cast<size_t>(num_people_);
  const size_t expected_robot =
    static_cast<size_t>(num_attention_heads_) * static_cast<size_t>(num_people_);
  if (people_attention.size() != expected_people ||
    robot_people_attention.size() != expected_robot)
  {
    RCLCPP_ERROR(
      logger_, "Unexpected attention sizes: people=%zu (expected %zu), robot=%zu (expected %zu)",
      people_attention.size(), expected_people, robot_people_attention.size(), expected_robot);
    return;
  }

  cabot_dnn_controller::msg::AttentionWeights people_msg;
  people_msg.header = header;
  people_msg.num_heads = static_cast<uint32_t>(num_attention_heads_);
  people_msg.num_queries = static_cast<uint32_t>(num_people_);
  people_msg.num_keys = static_cast<uint32_t>(num_people_);
  people_msg.weights = people_attention;
  people_attention_pub_->publish(people_msg);

  cabot_dnn_controller::msg::AttentionWeights robot_msg;
  robot_msg.header = header;
  robot_msg.num_heads = static_cast<uint32_t>(num_attention_heads_);
  robot_msg.num_queries = 1;
  robot_msg.num_keys = static_cast<uint32_t>(num_people_);
  robot_msg.weights = robot_people_attention;
  robot_people_attention_pub_->publish(robot_msg);
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

  std::vector<PlanarVelocity> odom_history;
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    odom_history = odom_history_;
  }

  sensor_msgs::msg::LaserScan::SharedPtr scan_msg;
  std::unordered_map<std::string, std::deque<PeopleHistoryRecord>> people_history;
  {
    std::scoped_lock state_lock(scan_mutex_, people_mutex_);
    scan_msg = last_scan_;
    people_history = people_history_;
  }
  // Keep the inference time at or after every scan-aligned history record in the snapshot.
  cmd.header.stamp = clock_->now();

  if (odom_history.empty() || !scan_msg) {
    RCLCPP_INFO(logger_, "odom or scan is not ready, return 0 velocity command");
    return cmd;
  }

  if (odom_history.size() != odom_length_) {
    RCLCPP_ERROR(logger_, "odom size is not correct, return 0 velocity command, input size=%zu, expected size=%zu",
      odom_history.size(), static_cast<size_t>(odom_length_));
    return cmd;
  }
  if (scan_msg->ranges.size() != kScanLength) {
    RCLCPP_ERROR(logger_, "scan size is not correct, return 0 velocity command, input size=%ld, expected size=%ld",
      scan_msg->ranges.size(), kScanLength);
    return cmd;
  }

  std::vector<float> h_odom(1 * odom_length_ * 2, 0.0f);
  for (size_t i = 0; i < odom_length_; i++) {
    if (!std::isfinite(odom_history[i].vx) || !std::isfinite(odom_history[i].wz)) {
      RCLCPP_ERROR(logger_, "Non-finite odometry input, return 0 velocity command");
      return cmd;
    }
    h_odom[i * 2 + 0] = odom_history[i].vx / static_cast<float>(max_linear_vel_);
    h_odom[i * 2 + 1] = odom_history[i].wz / static_cast<float>(max_angular_vel_);
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
    h_people = buildPeopleInput(tf_base_link_map, cmd.header.stamp, people_history);
  }

  float v_pred = 0.0;
  float w_pred = 0.0;
  std::vector<float> people_attention;
  std::vector<float> robot_people_attention;
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

    if (people_encoder_enabled_) {
      const size_t head_count = static_cast<size_t>(num_attention_heads_);
      const size_t person_count = static_cast<size_t>(num_people_);
      people_attention.resize(head_count * person_count * person_count);
      robot_people_attention.resize(head_count * person_count);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        people_attention.data(), d_people_attention_, people_attention.size(),
        trt_engine_->getTensorDataType(kOutputPeopleAttentionName), trt_stream_);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        robot_people_attention.data(), d_robot_people_attention_, robot_people_attention.size(),
        trt_engine_->getTensorDataType(kOutputRobotPeopleAttentionName), trt_stream_);
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

  if (people_encoder_enabled_) {
    std_msgs::msg::Header attention_header;
    attention_header.stamp = cmd.header.stamp;
    attention_header.frame_id = base_link_frame_;
    publishAttention(people_attention, robot_people_attention, attention_header);
  }

  if (debug_image_pub_ && debug_image_pub_->get_subscription_count() > 0) {
    constexpr int kImageSize = 400;
    constexpr float kMetersPerPixel = 0.1f;
    constexpr float kPeopleVelocityArrowSeconds = 1.0f;
    constexpr int kSubpixelShift = 4;
    constexpr int kSubpixelScale = 1 << kSubpixelShift;
    constexpr int kMaxSafePixelCoordinate = 1 << 20;
    constexpr double kMaxImageCoordinateMeters =
      static_cast<double>(kImageSize / 2 - 1) * kMetersPerPixel;
    const cv::Scalar people_velocity_color(255, 0, 0);
    cv::Mat image(kImageSize, kImageSize, CV_8UC3, cv::Scalar(20, 20, 20));
    const int cx = kImageSize / 2;
    const int cy = kImageSize / 2;
    const auto safePixelCoordinate = [&](double value) {
        if (!std::isfinite(value)) {
          return -kMaxSafePixelCoordinate;
        }
        return static_cast<int>(std::clamp(
            value,
            -static_cast<double>(kMaxSafePixelCoordinate),
            static_cast<double>(kMaxSafePixelCoordinate)));
      };
    const auto toPixel = [&](double x, double y) -> cv::Point {
        return cv::Point(
          safePixelCoordinate(cx + x / kMetersPerPixel),
          safePixelCoordinate(cy - y / kMetersPerPixel));
      };
    const auto toSubpixel = [&](double x, double y) -> cv::Point {
        return cv::Point(
          safePixelCoordinate((cx + x / kMetersPerPixel) * kSubpixelScale),
          safePixelCoordinate((cy - y / kMetersPerPixel) * kSubpixelScale));
      };
    const auto pointIsVisible = [&](double x, double y) {
        return std::isfinite(x) && std::isfinite(y) &&
               std::abs(x) <= kMaxImageCoordinateMeters &&
               std::abs(y) <= kMaxImageCoordinateMeters;
      };
    const auto clipEndpointToImage = [&](double x, double y, double dx, double dy) {
        double scale = 1.0;
        if (dx > 0.0) {
          scale = std::min(scale, (kMaxImageCoordinateMeters - x) / dx);
        } else if (dx < 0.0) {
          scale = std::min(scale, (-kMaxImageCoordinateMeters - x) / dx);
        }
        if (dy > 0.0) {
          scale = std::min(scale, (kMaxImageCoordinateMeters - y) / dy);
        } else if (dy < 0.0) {
          scale = std::min(scale, (-kMaxImageCoordinateMeters - y) / dy);
        }
        scale = std::clamp(scale, 0.0, 1.0);
        return std::array<double, 2>{x + scale * dx, y + scale * dy};
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

    std::vector<std::pair<cv::Point, cv::Point>> people_velocity_arrows;
    if (people_encoder_enabled_) {
      const size_t history_count = static_cast<size_t>(people_history_length_);
      for (size_t i = 0; i < static_cast<size_t>(num_people_); ++i) {
        for (size_t h = 0; h < history_count; ++h) {
          const size_t offset =
            (i * history_count + h) * static_cast<size_t>(people_dim_);
          if (h_people[offset + static_cast<size_t>(presence_index_)] <= 0.0f) {
            continue;
          }

          const float age_ratio = history_count > 1 ?
            static_cast<float>(h) / static_cast<float>(history_count - 1) : 1.0f;
          const int red = static_cast<int>(120.0f + 135.0f * age_ratio);
          const int green = static_cast<int>(40.0f + 40.0f * age_ratio);
          const int blue = static_cast<int>(40.0f + 40.0f * age_ratio);
          const cv::Scalar color(blue, green, red);

          const float x = h_people[offset + 0];
          const float y = h_people[offset + 1];
          if (!pointIsVisible(x, y)) {
            continue;
          }
          const cv::Point px = toPixel(x, y);

          const int radius = (h + 1 == history_count) ? 4 : 2;
          cv::circle(image, px, radius, color, -1, cv::LINE_AA);
          if (input_velocity_) {
            const float velocity_x = h_people[offset + 2];
            const float velocity_y = h_people[offset + 3];
            if (std::isfinite(velocity_x) && std::isfinite(velocity_y) &&
              (velocity_x != 0.0f || velocity_y != 0.0f))
            {
              const double velocity_dx =
                kPeopleVelocityArrowSeconds * static_cast<double>(velocity_x);
              const double velocity_dy =
                kPeopleVelocityArrowSeconds * static_cast<double>(velocity_y);
              const auto velocity_end = clipEndpointToImage(
                x, y, velocity_dx, velocity_dy);
              people_velocity_arrows.emplace_back(
                toSubpixel(x, y),
                toSubpixel(velocity_end[0], velocity_end[1]));
            }
          }
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

    if (people_encoder_enabled_ && !people_attention.empty()) {
      cv::Mat people_matrix(
        num_attention_heads_ * num_people_, num_people_, CV_32FC1,
        people_attention.data());
      cv::Mat robot_matrix(
        num_attention_heads_, num_people_, CV_32FC1,
        robot_people_attention.data());
      cv::Mat people_mean = cv::Mat::zeros(num_people_, num_people_, CV_32FC1);
      cv::Mat robot_mean = cv::Mat::zeros(1, num_people_, CV_32FC1);
      for (int head = 0; head < num_attention_heads_; ++head) {
        people_mean += people_matrix.rowRange(head * num_people_, (head + 1) * num_people_);
        robot_mean += robot_matrix.row(head);
      }
      people_mean /= static_cast<float>(num_attention_heads_);
      robot_mean /= static_cast<float>(num_attention_heads_);

      const size_t history_count = static_cast<size_t>(people_history_length_);
      std::vector<cv::Point> person_pixels(static_cast<size_t>(num_people_));
      std::vector<bool> person_present(static_cast<size_t>(num_people_), false);
      for (int person = 0; person < num_people_; ++person) {
        const size_t offset =
          (static_cast<size_t>(person) * history_count + history_count - 1) *
          static_cast<size_t>(people_dim_);
        const float person_x = h_people[offset];
        const float person_y = h_people[offset + 1];
        person_present[person] =
          h_people[offset + static_cast<size_t>(presence_index_)] > 0.0f &&
          pointIsVisible(person_x, person_y);
        if (person_present[person]) {
          person_pixels[person] = toPixel(person_x, person_y);
        }
      }

      const auto thicknessForWeight = [](float weight) {
          return std::max(1, static_cast<int>(std::lround(weight * 10.0f)));
        };
      constexpr float kMinVisibleWeight = 0.01f;
      const cv::Scalar people_attention_color(0, 180, 255);
      const cv::Scalar robot_attention_color(255, 220, 0);

      // Draw directed person-person attention as slightly offset parallel
      // arrows. Line width is the only encoding of the attention magnitude.
      for (int query = 0; query < num_people_; ++query) {
        if (!person_present[query]) {
          continue;
        }
        for (int key = 0; key < num_people_; ++key) {
          if (!person_present[key]) {
            continue;
          }
          const float weight = people_mean.at<float>(query, key);
          if (weight < kMinVisibleWeight) {
            continue;
          }
          if (query == key) {
            cv::circle(image, person_pixels[query], 8, people_attention_color,
              thicknessForWeight(weight), cv::LINE_AA);
            continue;
          }
          const cv::Point delta = person_pixels[key] - person_pixels[query];
          const double length = std::hypot(delta.x, delta.y);
          if (length < 1.0) {
            continue;
          }
          const cv::Point offset(
            static_cast<int>(std::lround(-delta.y * 3.0 / length)),
            static_cast<int>(std::lround(delta.x * 3.0 / length)));
          cv::arrowedLine(image, person_pixels[query] + offset, person_pixels[key] + offset,
            people_attention_color, thicknessForWeight(weight), cv::LINE_AA, 0, 0.08);
        }
      }

      // Robot-person attention originates at the robot center.
      const cv::Point robot_pixel = toPixel(0.0f, 0.0f);
      for (int key = 0; key < num_people_; ++key) {
        if (!person_present[key]) {
          continue;
        }
        const float weight = robot_mean.at<float>(0, key);
        if (weight >= kMinVisibleWeight) {
          cv::line(image, robot_pixel, person_pixels[key], robot_attention_color,
            thicknessForWeight(weight), cv::LINE_AA);
        }
      }

      // Keep people and robot positions visible above the attention lines.
      for (int person = 0; person < num_people_; ++person) {
        if (person_present[person]) {
          cv::circle(image, person_pixels[person], 4, cv::Scalar(80, 80, 255), -1, cv::LINE_AA);
        }
      }
      cv::circle(image, robot_pixel, 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    }

    for (const auto & [start, end] : people_velocity_arrows) {
      if (start == end) {
        continue;
      }
      cv::arrowedLine(
        image, start, end, people_velocity_color, 1,
        cv::LINE_AA, kSubpixelShift, 0.2);
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
  const rclcpp::Time stamp = clock_->now();
  std::scoped_lock state_lock(scan_mutex_, people_mutex_);
  last_scan_ = msg;
  // Advance people history on each /scan update using the latest /people observation.
  updatePeopleHistoryLocked(stamp.nanoseconds());
}

void DnnController::updatePeopleHistoryLocked(const std::int64_t stamp_ns)
{
  if (!people_encoder_enabled_ || people_history_length_ <= 0 || !has_people_observation_) {
    return;
  }

  const size_t max_history_size = static_cast<size_t>(people_history_length_);
  std::vector<std::string> erase_keys;
  for (auto & [key, records] : people_history_) {
    const auto observed = latest_people_.find(key);
    if (observed != latest_people_.end()) {
      PeopleHistoryRecord record = observed->second;
      record.stamp_ns = stamp_ns;
      records.push_back(record);
    } else {
      PeopleHistoryRecord absent_record;
      absent_record.stamp_ns = stamp_ns;
      absent_record.presence = 0.0f;
      records.push_back(absent_record);
    }
    while (records.size() > max_history_size) {
      records.pop_front();
    }
    const bool has_observed_record = std::any_of(
      records.begin(), records.end(), [](const auto & record) {
        return record.presence > 0.0f;
      });
    if (!has_observed_record) {
      erase_keys.push_back(key);
    }
  }
  for (const auto & key : erase_keys) {
    people_history_.erase(key);
  }
  for (const auto & [key, latest_record] : latest_people_) {
    if (people_history_.count(key) > 0) {
      continue;
    }
    PeopleHistoryRecord record = latest_record;
    record.stamp_ns = stamp_ns;
    people_history_[key].push_back(record);
  }
}

void DnnController::peopleCallback(const people_msgs::msg::People::SharedPtr msg)
{
  if (!people_encoder_enabled_ || people_history_length_ <= 0) {
    return;
  }

  if (map_frame_.empty()) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000,
      "Ignoring people message because map_frame is not set");
    return;
  }

  const std::string & source_frame = msg->header.frame_id;
  if (source_frame != map_frame_) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000,
      "Ignoring people message because header.frame_id must be '%s', got '%s'",
      map_frame_.c_str(), source_frame.c_str());
    return;
  }

  std::unordered_map<std::string, size_t> name_counts;
  name_counts.reserve(msg->people.size());
  for (const auto & person : msg->people) {
    if (!person.name.empty()) {
      ++name_counts[person.name];
    }
  }
  std::unordered_map<std::string, PeopleHistoryRecord> observed_people;
  observed_people.reserve(msg->people.size());
  for (size_t i = 0; i < msg->people.size(); ++i) {
    const auto & person = msg->people[i];
    if (person.name.empty()) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "Ignoring person at index %zu because name is empty", i);
      continue;
    }
    if (name_counts.at(person.name) > 1) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "Ignoring person because name '%s' is duplicated", person.name.c_str());
      continue;
    }
    const float x = static_cast<float>(person.position.x);
    const float y = static_cast<float>(person.position.y);
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    const float vx = static_cast<float>(person.velocity.x);
    const float vy = static_cast<float>(person.velocity.y);
    const bool velocity_is_finite = std::isfinite(vx) && std::isfinite(vy);
    // Position-only models do not depend on velocity, so keep detections whose
    // positions are valid even when the message has no usable velocity.
    if (input_velocity_ && !velocity_is_finite) {
      continue;
    }

    PeopleHistoryRecord record;
    record.position = {x, y};
    if (velocity_is_finite) {
      record.velocity = {vx, vy};
    }
    record.presence = 1.0f;
    observed_people.emplace(person.name, record);
  }

  std::lock_guard<std::mutex> people_lock(people_mutex_);
  latest_people_ = std::move(observed_people);
  has_people_observation_ = true;
}

}  // namespace cabot_dnn_controller

PLUGINLIB_EXPORT_CLASS(cabot_dnn_controller::DnnController, nav2_core::Controller)
