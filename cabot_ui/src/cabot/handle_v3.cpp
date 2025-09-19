// Copyright (c) 2023, 2024  Miraikan, Carnegie Mellon University, and ALPS ALPINE CO., LTD.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <utility>
#include <thread>
#include <chrono>
#include <algorithm>

#include "handle_v3.hpp"

using namespace std::chrono_literals;
static DirectionalIndicator di;
const std::vector<std::string> Handle::stimuli_names =
{"unknown", "left_turn", "right_turn", "left_dev", "right_dev", "front",
  "left_about_turn", "right_about_turn", "button_click", "button_holddown",
  "caution", "navigation;event;navigation_start", "navigation_arrived"};
const rclcpp::Duration Handle::double_click_interval_ = rclcpp::Duration(0, 250000000);
const rclcpp::Duration Handle::ignore_interval_ = rclcpp::Duration(0, 50000000);
const rclcpp::Duration Handle::holddown_min_interval_ = rclcpp::Duration(1, 0);
const rclcpp::Duration Handle::holddown_max_interval_ = rclcpp::Duration(50, 500000000);  // 50.5 sec (margin=0.5sec)
const rclcpp::Duration Handle::holddown_interval_ = rclcpp::Duration(1, 0);

std::string Handle::get_name(int stimulus)
{
  return stimuli_names[stimulus];
}

Handle::Handle(
  std::shared_ptr<CaBotHandleV3Node> node,
  std::function<void(const std::map<std::string, int> &)> eventListener,
  const int & vibratorType)
: node_(node), eventListener_(std::move(eventListener)),
  vibratorType_(vibratorType), logger_(rclcpp::get_logger("Handle_v3"))
{
  vibrator1_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator1", 100);
  vibrator2_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator2", 100);
  vibrator3_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator3", 100);
  vibrator4_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator4", 100);
  servo_free_pub_ = node->create_publisher<std_msgs::msg::Bool>("servo_free", rclcpp::QoS(1));
  servo_target_pub_ = node->create_publisher<std_msgs::msg::Int16>("servo_target", rclcpp::QoS(10));
  button_sub_ = node->create_subscription<std_msgs::msg::Int8>(
    "pushed", rclcpp::SensorDataQoS(), std::bind(&Handle::buttonCallback, this, std::placeholders::_1));
  event_sub_ = node->create_subscription<std_msgs::msg::String>(
    "event", rclcpp::SensorDataQoS(), std::bind(&Handle::eventCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(), std::bind(&Handle::cmdVelCallback, this, std::placeholders::_1));
  servo_pos_sub_ = node->create_subscription<std_msgs::msg::Int16>(
    "servo_pos", rclcpp::SensorDataQoS(), std::bind(&Handle::servoPosCallback, this, std::placeholders::_1));
  turn_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "turn_pose", rclcpp::SensorDataQoS(), std::bind(&Handle::turnPoseCallback, this, std::placeholders::_1));
  turn_pose_prefer_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "turn_pose_prefer", rclcpp::SensorDataQoS(), std::bind(&Handle::turnPosePreferentialCallback, this, std::placeholders::_1));
  turn_type_sub_ = node->create_subscription<std_msgs::msg::String>(
    "turn_type", rclcpp::SensorDataQoS(), std::bind(&Handle::turnTypeCallback, this, std::placeholders::_1));
  rotation_complete_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "rotation_complete", rclcpp::SensorDataQoS(), std::bind(&Handle::rotationCompleteCallback, this, std::placeholders::_1));
  pause_control_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "pause_control", rclcpp::SensorDataQoS(), std::bind(&Handle::pauseControlCallback, this, std::placeholders::_1));
  change_di_control_mode_sub_ = node->create_subscription<std_msgs::msg::String>(
    "change_di_control_mode", rclcpp::SensorDataQoS(), std::bind(&Handle::changeDiControlModeCallback, this, std::placeholders::_1));
  local_plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/local_plan", rclcpp::SensorDataQoS(), std::bind(&Handle::localPlanCallback, this, std::placeholders::_1));
  plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::SensorDataQoS(), std::bind(&Handle::planCallback, this, std::placeholders::_1));

  for (int i = 0; i < 9; ++i) {
    rclcpp::Time zerotime(0, 0, RCL_ROS_TIME);
    last_up[i] = zerotime;
    last_dwn[i] = zerotime;
    last_holddwn[i] = zerotime;
    up_count[i] = 0;
    btn_dwn[i] = false;
  }
  is_navigating_ = false;
  is_servo_free_ = true;
  is_waiting_ = true;
  is_waiting_cnt_ = 0;
  recalculation_cnt_of_path = 0;
  last_turn_type_ = turn_type_::NORMAL;
  wma_filter_coef_ = -0.1f;
  wma_window_size_ = 3;
  di.control_mode = "both";
  di.reference_pose.header.frame_id = "base_footprint";
  di.reference_pose.pose.position.x = 0.0f;
  di.reference_pose.pose.position.y = 0.0f;
  di.reference_pose.pose.position.z = 0.0f;
  di.reference_pose.pose.orientation.x = 0.0f;
  di.reference_pose.pose.orientation.y = 0.0f;
  di.reference_pose.pose.orientation.z = 0.0f;
  di.reference_pose.pose.orientation.w = 1.0f;
  di.target_turn_angle = 0.0f;
  di.is_controlled_exclusive = false;
  di.current_servo_pos = 0;
  di.target_pos_global = 0;
  di.target_pos_local = 0;
  q_.setRPY(0, 0, 0);
  m_.setRotation(q_);
  tfBuffer = new tf2_ros::Buffer(node->get_clock());
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

  callbacks_[vib_keys::LEFT_TURN] = std::bind(&Handle::vibrateLeftTurn, this);
  callbacks_[vib_keys::RIGHT_TURN] = std::bind(&Handle::vibrateRightTurn, this);
  callbacks_[vib_keys::LEFT_DEV] = std::bind(&Handle::vibrateLeftDeviation, this);
  callbacks_[vib_keys::RIGHT_DEV] = std::bind(&Handle::vibrateRightDeviation, this);
  callbacks_[vib_keys::FRONT] = std::bind(&Handle::vibrateFront, this);
  callbacks_[vib_keys::LEFT_ABOUT_TURN] = std::bind(&Handle::vibrateAboutLeftTurn, this);
  callbacks_[vib_keys::RIGHT_ABOUT_TURN] = std::bind(&Handle::vibrateAboutRightTurn, this);
  callbacks_[vib_keys::BUTTON_CLICK] = std::bind(&Handle::vibrateButtonClick, this);
  callbacks_[vib_keys::BUTTON_HOLDDOWN] = std::bind(&Handle::vibrateButtonHolddown, this);
  callbacks_[vib_keys::CAUTION] = std::bind(&Handle::vibrateCautionPattern, this);
  callbacks_[vib_keys::NAVIGATION_START] = std::bind(&Handle::navigationStart, this);
  callbacks_[vib_keys::NAVIGATION_ARRIVED] = std::bind(&Handle::navigationArrived, this);

  vibration_timer_ = node->create_wall_timer(0.01s, std::bind(&Handle::timer_callback, this));
  vib_waiting_timer_ = node->create_wall_timer(1.0s, std::bind(&Handle::vib_waiting_timer_callback, this));
}

float Handle::getEulerYawDegrees(const double & x, const double & y, const double & z, const double & w)
{
  double roll, pitch, yaw;
  q_.setX(x);
  q_.setY(y);
  q_.setZ(z);
  q_.setW(w);
  m_.setRotation(q_);
  m_.getRPY(roll, pitch, yaw);
  RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "yaw: %f", yaw);
  return static_cast<float>(yaw * 180 / M_PI);
}

float Handle::getWeightedMovingAverage(const std::vector<float> & data)
{
  float sum_weighted_value = 0.0f;
  float sum_weight = 0.0f;
  float median = getMedian(data);
  for (float value : data) {
    float weight = exp(wma_filter_coef_ * std::abs(value - median));
    sum_weighted_value += value * weight;
    sum_weight += weight;
    RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "value: %f, weight: %f, median: %f", value, weight, median);
  }
  return 1.5f * sum_weighted_value / sum_weight;  // multiply 1.5 to emphasize the value of "di.target_pos_local"
}

float Handle::getMedian(const std::vector<float> & data)
{
  size_t data_size = data.size();
  std::vector<float> buf(data_size);
  std::copy(data.begin(), data.end(), std::back_inserter(buf));
  std::sort(buf.begin(), buf.end());
  if (data_size % 2 == 0) {
    return (buf[static_cast<size_t>(data_size / 2) - 1] + buf[static_cast<size_t>(data_size / 2)]) / 2.0f;
  } else {
    return buf[static_cast<size_t>(data_size / 2)];
  }
}

float Handle::getRotationDegree(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  float rotate_deg = 0.0f;
  tf2::Quaternion q_ref, q_target, q_rot;
  tf2::convert(p1.orientation, q_ref);
  tf2::convert(p2.orientation, q_target);
  q_rot = q_target * q_ref.inverse();
  q_rot.normalize();
  rotate_deg = getEulerYawDegrees(q_rot.x(), q_rot.y(), q_rot.z(), q_rot.w());
  return rotate_deg;
}

float Handle::getDistance(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  float diff_x = p2.position.x - p1.position.x;
  float diff_y = p2.position.y - p1.position.y;
  float diff_z = p2.position.z - p1.position.z;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}

bool Handle::isPoseMatching(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  float angle_diff = getRotationDegree(p1, p2);
  float distance_diff = getDistance(p1, p2);
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "angle_diff: %f, distance_diff: %f", angle_diff, distance_diff);
  if (std::abs(angle_diff) <= 10 && distance_diff <= 0.75f) {
    return true;
  } else {
    return false;
  }
}

void Handle::transformPoseToReferenceFrame(
  const geometry_msgs::msg::PoseStamped & pose_in, geometry_msgs::msg::PoseStamped & pose_out,
  const std::string reference_frame)
{
  if (pose_in.header.frame_id != reference_frame) {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Transform: <%s> to <%s>", pose_in.header.frame_id.c_str(), reference_frame.c_str());
      transformStamped = tfBuffer->lookupTransform(reference_frame, pose_in.header.frame_id, pose_in.header.stamp, rclcpp::Duration(1s));
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "%s", ex.what());
    }
    tf2::doTransform(pose_in, pose_out, transformStamped);
  } else {
    pose_out = pose_in;
  }
}

void Handle::setTurnAngle()
{
  geometry_msgs::msg::PoseStamped target_pose;
  if (!di.turn_angle_queue_prefer_.empty()) {
    transformPoseToReferenceFrame(di.turn_angle_queue_prefer_.front(), target_pose, di.reference_pose.header.frame_id);
    di.target_turn_angle = getRotationDegree(di.reference_pose.pose, target_pose.pose);
    di.turn_angle_queue_prefer_.erase(di.turn_angle_queue_prefer_.begin());
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "amount_of_rotation (preferential): %f", di.target_turn_angle);
  } else if (!di.turn_angle_queue_.empty()) {
    transformPoseToReferenceFrame(di.turn_angle_queue_.front(), target_pose, di.reference_pose.header.frame_id);
    di.target_turn_angle = getRotationDegree(di.reference_pose.pose, target_pose.pose);
    di.turn_angle_queue_.erase(di.turn_angle_queue_.begin());
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "amount_of_rotation: %f", di.target_turn_angle);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "There is NO pose in queue, this is bug.");
    return;
  }
  if (std::abs(di.target_turn_angle) >= 20.0f) {  // control dial when turn/rotation angle is greater than or equal to 20[deg]
    previous_pose_ = getCurrentPose();
    di.is_controlled_exclusive = true;
    changeServoPos(static_cast<int16_t>(di.target_turn_angle));
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "(global) target_yaw: %d", static_cast<int16_t>(di.target_turn_angle));
  } else {
    di.target_turn_angle = 0.0f;
  }
}

void Handle::checkTurnAngleQueue()
{
  auto node = node_.lock();
  if (!di.is_controlled_exclusive && (!di.turn_angle_queue_prefer_.empty() || !di.turn_angle_queue_.empty())) {
    setTurnAngle();
  }
  for (int idx = di.turn_angle_queue_.size() - 1; idx >= 0; idx--) {
    geometry_msgs::msg::PoseStamped target_pose;
    di.turn_angle_queue_[idx].header.stamp = node->get_clock()->now();
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "There is pose in target_queue, timestamp should be updated for doing transform");
    transformPoseToReferenceFrame(di.turn_angle_queue_[idx], target_pose, di.reference_pose.header.frame_id);
    if (isPoseMatching(target_pose.pose, di.reference_pose.pose)) {
      RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Drop head of queue because passed");
      di.turn_angle_queue_.erase(di.turn_angle_queue_.begin() + idx);
    }
  }
}

void Handle::timer_callback()
{
  if (vibration_queue_.size() > 0) {
    Vibration & vibration = vibration_queue_.front();
    RCLCPP_DEBUG(rclcpp::get_logger("handle"), "vibration.i = %d", vibration.i);
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "handle"), "vibration.numberVibration = %d", vibration.numberVibrations);
    if (vibration.numberVibrations == 0) {
      vibration_queue_.erase(vibration_queue_.begin());
      RCLCPP_INFO(rclcpp::get_logger("handle"), "Done");
    } else if (vibration.i == 0 && vibration.numberVibrations > 0) {
      std::unique_ptr<std_msgs::msg::UInt8> msg = std::make_unique<std_msgs::msg::UInt8>();
      msg->data = vibration.duration * 0.1;
      vibration.vibratorPub->publish(std::move(msg));
      RCLCPP_INFO(rclcpp::get_logger("handle"), "publish %d", vibration.duration);
      RCLCPP_INFO(rclcpp::get_logger("handle"), "sleep %d ms", vibration.sleep);
      vibration.i++;
    } else if (vibration.i == vibration.duration * 0.1 && vibration.numberVibrations == 1) {
      vibration.numberVibrations = 0;
    } else if (vibration.i < (vibration.duration + vibration.sleep) * 0.1) {
      vibration.i++;
    } else {
      vibration.i = 0;
      vibration.numberVibrations--;
    }
  }
}

void Handle::vib_waiting_timer_callback()
{
  if (is_waiting_) {
    if (is_waiting_cnt_ < 2) {  // 1.0sec(vib_waiting_timer_) * 2 = 2.0sec
      is_waiting_cnt_++;
    } else {
      // setServoFree(true);
      // vibrateWaitingPattern();
    }
  } else {
    if (is_waiting_cnt_ > 0) {
      is_waiting_cnt_ = 0;
      vibration_queue_.clear();
    }
  }
}

void Handle::buttonCallback(std_msgs::msg::Int8::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "node_ is null in buttonCallback");
    return;
  }
  for (int index = 1; index <= 5; ++index) {
    buttonCheck((msg->data >> (index - 1)) & 0x01, index);
  }
}

void Handle::buttonCheck(bool btn_push, int index)
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "node_ is null in buttonCallback");
    return;
  }
  std::map<std::string, int> event;

  rclcpp::Time now = node->get_clock()->now();
  rclcpp::Time zerotime(0, 0, RCL_ROS_TIME);
  if (btn_push && !btn_dwn[index] &&
    !(last_up[index] != zerotime && now - last_up[index] < ignore_interval_))
  {
    event["button"] = index;
    event["up"] = false;  // 0
    btn_dwn[index] = true;
    last_dwn[index] = now;
  }
  if (!btn_push && btn_dwn[index]) {
    event["button"] = index;
    event["up"] = true;  // 1
    up_count[index]++;
    last_up[index] = now;
    btn_dwn[index] = false;
  }
  if (last_up[index] != zerotime &&
    !btn_dwn[index] &&
    now - last_up[index] > double_click_interval_)
  {
    if (last_holddwn[index] == zerotime) {
      event["buttons"] = index;
      event["count"] = up_count[index];
    }
    last_up[index] = zerotime;
    last_holddwn[index] = zerotime;
    up_count[index] = 0;
  }
  if (btn_push && btn_dwn[index] &&
    last_dwn[index] != zerotime &&
    (now - last_dwn[index] > holddown_min_interval_ &&
    now - last_holddwn[index] > holddown_interval_ &&
    now - last_dwn[index] < holddown_max_interval_))
  {
    event["holddown"] = index;
    event["duration"] = static_cast<int>((now - last_dwn[index]).seconds());
    last_holddwn[index] = now;
  }
  if (!event.empty()) {
    eventListener_(event);
  }
}

void Handle::eventCallback(std_msgs::msg::String::SharedPtr msg)
{
  const std::string name = msg->data;
  auto it = std::find(stimuli_names.begin(), stimuli_names.end(), name.c_str());
  if (it != stimuli_names.end()) {
    int index = std::distance(stimuli_names.begin(), it);
    executeStimulus(index);
  } else {
    RCLCPP_DEBUG(logger_, "Stimulus '%s' not found.", name.c_str());
  }
}

void Handle::executeStimulus(int index)
{
  RCLCPP_INFO(logger_, "execute_stimulus, %d", index);
  const std::size_t size = stimuli_names.size();
  if (index >= 0 && index < static_cast<int>(size) && callbacks_[index]) {
    callbacks_[index]();
    RCLCPP_INFO(logger_, "executed");
  }
}

void Handle::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::vector<double> linear = {msg->linear.x, msg->linear.y, msg->linear.z};
  std::vector<double> angular = {msg->angular.x, msg->angular.y, msg->angular.z};
  if ((linear == std::vector<double> {0.0, 0.0, 0.0}) && (angular == std::vector<double> {0.0, 0.0, 0.0})) {
    is_waiting_ = true;
  } else {
    is_waiting_ = false;
  }
}

void Handle::servoPosCallback(std_msgs::msg::Int16::SharedPtr msg)
{
  di.current_servo_pos = msg->data;
  RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "current servo_pos: %d", di.current_servo_pos);
  checkTurnAngleQueue();
  if (di.is_controlled_exclusive) {
    current_pose_ = getCurrentPose();
    float turned_angle = getRotationDegree(previous_pose_, current_pose_);  // The angle that the Cabot has already turned
    di.target_pos_global = static_cast<int16_t>(di.target_turn_angle - turned_angle);
    if (std::abs(di.target_pos_global) < di.THRESHOLD_RESET) {
      if (std::abs(di.target_pos_global - di.target_pos_local) < di.THRESHOLD_PASS_CONTROL_MIN) {
        // resetServoPosition();
        di.is_controlled_exclusive = false;
        RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "(global -> local) global: %d, local: %d ", di.target_pos_global, di.target_pos_local);
      } else {
        RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "(global -> local) waiting for pass control, global: %d, local: %d", di.target_pos_global, di.target_pos_local);
      }
    } else {
      if (std::abs(di.target_pos_global - di.target_pos_local) > di.THRESHOLD_PASS_CONTROL_MAX) {
        resetServoPosition();
        RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "(global -> local) over limit, global: %d, local: %d", di.target_pos_global, di.target_pos_local);
      } else {
        changeServoPos(di.target_pos_global);
        RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "(global) global: %d, local: %d", di.target_pos_global, di.target_pos_local);
      }
    }
  }
}

void Handle::turnPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.header = msg->header;
  orig_pose.pose = msg->pose;
  if (di.control_mode == "both" || di.control_mode == "global") {
    di.turn_angle_queue_.push_back(orig_pose);
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Append turn_angle que");
    if (!di.is_controlled_exclusive) {
      setTurnAngle();
    } else {
      RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Wait for finish preferential rotate command");
    }
  }
}

void Handle::turnPosePreferentialCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.header = msg->header;
  orig_pose.pose = msg->pose;
  if (di.control_mode == "both" || di.control_mode == "global") {
    di.turn_angle_queue_prefer_.push_back(orig_pose);
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Append preferential turn_angle que");
    setTurnAngle();
  }
}

void Handle::turnTypeCallback(std_msgs::msg::String::SharedPtr msg)
{
  std::string turn_type = msg->data;
  if (turn_type == "Type.Normal") {
    last_turn_type_ = turn_type_::NORMAL;
    RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "Turn_type: Normal");
  } else if (turn_type == "Type.Avoiding") {
    last_turn_type_ = turn_type_::AVOIDING;
    RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "Turn_type: Avoiding");
  }
}

void Handle::rotationCompleteCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  bool is_completed_rotation = msg->data;
  if (is_completed_rotation) {
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "rotation completed");
    resetServoPosition();
  }
}

void Handle::pauseControlCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && di.current_servo_pos != 0) {
    is_navigating_ = false;
    wma_data_buffer_.clear();
    di.turn_angle_queue_.clear();
    di.turn_angle_queue_prefer_.clear();
    resetServoPosition();
    for (uint8_t i = 0; i < 5; i++) {
      RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "wait for reset dial position");
      changeServoPos(0);
    }
    setServoFree(true);
  }
}

void Handle::localPlanCallback(nav_msgs::msg::Path::SharedPtr msg)
{
  if (di.control_mode == "both" || di.control_mode == "local") {
    size_t local_plan_len = msg->poses.size();
    if (local_plan_len > 1) {
      geometry_msgs::msg::PoseStamped start_pose = msg->poses[0];
      geometry_msgs::msg::PoseStamped end_pose = msg->poses[static_cast<size_t>(local_plan_len / 2) - 1];
      float di_target = getRotationDegree(start_pose.pose, end_pose.pose);
      if (wma_data_buffer_.size() >= wma_window_size_) {
        wma_data_buffer_.erase(wma_data_buffer_.begin());
        wma_data_buffer_.push_back(di_target);
      } else {
        wma_data_buffer_.push_back(di_target);
      }
      di.target_pos_local = static_cast<int16_t>(getWeightedMovingAverage(wma_data_buffer_));
      if (!di.is_controlled_exclusive) {
        if (std::abs(di.target_pos_local) >= di.THRESHOLD_RESET) {
          changeServoPos(di.target_pos_local);
          RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "(local) target: %d", di.target_pos_local);
        } else {
          if (di.current_servo_pos != 0) {
            resetServoPosition();
          }
        }
      }
    }
  }
}

void Handle::planCallback(nav_msgs::msg::Path::SharedPtr msg)
{
  if (di.is_controlled_exclusive && (di.control_mode == "both" || di.control_mode == "global")) {
    recalculation_cnt_of_path += 1;
    if (recalculation_cnt_of_path >= 2) {
      di.is_controlled_exclusive = false;
      RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "change di control mode because plan calculated repeatedly");
    }
  }
}

void Handle::changeDiControlModeCallback(std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "both" || msg->data == "global" || msg->data == "local" || msg->data == "none") {
    di.control_mode = msg->data;
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "success change di control mode: %s", msg->data.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "failed change di control mode: %s", msg->data.c_str());
  }
}

void Handle::changeServoPos(int16_t target_pos)
{
  setServoFree(false);
  std::unique_ptr<std_msgs::msg::Int16> msg = std::make_unique<std_msgs::msg::Int16>();
  msg->data = -1 * target_pos;
  RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "send to servo: %d", msg->data);
  servo_target_pub_->publish(std::move(msg));
}

void Handle::setServoFree(bool is_free)
{
  std::unique_ptr<std_msgs::msg::Bool> msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = is_free;
  is_servo_free_ = is_free;
  RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "servo_free: %d", msg->data);
  servo_free_pub_->publish(std::move(msg));
}

void Handle::resetServoPosition()
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Reset: servo_pos");
  di.is_controlled_exclusive = false;
  di.target_turn_angle = 0.0f;
  di.target_pos_global = 0.0f;
  recalculation_cnt_of_path = 0;
  changeServoPos(0);
}

void Handle::navigationArrived()
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Navigation arrived");
  is_navigating_ = false;
  wma_data_buffer_.clear();
  di.turn_angle_queue_.clear();
  di.turn_angle_queue_prefer_.clear();
  resetServoPosition();
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator1_pub_, VibConst::ERM::NumVibrations::HAS_ARRIVED, VibConst::ERM::Duration::HAS_ARRIVED, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator1_pub_, VibConst::LRA::NumVibrations::HAS_ARRIVED, VibConst::LRA::Duration::HAS_ARRIVED, VibConst::LRA::Sleep::DEFAULT);
  }
  for (uint8_t i = 0; i < 5; i++) {
    RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "wait for reset dial position");
    changeServoPos(0);
  }
  setServoFree(true);
}

void Handle::navigationStart()
{
  is_navigating_ = true;
  wma_data_buffer_.clear();
  di.turn_angle_queue_.clear();
  di.turn_angle_queue_prefer_.clear();
  resetServoPosition();
}

void Handle::vibrateLeftTurn()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator3_pub_, VibConst::ERM::NumVibrations::TURN, VibConst::ERM::Duration::TURN, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator3_pub_, VibConst::LRA::NumVibrations::TURN, VibConst::LRA::Duration::TURN, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateRightTurn()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator4_pub_, VibConst::ERM::NumVibrations::TURN, VibConst::ERM::Duration::TURN, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator4_pub_, VibConst::LRA::NumVibrations::TURN, VibConst::LRA::Duration::TURN, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateLeftDeviation()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator3_pub_, VibConst::ERM::NumVibrations::DEVIATION, VibConst::ERM::Duration::DEVIATION, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator3_pub_, VibConst::LRA::NumVibrations::DEVIATION, VibConst::LRA::Duration::DEVIATION, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateRightDeviation()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator4_pub_, VibConst::ERM::NumVibrations::DEVIATION, VibConst::ERM::Duration::DEVIATION, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator4_pub_, VibConst::LRA::NumVibrations::DEVIATION, VibConst::LRA::Duration::DEVIATION, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateFront()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator1_pub_, VibConst::ERM::NumVibrations::CONFIRMATION, VibConst::ERM::Duration::SINGLE_VIBRATION, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator1_pub_, VibConst::LRA::NumVibrations::CONFIRMATION, VibConst::LRA::Duration::SINGLE_VIBRATION, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateAboutLeftTurn()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator3_pub_, VibConst::ERM::NumVibrations::ABOUT_TURN, VibConst::ERM::Duration::ABOUT_TURN, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator3_pub_, VibConst::LRA::NumVibrations::ABOUT_TURN, VibConst::LRA::Duration::ABOUT_TURN, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateAboutRightTurn()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator4_pub_, VibConst::ERM::NumVibrations::ABOUT_TURN, VibConst::ERM::Duration::ABOUT_TURN, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator4_pub_, VibConst::LRA::NumVibrations::ABOUT_TURN, VibConst::LRA::Duration::ABOUT_TURN, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateBack()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator1_pub_, VibConst::ERM::NumVibrations::CONFIRMATION, VibConst::ERM::Duration::SINGLE_VIBRATION, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator1_pub_, VibConst::LRA::NumVibrations::CONFIRMATION, VibConst::LRA::Duration::SINGLE_VIBRATION, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateButtonClick()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator1_pub_, VibConst::ERM::NumVibrations::BUTTON_CLICK, VibConst::ERM::Duration::BUTTON_CLICK, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator1_pub_, VibConst::LRA::NumVibrations::BUTTON_CLICK, VibConst::LRA::Duration::BUTTON_CLICK, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateButtonHolddown()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator1_pub_, VibConst::ERM::NumVibrations::BUTTON_HOLDDOWN, VibConst::ERM::Duration::BUTTON_HOLDDOWN, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator1_pub_, VibConst::LRA::NumVibrations::BUTTON_HOLDDOWN, VibConst::LRA::Duration::BUTTON_HOLDDOWN, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::vibrateCautionPattern()
{
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator1_pub_, VibConst::ERM::NumVibrations::CAUTION, VibConst::ERM::Duration::CAUTION, VibConst::ERM::Sleep::CAUTION);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator1_pub_, VibConst::LRA::NumVibrations::CAUTION, VibConst::LRA::Duration::CAUTION, VibConst::LRA::Sleep::CAUTION);
  }
}

void Handle::vibrateWaitingPattern()
{
  Vibration vibration;
  if (vibratorType_ == vibrator_type_::ERM) {
    vibration.numberVibrations = VibConst::ERM::NumVibrations::WAITING;
    vibration.duration = VibConst::ERM::Duration::WAITING;
    vibration.sleep = VibConst::ERM::Sleep::WAITING;
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibration.numberVibrations = VibConst::LRA::NumVibrations::WAITING;
    vibration.duration = VibConst::LRA::Duration::WAITING;
    vibration.sleep = VibConst::LRA::Sleep::WAITING;
  }
  vibration.vibratorPub = vibrator1_pub_;
  vibration_queue_.push_back(vibration);
}

void Handle::vibratePattern(
  const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & vibratorPub,
  unsigned int numberVibrations, unsigned int duration, unsigned int sleep)
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Start vibratePattern.");
  Vibration vibration;
  vibration.numberVibrations = numberVibrations;
  vibration.duration = duration;
  vibration.sleep = sleep;
  vibration.vibratorPub = vibratorPub;
  if (is_waiting_) {
    vibration_queue_.insert(vibration_queue_.begin(), vibration);
  } else {
    vibration_queue_.push_back(vibration);
  }
}

geometry_msgs::msg::Pose Handle::getCurrentPose()
{
  geometry_msgs::msg::Pose current_pose;
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped = tfBuffer->lookupTransform("map_global", "base_footprint", rclcpp::Time(0));
  tf2::convert(transformStamped.transform.rotation, current_pose.orientation);
  return current_pose;
}
