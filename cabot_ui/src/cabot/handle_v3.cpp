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
const rclcpp::Duration Handle::holddown_max_interval_ = rclcpp::Duration(5, 100000000);  // 5.1 sec (margin=0.1sec)
const rclcpp::Duration Handle::holddown_interval_ = rclcpp::Duration(1, 0);

std::string Handle::get_name(int stimulus)
{
  return stimuli_names[stimulus];
}

Handle::Handle(
  CaBotHandleV3Node * node,
  std::function<void(const std::map<std::string, std::string> &)> eventListener,
  const std::vector<std::string> & buttonKeys, const int & vibratorType)
: node_(node), eventListener_(std::move(eventListener)), buttonKeys_(buttonKeys),
  vibratorType_(vibratorType), logger_(rclcpp::get_logger("Handle_v3"))
{
  vibrator1_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator1", 100);
  vibrator2_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator2", 100);
  vibrator3_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator3", 100);
  vibrator4_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("vibrator4", 100);
  servo_free_pub_ = node_->create_publisher<std_msgs::msg::Bool>("servo_free", rclcpp::QoS(1));
  servo_target_pub_ = node_->create_publisher<std_msgs::msg::Int16>("servo_target", rclcpp::QoS(10));
  button_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
    "pushed", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Int8::UniquePtr msg) {
      buttonCallback(msg);
    });
  event_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "event", rclcpp::SensorDataQoS(), [this](std_msgs::msg::String::UniquePtr msg) {
      eventCallback(std::move(msg));
    });
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::Twist::UniquePtr msg) {
      cmdVelCallback(msg);
    });
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::UniquePtr msg) {
      handleImuCallback(msg);
    });
  servo_pos_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
    "servo_pos", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Int16::UniquePtr msg) {
      servoPosCallback(msg);
    });
  turn_angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "turn_angle", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Float32::UniquePtr msg) {
      turnAngleCallback(msg);
    });
  turn_type_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "turn_type", rclcpp::SensorDataQoS(), [this](std_msgs::msg::String::UniquePtr msg) {
      turnTypeCallback(msg);
    });
  turn_end_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "turn_end", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Bool::UniquePtr msg) {
      turnEndCallback(msg);
    });
  change_di_control_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "change_di_control_mode", rclcpp::SensorDataQoS(), [this](std_msgs::msg::String::UniquePtr msg) {
      changeDiControlModeCallback(msg);
    });
  local_plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/local_plan", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Path::UniquePtr msg) {
      localPlanCallback(msg);
    });
  angular_distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/angular_distance", rclcpp::SensorDataQoS(), [this](std_msgs::msg::Float64::UniquePtr msg) {
      angularDistanceCallback(msg);
    });
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
  last_turn_type_ = turn_type_::NORMAL;
  current_imu_yaw_ = 0.0f;
  previous_imu_yaw_ = 0.0f;  // yaw angle at turn start position
  wma_filter_coef_ = -0.1f;
  wma_window_size_ = 3;
  di.control_mode = "both";
  di.target_turn_angle = 0.0f;
  di.is_controlled_by_imu = false;
  di.target_pos_global = 0;
  di.target_pos_local = 0;
  q_.setRPY(0, 0, 0);
  m_.setRotation(q_);

  callbacks_.resize(stimuli_names.size(), nullptr);
  callbacks_[1] = [this]() {
      vibrateLeftTurn();
    };
  callbacks_[2] = [this]() {
      vibrateRightTurn();
    };
  callbacks_[3] = [this]() {
      vibrateLeftDeviation();
    };
  callbacks_[4] = [this]() {
      vibrateRightDeviation();
    };
  callbacks_[5] = [this]() {
      vibrateFront();
    };
  callbacks_[6] = [this]() {
      vibrateAboutLeftTurn();
    };
  callbacks_[7] = [this]() {
      vibrateAboutRightTurn();
    };
  callbacks_[8] = [this]() {
      vibrateButtonClick();
    };
  callbacks_[9] = [this]() {
      vibrateButtonHolddown();
    };
  callbacks_[10] = [this]() {
      vibrateCautionPattern();
    };
  callbacks_[11] = [this]() {
      navigationStart();
    };
  callbacks_[12] = [this]() {
      navigationArrived();
    };
  vibration_timer_ = node_->create_wall_timer(0.01s, std::bind(&Handle::timer_callback, this));
  vib_waiting_timer_ = node_->create_wall_timer(1.0s, std::bind(&Handle::vib_waiting_timer_callback, this));
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

void Handle::buttonCallback(std_msgs::msg::Int8::UniquePtr & msg)
{
  for (int index = 1; index <= 5; ++index) {
    if (index >= 1 && index <= static_cast<int>(ButtonType::BUTTON_CENTER)) {
      buttonCheck(msg, index);
    }
  }
}

void Handle::buttonCheck(std_msgs::msg::Int8::UniquePtr & msg, int index)
{
  event.clear();
  int bit = 1 << (index - 1);
  bool btn_push = (msg->data & bit) != 0;
  rclcpp::Time now = node_->get_clock()->now();
  rclcpp::Time zerotime(0, 0, RCL_ROS_TIME);
  if (btn_push && !btn_dwn[index] &&
    !(last_up[index] != zerotime && now - last_up[index] < ignore_interval_))
  {
    event.insert(std::pair<std::string, std::string>("button", std::to_string(button_keys(index))));
    event.insert(std::pair<std::string, std::string>("up", "False"));
    btn_dwn[index] = true;
    last_dwn[index] = now;
  }
  if (!btn_push && btn_dwn[index]) {
    event.insert(std::pair<std::string, std::string>("button", std::to_string(button_keys(index))));
    event.insert(std::pair<std::string, std::string>("up", "True"));
    up_count[index]++;
    last_up[index] = now;
    btn_dwn[index] = false;
  }
  if (last_up[index] != zerotime &&
    !btn_dwn[index] &&
    now - last_up[index] > double_click_interval_)
  {
    if (last_holddwn[index] == zerotime) {
      event.insert(std::pair<std::string, std::string>("buttons", std::to_string(button_keys(index))));
      event.insert(std::pair<std::string, std::string>("count", std::to_string(up_count[index])));
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
    event.insert(std::pair<std::string, std::string>("holddown", std::to_string(button_keys(index))));
    int duration = static_cast<int>((now - last_dwn[index]).seconds());
    event.insert(std::pair<std::string, std::string>("duration", std::to_string(duration)));
    last_holddwn[index] = now;
  }
  if (!event.empty()) {
    eventListener_(event);
  }
}

void Handle::eventCallback(std_msgs::msg::String::UniquePtr msg)
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

void Handle::cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr & msg)
{
  std::vector<double> linear = {msg->linear.x, msg->linear.y, msg->linear.z};
  std::vector<double> angular = {msg->angular.x, msg->angular.y, msg->angular.z};
  if ((linear == std::vector<double> {0.0, 0.0, 0.0}) && (angular == std::vector<double> {0.0, 0.0, 0.0})) {
    is_waiting_ = true;
  } else {
    is_waiting_ = false;
  }
}

void Handle::handleImuCallback(sensor_msgs::msg::Imu::UniquePtr & msg)
{
  current_imu_yaw_ = getEulerYawDegrees(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void Handle::servoPosCallback(std_msgs::msg::Int16::UniquePtr & msg)
{
  if (di.is_controlled_by_imu) {
    float turned_angle = current_imu_yaw_ - previous_imu_yaw_;  // Angle at which the Cabot has already turned
    if (turned_angle >= 180.0f) {
      turned_angle -= 360.0f;
    } else if (turned_angle < -180.0f) {
      turned_angle += 360.0f;
    }
    di.target_pos_global = static_cast<int16_t>(di.target_turn_angle - turned_angle);
    if (std::abs(di.target_pos_global) < di.THRESHOLD_RESET) {
      if (std::abs(di.target_pos_global - di.target_pos_local) < di.THRESHOLD_PASS_CONTROL_MIN) {
        resetServoPosition();
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

void Handle::turnAngleCallback(std_msgs::msg::Float32::UniquePtr & msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "turn_angle: %f", msg->data);
  if (std::abs(msg->data) >= 60.0f) {  // thtMinimumTurn = 60
    if (di.control_mode == "both" || di.control_mode == "global") {
      di.target_turn_angle = msg->data;
      previous_imu_yaw_ = current_imu_yaw_;
      di.is_controlled_by_imu = true;
      changeServoPos(static_cast<int16_t>(di.target_turn_angle));
      RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "(global) target_yaw: %d", static_cast<int16_t>(di.target_turn_angle));
      RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "(global) start_yaw: %f", previous_imu_yaw_);
    }
  }
}

void Handle::turnTypeCallback(std_msgs::msg::String::UniquePtr & msg)
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

void Handle::turnEndCallback(std_msgs::msg::Bool::UniquePtr & msg)
{
  bool is_turn_end = msg->data;
  if (is_turn_end) {
    if (di.control_mode == "both" || di.control_mode == "local") {
      di.is_controlled_by_imu = false;
    } else {
      resetServoPosition();
    }
  }
}

void Handle::localPlanCallback(nav_msgs::msg::Path::UniquePtr & msg)
{
  if (di.control_mode == "both" || di.control_mode == "local") {
    size_t local_plan_len = msg->poses.size();
    if (local_plan_len > 1) {
      geometry_msgs::msg::PoseStamped start_pose = msg->poses[0];
      geometry_msgs::msg::PoseStamped end_pose = msg->poses[static_cast<size_t>(local_plan_len / 2) - 1];
      float start_pose_yaw = getEulerYawDegrees(start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w);
      float end_pose_yaw = getEulerYawDegrees(end_pose.pose.orientation.x, end_pose.pose.orientation.y, end_pose.pose.orientation.z, end_pose.pose.orientation.w);
      float di_target = end_pose_yaw - start_pose_yaw;
      if (di_target > 180.0f) {
        di_target -= 360.0f;
      } else if (di_target < -180.0f) {
        di_target += 360.0f;
      }
      if (wma_data_buffer_.size() >= wma_window_size_) {
        wma_data_buffer_.erase(wma_data_buffer_.begin());
        wma_data_buffer_.push_back(di_target);
      } else {
        wma_data_buffer_.push_back(di_target);
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "(local) start: %f, end: %f, diff: %f", start_pose_yaw, end_pose_yaw, di_target);
      di.target_pos_local = static_cast<int16_t>(getWeightedMovingAverage(wma_data_buffer_));
      if (!di.is_controlled_by_imu) {
        if (std::abs(di.target_pos_local) >= di.THRESHOLD_RESET) {
          changeServoPos(di.target_pos_local);
          RCLCPP_DEBUG(rclcpp::get_logger("Handle_v3"), "(local) target: %d", di.target_pos_local);
        } else {
          resetServoPosition();
        }
      }
    }
  }
}

void Handle::angularDistanceCallback(std_msgs::msg::Float64::UniquePtr & msg)
{
  if (di.control_mode == "both" || di.control_mode == "local") {
    double angular_data = msg->data;
    float di_target = static_cast<float>(angular_data) * 180 / M_PI;
    RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "di control: %f", di_target);
    if (!di.is_controlled_by_imu) {
      changeServoPos(static_cast<int16_t>(di_target));
    }
  }
}

void Handle::changeDiControlModeCallback(std_msgs::msg::String::UniquePtr & msg)
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
  servo_target_pub_->publish(std::move(msg));
}

void Handle::setServoFree(bool is_free)
{
  std::unique_ptr<std_msgs::msg::Bool> msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = is_free;
  is_servo_free_ = is_free;
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "servo_free: %d", msg->data);
  servo_free_pub_->publish(std::move(msg));
}

void Handle::navigationArrived()
{
  is_navigating_ = false;
  wma_data_buffer_.clear();
  resetServoPosition();
  setServoFree(true);
  if (vibratorType_ == vibrator_type_::ERM) {
    vibratePattern(vibrator1_pub_, VibConst::ERM::NumVibrations::HAS_ARRIVED, VibConst::ERM::Duration::HAS_ARRIVED, VibConst::ERM::Sleep::DEFAULT);
  } else if (vibratorType_ == vibrator_type_::LRA) {
    vibratePattern(vibrator1_pub_, VibConst::LRA::NumVibrations::HAS_ARRIVED, VibConst::LRA::Duration::HAS_ARRIVED, VibConst::LRA::Sleep::DEFAULT);
  }
}

void Handle::navigationStart()
{
  is_navigating_ = true;
  wma_data_buffer_.clear();
  resetServoPosition();
}

void Handle::resetServoPosition()
{
  RCLCPP_INFO(rclcpp::get_logger("Handle_v3"), "Reset: servo_pos");
  di.is_controlled_by_imu = false;
  di.target_turn_angle = 0.0f;
  di.target_pos_global = 0.0f;
  changeServoPos(0);
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
