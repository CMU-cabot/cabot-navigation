// Copyright (c) 2023  Miraikan and Carnegie Mellon University
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

#include "handle_v2.hpp"

using namespace std::chrono_literals;

const std::vector<std::string> Handle::stimuli_names =
{"unknown", "left_turn", "right_turn", "left_dev", "right_dev", "front",
  "left_about_turn", "right_about_turn", "button_click", "button_holddown"};
const rclcpp::Duration Handle::double_click_interval_ = rclcpp::Duration(0, 250000000);  // 250 msec
const rclcpp::Duration Handle::ignore_interval_ = rclcpp::Duration(0, 50000000);  // 50 msec
const rclcpp::Duration Handle::holddown_min_interval_ = rclcpp::Duration(1, 0);
const rclcpp::Duration Handle::holddown_max_interval_ = rclcpp::Duration(50, 500000000);  // 50.5 sec (margin=0.5sec)
const rclcpp::Duration Handle::holddown_interval_ = rclcpp::Duration(1, 0);

std::string Handle::get_name(int stimulus)
{
  return stimuli_names[stimulus];
}

Handle::Handle(
  std::shared_ptr<CaBotHandleV2Node> node,  // Use shared_ptr here
  std::function<void(const std::map<std::string, int> &)> eventListener)
: node_(node), eventListener_(std::move(eventListener)),
  logger_(rclcpp::get_logger("handle"))
{
  power_ = 255;
  vibrator1_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator1", 100);
  vibrator2_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator2", 100);
  vibrator3_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator3", 100);
  vibrator4_pub_ = node->create_publisher<std_msgs::msg::UInt8>("vibrator4", 100);
  for (int i = 0; i < 9; ++i) {
    rclcpp::Time zerotime(0, 0, RCL_ROS_TIME);
    last_up[i] = zerotime;
    last_dwn[i] = zerotime;
    last_holddwn[i] = zerotime;
    up_count[i] = 0;
    btn_dwn[i] = false;
  }
  button_sub_ = node->create_subscription<std_msgs::msg::Int8>(
    "pushed", rclcpp::SensorDataQoS(), std::bind(&Handle::buttonCallback, this, std::placeholders::_1));
  event_sub_ = node->create_subscription<std_msgs::msg::String>(
    "event", rclcpp::SensorDataQoS(), std::bind(&Handle::eventCallback, this, std::placeholders::_1));
  duration_ = 150;
  duration_single_vibration_ = 400;
  duration_about_turn_ = 400;
  duration_button_click_ = 50;
  duration_button_holddown_ = 100;
  sleep_ = 150;
  num_vibrations_turn_ = 4;
  num_vibrations_deviation_ = 2;
  num_vibrations_about_turn_ = 2;
  num_vibrations_confirmation_ = 1;
  num_vibrations_button_click_ = 1;
  num_vibrations_button_holddown_ = 1;

  callbacks_[vib_keys::LEFT_TURN] = std::bind(&Handle::vibrateLeftTurn, this);
  callbacks_[vib_keys::RIGHT_TURN] = std::bind(&Handle::vibrateRightTurn, this);
  callbacks_[vib_keys::LEFT_DEV] = std::bind(&Handle::vibrateLeftDeviation, this);
  callbacks_[vib_keys::RIGHT_DEV] = std::bind(&Handle::vibrateRightDeviation, this);
  callbacks_[vib_keys::FRONT] = std::bind(&Handle::vibrateFront, this);
  callbacks_[vib_keys::LEFT_ABOUT_TURN] = std::bind(&Handle::vibrateAboutLeftTurn, this);
  callbacks_[vib_keys::RIGHT_ABOUT_TURN] = std::bind(&Handle::vibrateAboutRightTurn, this);
  callbacks_[vib_keys::BUTTON_CLICK] = std::bind(&Handle::vibrateButtonClick, this);
  callbacks_[vib_keys::BUTTON_HOLDDOWN] = std::bind(&Handle::vibrateButtonHolddown, this);
  
  vibration_timer_ = node->create_wall_timer(0.01s, std::bind(&Handle::timer_callback, this));
}

Handle::~Handle()
{
  RCLCPP_INFO(logger_, "Destroying Handle");
  if (!vibration_queue_.empty()) {
    RCLCPP_WARN(logger_, "Clearing non-empty vibration_queue_");
  }
  vibration_queue_.clear();  // Clear vibration queue to release resources
  node_.reset();  // Reset the shared pointer to avoid dangling references
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
      RCLCPP_INFO(rclcpp::get_logger("handle"), "sleep %d ms", vibration.duration);
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
  const std::size_t size = sizeof(stimuli_names) / sizeof(stimuli_names[0]);
  if (index >= 0 && index < static_cast<int>(size) && callbacks_[index]) {
    callbacks_[index]();
    RCLCPP_INFO(logger_, "executed");
  }
}

void Handle::vibrateLeftTurn()
{
  vibratePattern(vibrator3_pub_, num_vibrations_turn_, duration_);
}

void Handle::vibrateRightTurn()
{
  vibratePattern(vibrator4_pub_, num_vibrations_turn_, duration_);
}

void Handle::vibrateLeftDeviation()
{
  vibratePattern(vibrator3_pub_, num_vibrations_deviation_, duration_);
}

void Handle::vibrateRightDeviation()
{
  vibratePattern(vibrator4_pub_, num_vibrations_deviation_, duration_);
}

void Handle::vibrateFront()
{
  vibratePattern(vibrator1_pub_, num_vibrations_confirmation_, duration_single_vibration_);
}

void Handle::vibrateAboutLeftTurn()
{
  vibratePattern(vibrator3_pub_, num_vibrations_about_turn_, duration_about_turn_);
}

void Handle::vibrateAboutRightTurn()
{
  vibratePattern(vibrator4_pub_, num_vibrations_about_turn_, duration_about_turn_);
}

void Handle::vibrateBack()
{
  vibratePattern(vibrator2_pub_, num_vibrations_confirmation_, duration_single_vibration_);
}

void Handle::vibrateButtonClick()
{
  vibratePattern(vibrator1_pub_, num_vibrations_button_click_, duration_button_click_);
}

void Handle::vibrateButtonHolddown()
{
  vibratePattern(vibrator1_pub_, num_vibrations_button_holddown_, duration_button_holddown_);
}

void Handle::vibratePattern(
  const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & vibratorPub,
  int numberVibrations, int duration)
{
  RCLCPP_INFO(rclcpp::get_logger("handle"), "Start vibratePattern .");
  Vibration vibration;
  vibration.duration = duration;
  vibration.numberVibrations = numberVibrations;
  vibration.sleep = sleep_;
  vibration.vibratorPub = vibratorPub;
  vibration_queue_.push_back(vibration);
}
