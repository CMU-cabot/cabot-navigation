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

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <utility>

#include "cabot_handle_v3_node.hpp"

std::shared_ptr<CaBotHandleV3Node> node_;

CaBotHandleV3Node::CaBotHandleV3Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("cabot_handle_v3_node", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  handle_(nullptr), button_keys_({}), event_pub_(nullptr), vibrator_type_(1)
{
  event_pub_ = create_publisher<std_msgs::msg::String>("/cabot/event", rclcpp::QoS(10));
  button_keys_ = declare_parameter("buttons", std::vector<std::string>{""});
  std::string button_keys_str = std::accumulate(
    button_keys_.begin(), button_keys_.end(), std::string(),
    [](const std::string & result, const std::string & key) {
      return result.empty() ? key : result + ", " + key;
    });
  eventListener_callback = [this](const std::map<std::string, std::string> & msg) {
      this->eventListener(msg);
    };
  declare_parameter("vibrator_type", vibrator_type_);
  vibrator_type_ = get_parameter("vibrator_type").as_int();
  RCLCPP_INFO(get_logger(), "vibrator_type: %d", vibrator_type_);
  handle_ = std::make_unique<Handle>(this, eventListener_callback, button_keys_, vibrator_type_);
  RCLCPP_INFO(get_logger(), "buttons: %s", button_keys_str.c_str());
  bool no_vibration = declare_parameter("no_vibration", false);
  RCLCPP_INFO(get_logger(), "no_vibration = %s", no_vibration ? "true" : "false");
  notification_callback = [this](const std_msgs::msg::Int8::UniquePtr msg) {
      this->notificationCallback(std::move(msg));
    };
  if (!no_vibration) {
    notification_sub = create_subscription<std_msgs::msg::Int8>(
      "/cabot/notification", 10, notification_callback);
  }
}

void CaBotHandleV3Node::printStackTrace()
{
  void * array[10];
  size_t size;
  size = backtrace(array, 10);
  char ** symbols = backtrace_symbols(array, size);
  for (size_t i = 0; i < size; i++) {
    RCLCPP_ERROR(rclcpp::get_logger("cabot_handle_v3_node"), "StackTrace[%zu]: %s", i, symbols[i]);
  }
  free(symbols);
}

void CaBotHandleV3Node::notificationCallback(const std_msgs::msg::Int8::UniquePtr & msg)
{
  if (msg) {
    std::string log_msg_ = "Received notification: " + std::to_string(msg->data);
    RCLCPP_INFO(this->get_logger(), log_msg_.c_str());
    this->handle_->executeStimulus(msg->data);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Received nullptr message in notificationCallback");
  }
  rclcpp::Clock::SharedPtr clock = this->get_clock();
  RCLCPP_INFO(
    this->get_logger(), "Node clock type (in notificationCallback): %d", clock->get_clock_type());
}

void CaBotHandleV3Node::eventListener(const std::map<std::string, std::string> & msg)
{
  std::shared_ptr<BaseEvent> event = nullptr;
  std::string msg_str;
  for (std::map<std::string, std::string>::const_iterator it = msg.begin(); it != msg.end(); ++it) {
    msg_str += "'" + it->first + "': " + it->second;
    if (std::next(it) != msg.end()) {
      msg_str += ", ";
    }
  }
  msg_str = "{" + msg_str + "}";
  RCLCPP_INFO(get_logger(), msg_str.c_str());
  if (msg_str.find("buttons") != std::string::npos) {
    const int & buttons = std::stoi(msg.at("buttons"));
    const int & count = std::stoi(msg.at("count"));
    event = std::make_shared<ClickEvent>(buttons, count);
  } else if (msg_str.find("button") != std::string::npos) {
    const int & button = std::stoi(msg.at("button"));
    const bool & up = (msg.at("up") == "True") ? true : false;
    const bool & hold = (msg.find("hold") != msg.end()) ? true : false;
    std::shared_ptr<ButtonEvent> buttonEvent = std::make_shared<ButtonEvent>(button, up, hold);
    event = buttonEvent;
    // button down confirmation
    if (buttonEvent && buttonEvent->is_up()) {
      this->handle_->executeStimulus(8);
    }
  } else if (msg_str.find("holddown") != std::string::npos) {
    const int & hold = std::stoi(msg.at("holddown"));
    const int & duration = std::stoi(msg.at("duration"));
    std::shared_ptr<HoldDownEvent> holdDownEvent = std::make_shared<HoldDownEvent>(hold, duration);
    event = holdDownEvent;
    // button hold down confirmation
    if (holdDownEvent) {
      this->handle_->executeStimulus(9);
    }
  }
  if (event) {
    RCLCPP_INFO(get_logger(), event->toString().c_str());
    std::unique_ptr<std_msgs::msg::String> msg = std::make_unique<std_msgs::msg::String>();
    msg->data = event->toString();
    event_pub_->publish(std::move(msg));
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotHandleV3Node)
