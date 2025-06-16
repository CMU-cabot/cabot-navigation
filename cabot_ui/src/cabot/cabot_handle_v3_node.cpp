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
  handle_(nullptr), event_pub_(nullptr), vibrator_type_(1)
{
  event_pub_ = create_publisher<std_msgs::msg::String>("/cabot/event", rclcpp::QoS(10));
  declare_parameter("vibrator_type", vibrator_type_);
  vibrator_type_ = get_parameter("vibrator_type").as_int();
  RCLCPP_INFO(get_logger(), "vibrator_type: %d", vibrator_type_);
  bool no_vibration = declare_parameter("no_vibration", false);
  RCLCPP_INFO(get_logger(), "no_vibration = %s", no_vibration ? "true" : "false");
  if (!no_vibration) {
    notification_sub = create_subscription<std_msgs::msg::Int8>(
      "/cabot/notification", 10, std::bind(&CaBotHandleV3Node::notificationCallback, this, std::placeholders::_1));
  }

  timer = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      initializeHandle();
      timer->cancel();
    });
}

void CaBotHandleV3Node::initializeHandle()
{
  if (handle_) {
    RCLCPP_WARN(get_logger(), "Handle is already initialized");
    return;
  }
  handle_ = std::make_unique<Handle>(
    std::static_pointer_cast<CaBotHandleV3Node>(shared_from_this()),
    std::bind(&CaBotHandleV3Node::eventListener, this, std::placeholders::_1),
    vibrator_type_
  );
}

CaBotHandleV3Node::~CaBotHandleV3Node()
{
  RCLCPP_INFO(get_logger(), "Destroying CaBotHandleV2Node");
  if (handle_) {
    RCLCPP_INFO(get_logger(), "Resetting handle_");
    handle_.reset();  // Explicitly reset the unique_ptr to release resources
  } else {
    RCLCPP_WARN(get_logger(), "handle_ is already null");
  }
}

void CaBotHandleV3Node::notificationCallback(const std_msgs::msg::Int8::SharedPtr msg)
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

void CaBotHandleV3Node::eventListener(const std::map<std::string, int> & msg)
{
  std::shared_ptr<BaseEvent> event = nullptr;
  std::string msg_str;
  for (auto it = msg.begin(); it != msg.end(); ++it) {
    msg_str += "'" + it->first + "': " + std::to_string(it->second);
    if (std::next(it) != msg.end()) {
      msg_str += ", ";
    }
  }
  msg_str = "{" + msg_str + "}";
  RCLCPP_INFO(get_logger(), msg_str.c_str());
  if (msg_str.find("buttons") != std::string::npos) {
    const int & buttons = msg.at("buttons");
    const int & count = msg.at("count");
    event = std::make_shared<ClickEvent>(buttons, count);
  } else if (msg_str.find("button") != std::string::npos) {
    const int & button = msg.at("button");
    const bool & up = (msg.at("up") == 1);
    const bool & hold = (msg.find("hold") != msg.end()) ? true : false;
    std::shared_ptr<ButtonEvent> buttonEvent = std::make_shared<ButtonEvent>(button, up, hold);
    event = buttonEvent;
    // button down confirmation
    if (buttonEvent && !buttonEvent->is_up()) {
      this->handle_->executeStimulus(8);
    }
  } else if (msg_str.find("holddown") != std::string::npos) {
    const int & hold = msg.at("holddown");
    const int & duration = msg.at("duration");
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
