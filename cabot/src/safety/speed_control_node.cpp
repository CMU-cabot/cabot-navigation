// Copyright (c) 2020, 2022  Carnegie Mellon University
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
//
// Clutch Adapter Node
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace CaBotSafety
{
class SpeedControlNode : public rclcpp::Node
{
public:
  explicit SpeedControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("speed_control_node", options),
    cmdVelInput_("/cmd_vel"),
    cmdVelOutput_("/cmd_vel_limit"),
    userSpeedInput_("/user_speed"),
    mapSpeedInput_("/map_speed"),
    wheelieInput_("wheelie_state"),
    userSpeedLimit_(2.0),
    mapSpeedLimit_(2.0),
    targetRate_(40),
    currentLinear_(0.0),
    currentAngular_(0.0),
    lastCmdVelInput_(0, 0, get_clock()->get_clock_type())
  {
    RCLCPP_INFO(get_logger(), "SpeedControlNodeClass Constructor");
    onInit();
  }

  ~SpeedControlNode()
  {
    RCLCPP_INFO(get_logger(), "SpeedControlNodeClass Destructor");
    for (auto sub : speedSubs_) {
      sub.reset();
    }
    for (auto srv : configureServices_) {
      srv.reset();
    }
    timer_.reset();
    callback_handler_.reset();
    cmdVelPub.reset();
    cmdVelSub.reset();
    userSpeedSub.reset();
    mapSpeedSub.reset();
    wheelieStateSub.reset();
  }

private:
  void onInit()
  {
    RCLCPP_INFO(get_logger(), "Speed Control Adapter Node - %s", __FUNCTION__);

    cmdVelInput_ = declare_parameter("cmd_vel_input", cmdVelInput_);
    cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
      cmdVelInput_, 10,
      std::bind(&SpeedControlNode::cmdVelCallback, this, std::placeholders::_1));
    wheelieStateSub = create_subscription<std_msgs::msg::Bool>(
      wheelieInput_, 10,
      std::bind(&SpeedControlNode::wheelieStateCallback, this, std::placeholders::_1));
    cmdVelOutput_ = declare_parameter("cmd_vel_output", cmdVelOutput_);
    cmdVelPub = create_publisher<geometry_msgs::msg::Twist>(cmdVelOutput_, 10);
    cmdVelPub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_wheelie", 10);

    speedInput_ = declare_parameter("speed_input", speedInput_);
    speedLimit_ = declare_parameter("speed_limit", speedLimit_);
    speedTimeOut_ = declare_parameter("speed_timeout", speedTimeOut_);
    completeStop_ = declare_parameter("complete_stop", completeStop_);
    configurable_ = declare_parameter("configurable", configurable_);
    targetRate_ = declare_parameter("target_rate", targetRate_);

    for (uint64_t index = 0; index < speedInput_.size(); index++) {
      auto topic = speedInput_[index];
      std::function<void(const std_msgs::msg::Float32::SharedPtr)> callback =
        [&, index](const std_msgs::msg::Float32::SharedPtr input)
        {
          // RCLCPP_INFO(get_logger(), "receive index=%d, limit=%.2f", index, input->data);
          speedLimit_[index] = input->data;
          callbackTime_[index] = get_clock()->now();
        };
      auto qos = rclcpp::SystemDefaultsQoS().transient_local();
      auto sub = create_subscription<std_msgs::msg::Float32>(topic, qos, callback);
      speedSubs_.push_back(sub);
      if (speedLimit_.size() <= index) {
        speedLimit_.push_back(0);
      }
      if (speedTimeOut_.size() <= index) {
        speedTimeOut_.push_back(0);
      }
      if (completeStop_.size() <= index) {
        completeStop_.push_back(false);
      }

      enabled_.push_back(true);  // enabled at initial moment
      if (index < configurable_.size() && configurable_[index]) {
        auto logger = get_logger();
        std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr)> srvsCallback =
          [&, index, logger](const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
          {
            RCLCPP_INFO(logger, "receive enabled index=%ld, value=%d", index, request->data);
            enabled_[index] = request->data;
            response->success = true;
            response->message = "success";
          };
        auto server = create_service<std_srvs::srv::SetBool>(topic + "_enabled", srvsCallback);
        configureServices_.push_back(server);
      }
      if (callbackTime_.size() <= index) {
        callbackTime_.push_back(get_clock()->now());
      }

      callback_handler_ =
        add_on_set_parameters_callback(std::bind(&SpeedControlNode::param_set_callback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Subscribe to %s (index=%ld)", topic.c_str(), index);
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / targetRate_),
      std::bind(&SpeedControlNode::timerCallback, this));
  }

  rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
    for (auto && param : params) {
      if (!has_parameter(param.get_name())) {
        continue;
      }
      RCLCPP_DEBUG(get_logger(), "change param %s", param.get_name().c_str());

      if (param.get_name() == "complete_stop") {
        std::vector<bool> doubleArray = param.as_bool_array();
        for (int i = 0; i < doubleArray.size() && i < completeStop_.size(); i++) {
          completeStop_[i] = doubleArray[i];
        }
      }
    }
    results->successful = true;
    return *results;
  }

  void timerCallback()
  {
    for (size_t i = 0; i < callbackTime_.size(); i++) {
      if (speedTimeOut_[i] <= 0.0) {
        continue;
      }
      if ((get_clock()->now() - callbackTime_[i]) > rclcpp::Duration(std::chrono::duration<double>(speedTimeOut_[i]))) {
        speedLimit_[i] = 0.0;
      }
    }

    // force stop
    if (get_clock()->now() - lastCmdVelInput_ > rclcpp::Duration(500ms)) {
      currentLinear_ = 0;
      currentAngular_ = 0;
    }

    double l = currentLinear_;
    double r = currentAngular_;

    if (speedLimit_.size() > 0) {
      for (uint64_t index = 0; index < speedLimit_.size(); index++) {
        if (!enabled_[index]) {continue;}
        double limit = speedLimit_[index];
        if (limit < l) {
          l = limit;
        }
        if (l < -limit) {
          l = -limit;
        }

        // if limit equals zero and complete stop is true then angular is also zero
        if (limit == 0 && completeStop_[index]) {
          r = 0;
        }
      }
    } else {
      // backward compatibility
      // limit the input speeed
      if (userSpeedLimit_ < l) {
        l = userSpeedLimit_;
      }
      if (l < -userSpeedLimit_) {
        l = -userSpeedLimit_;
      }

      if (mapSpeedLimit_ < l) {
        l = mapSpeedLimit_;
      }
      if (l < -mapSpeedLimit_) {
        l = -mapSpeedLimit_;
      }
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = l;

    if (currentLinear_ != 0 && l != 0) {
      // to fit curve, adjust angular speed
      cmd_vel.angular.z = r / currentLinear_ * l;
    } else {
      cmd_vel.angular.z = r;
    }

    cmdVelPub->publish(cmd_vel);
  }

  void userSpeedCallback(const std_msgs::msg::Float32::SharedPtr input)
  {
    userSpeedLimit_ = input->data;
  }

  void mapSpeedCallback(const std_msgs::msg::Float32::SharedPtr input)
  {
    mapSpeedLimit_ = input->data;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input)
  {
    currentLinear_ = input->linear.x;
    currentAngular_ = input->angular.z;

    lastCmdVelInput_ = get_clock()->now();
  }

    void wheelieStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_wheelie_ = msg->data;  // Update Wheelie status
    }

    void wheelieControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd_vel = *msg;

        if (is_wheelie_)  // If in wheelie state, set speed to zero
        {
            RCLCPP_WARN(this->get_logger(), "Wheelie detected! Stopping the robot.");
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        cmdVelPub->publish(cmd_vel);
    }

  std::string cmdVelInput_;
  std::string cmdVelOutput_;
  std::string userSpeedInput_;
  std::string mapSpeedInput_;
  std::string wheelieInput_;

  std::vector<std::string> speedInput_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> speedSubs_;
  std::vector<double> speedLimit_;
  std::vector<rclcpp::Time> callbackTime_;
  std::vector<double> speedTimeOut_;
  std::vector<bool> completeStop_;
  std::vector<bool> enabled_;
  std::vector<bool> configurable_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> configureServices_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;

  double userSpeedLimit_;
  double mapSpeedLimit_;
  int targetRate_;
  bool is_wheelie_;

  double currentLinear_;
  double currentAngular_;
  rclcpp::Time lastCmdVelInput_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr userSpeedSub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mapSpeedSub;
  // Willy State Subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wheelieStateSub;
};  // class SpeedControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::SpeedControlNode)
