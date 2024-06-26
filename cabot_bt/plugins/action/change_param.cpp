// Copyright (c) 2020  Carnegie Mellon University
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

#include <behaviortree_cpp_v3/action_node.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcl_yaml_param_parser/types.h>
#include <rcutils/allocator.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

enum Status
{
  Initial,
  Saving,
  Setting,
  Waiting,
  Done
};

class ChangeParamAction : public BT::StatefulActionNode
{
public:
  ChangeParamAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::StatefulActionNode(action_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    state_ = Status::Initial;

    if (getInput("node_name", remote_node_name_)) {
      parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, remote_node_name_);
    }
    if (getInput("store", store_name_)) {
      config().blackboard->get<std::shared_ptr<std::map<std::string, std::vector<rclcpp::Parameter>>>>(store_name_, store_);
    }

    int count = 30;
    RCLCPP_INFO(node_->get_logger(), "Waiting %s parameter server", remote_node_name_.c_str());
    while (!parameters_client_->wait_for_service(1s)) {
      if (count < 0) {
        parameters_client_ = nullptr;
        break;
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }
  }

  ChangeParamAction() = delete;

  ~ChangeParamAction()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down ChangeParamAction BT node");
  }

  BT::NodeStatus onStart() override
  {
    if (!parameters_client_) {
      RCLCPP_INFO(node_->get_logger(), "Check node_name %s", remote_node_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    std::string param_name;
    if (!getInput("param_name", param_name)) {
      RCLCPP_INFO(node_->get_logger(), "param_name is missing");
      return BT::NodeStatus::FAILURE;
    }
    state_ = Status::Saving;
    RCLCPP_INFO(node_->get_logger(), "get parameters %s", param_name.c_str());
    parameters_ = parameters_client_->get_parameters({param_name});
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);
    if (state_ == Status::Saving) {
      if (parameters_.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
        RCLCPP_INFO(node_->get_logger(), "parameters size = %ld", parameters_.get().size());
        if (parameters_.get().size() > 0) {
          auto param = parameters_.get().at(0);

          if (store_->find(remote_node_name_) == store_->end()) {
            store_->insert(std::pair<std::string, std::vector<rclcpp::Parameter>>(remote_node_name_, std::vector<rclcpp::Parameter>()));
          }
          store_->at(remote_node_name_).push_back(param);
        }
        state_ = Status::Setting;
      }
      return BT::NodeStatus::RUNNING;
    }

    if (state_ == Status::Setting) {
      RCLCPP_INFO(node_->get_logger(), "Set parameters");
      std::string param_name;
      if (!getInput("param_name", param_name)) {
        RCLCPP_INFO(node_->get_logger(), "param_name is missing");
        return BT::NodeStatus::FAILURE;
      }

      std::string param_value;
      if (!getInput("param_value", param_value)) {
        RCLCPP_INFO(node_->get_logger(), "param_value is missing");
        return BT::NodeStatus::FAILURE;
      }
      rcutils_allocator_t allocator = rcutils_get_default_allocator();
      rcl_params_t * params_hdl = rcl_yaml_node_struct_init(allocator);
      bool res = rcl_parse_yaml_value(
        remote_node_name_.c_str(),
        param_name.c_str(),
        param_value.c_str(), params_hdl);
      if (!res) {
        RCLCPP_INFO(node_->get_logger(), "Coule not parse param_value %s", param_value.c_str());
        return BT::NodeStatus::FAILURE;
      }
      auto map = rclcpp::parameter_map_from(params_hdl);
      for (auto p : map.at(remote_node_name_)) {
        RCLCPP_INFO(node_->get_logger(), "Param %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
      }
      future_ = parameters_client_->set_parameters(map.at(remote_node_name_));
      state_ = Status::Waiting;
      return BT::NodeStatus::RUNNING;
    }

    if (state_ == Status::Waiting) {
      if (future_.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
        RCLCPP_INFO(node_->get_logger(), "Success to set parameters");
        state_ = Status::Done;
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
  }

  void logStuck(const std::string & msg) const
  {
    static std::string prev_msg;

    if (msg == prev_msg) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    prev_msg = msg;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<std::string>("node_name", "node name"),
      BT::InputPort<std::string>("param_name", "param name"),
      BT::InputPort<std::string>("param_value", "param value"),
      BT::InputPort<std::string>("store", "store name")
    };
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::ParameterMap * map_;
  std::string remote_node_name_;
  Status state_;
  BT::NodeStatus result_;
  std::shared_ptr<std::map<std::string, std::vector<rclcpp::Parameter>>> store_;
  std::string store_name_;
  std::shared_future<std::vector<rclcpp::Parameter>> parameters_;
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::ChangeParamAction>("ChangeParam");
}
