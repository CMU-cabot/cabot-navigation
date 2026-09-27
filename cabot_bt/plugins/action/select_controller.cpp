// Copyright (c) 2026 Carnegie Mellon University
// SPDX-License-Identifier: MIT

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cabot_bt
{
class SelectController : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::Trigger>
{
public:
  SelectController(const std::string & name, const BT::NodeConfiguration & config)
  : BtServiceNode(name, config, "/cabot/get_controller") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<std::string>("controller"),
      BT::OutputPort<std::string>("planner")});
  }

  BT::NodeStatus on_completion(std::shared_ptr<std_srvs::srv::Trigger::Response> response) override
  {
    static const std::map<std::string, std::pair<std::string, std::string>> modes = {
      {"follow", {"FollowPath", "CaBot"}},
      {"blind", {"BlindFollowPath", "PathForward"}},
      {"mpc", {"MPCFollowPath", "PathForward"}},
      {"rl", {"RLFollowPath", "PathForward"}},
      {"crowdattn", {"RLFollowPath", "PathForward"}},
      {"hybrid", {"HybridRLFollowPath", "PathForward"}},
      {"sm", {"SocialMomentumFollowPath", "PathForward"}}};
    const auto mode = modes.find(response->message);
    if (!response->success || mode == modes.end()) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("controller", mode->second.first);
    setOutput("planner", mode->second.second);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace cabot_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::SelectController>("SelectController");
}
