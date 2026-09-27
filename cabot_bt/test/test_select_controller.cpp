// Copyright (c) 2026 Carnegie Mellon University
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include "../plugins/action/select_controller.cpp"

TEST(SelectController, MapsModesAtomicallyAndRejectsUnknown)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("selection_test");
  auto service = node->create_service<std_srvs::srv::Trigger>(
    "/cabot/get_controller", [](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr) {});
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("bt_loop_duration", std::chrono::milliseconds(10));
  blackboard->set("server_timeout", std::chrono::milliseconds(1000));
  blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(1000));
  BT::NodeConfiguration config;
  config.blackboard = blackboard;
  config.input_ports["service_name"] = "/cabot/get_controller";
  config.output_ports["controller"] = "{controller}";
  config.output_ports["planner"] = "{planner}";
  cabot_bt::SelectController selector("SelectController", config);
  const std::map<std::string, std::string> modes = {
    {"follow", "FollowPath"}, {"crowdattn", "RLFollowPath"}, {"mpc", "MPCFollowPath"},
    {"rl", "RLFollowPath"}, {"hybrid", "HybridRLFollowPath"}, {"sm", "SocialMomentumFollowPath"}};
  for (const auto & mode : modes) {
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    response->success = true;
    response->message = mode.first;
    ASSERT_EQ(selector.on_completion(response), BT::NodeStatus::SUCCESS);
    ASSERT_EQ(blackboard->get<std::string>("controller"), mode.second);
    ASSERT_EQ(blackboard->get<std::string>("planner"), mode.first == "follow" ? "CaBot" : "PathForward");
  }
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  response->success = true;
  response->message = "unknown";
  ASSERT_EQ(selector.on_completion(response), BT::NodeStatus::FAILURE);
  response->message = "follow";
  response->success = false;
  ASSERT_EQ(selector.on_completion(response), BT::NodeStatus::FAILURE);
  rclcpp::shutdown();
}
