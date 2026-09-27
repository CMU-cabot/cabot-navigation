// Copyright (c) 2026 Carnegie Mellon University
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include "cabot_navigation2/cabot_hybrid_rl_controller.hpp"

namespace cabot_navigation2
{
class HybridControllerTestPeer
{
public:
  static void rlGoal(CaBotHybridRLController & controller, double x, double y)
  {
    auto goal = std::make_shared<geometry_msgs::msg::Point>();
    goal->x = x;
    goal->y = y;
    controller.rlSubgoalCallback(goal);
  }
  static void people(CaBotHybridRLController & controller,
    const std::vector<std::pair<double, double>> & positions)
  {
    auto message = std::make_shared<lidar_process_msgs::msg::PositionHistoryArray>();
    lidar_process_msgs::msg::PositionArray people;
    for (size_t i = 0; i < positions.size(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = positions[i].first;
      p.y = positions[i].second;
      people.positions.push_back(p);
      std_msgs::msg::Int32 id;
      id.data = static_cast<int32_t>(i);
      people.ids.push_back(id);
    }
    people.quantity = positions.size();
    message->positions_history.assign(10, people);
    controller.rlPeopleCallback(message);
  }
};
}  // namespace cabot_navigation2

class HybridControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("hybrid_test");
    veto = node->add_on_set_parameters_callback([this](const auto &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = !reject_transaction;
        result.reason = "test transaction veto";
        return result;
      });
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("hybrid_test_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    map = costmap->getCostmap();
    map->resizeMap(600, 600, 0.01, -3.0, -3.0);
    map->resetMap(0, 0, 600, 600);
    for (unsigned int y = 0; y < 600; ++y) {
      for (unsigned int x = 0; x < 600; ++x) {map->setCost(x, y, 0);}
    }
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    controller.configure(node, "HybridRLFollowPath", tf, costmap);
    controller.activate();
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.0;
    setGoal(2.0, 0.0);
  }

  void TearDown() override
  {
    controller.deactivate();
    controller.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  }

  void setGoal(double x, double y)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    auto goal = pose;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    path.poses = {goal};
    controller.setPlan(path);
    cabot_navigation2::HybridControllerTestPeer::rlGoal(controller, x, y);
  }

  geometry_msgs::msg::Twist command()
  {
    return controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr).twist;
  }

  rcl_interfaces::msg::SetParametersResult set(std::initializer_list<std::pair<std::string, double>> values)
  {
    std::vector<rclcpp::Parameter> parameters;
    for (const auto & value : values) {
      parameters.emplace_back("HybridRLFollowPath." + value.first, value.second);
    }
    return node->set_parameters_atomically(parameters);
  }

  void blockTranslation()
  {
    for (unsigned int y = 0; y < 600; ++y) {
      for (unsigned int x = 0; x < 600; ++x) {map->setCost(x, y, 254);}
    }
    unsigned int x, y;
    ASSERT_TRUE(map->worldToMap(0, 0, x, y));
    map->setCost(x, y, 0);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  nav2_costmap_2d::Costmap2D * map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  cabot_navigation2::CaBotHybridRLController controller;
  geometry_msgs::msg::PoseStamped pose;
  bool reject_transaction = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr veto;
};

TEST_F(HybridControllerTest, BlockedForwardGoalWaitsInsteadOfTurningAway)
{
  blockTranslation();
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}

TEST_F(HybridControllerTest, TiedCostsAndOddSampleCountStillChooseStop)
{
  ASSERT_TRUE(set({{"heading_cost_wt", 0.0}, {"angular_cost_wt", 0.0}, {"angular_sample_size", 5.0}}).successful);
  blockTranslation();
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}

TEST_F(HybridControllerTest, WallInFrontDoesNotCauseTurnAround)
{
  for (unsigned int y = 0; y < 600; ++y) {
    for (unsigned int x = 305; x < 600; ++x) {map->setCost(x, y, 254);}
  }
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}

TEST_F(HybridControllerTest, NecessaryLeftAndRightTurnsRemainAvailable)
{
  blockTranslation();
  setGoal(0.0, 2.0);
  EXPECT_GT(command().angular.z, 0.0);
  setGoal(0.0, -2.0);
  EXPECT_LT(command().angular.z, 0.0);
  setGoal(-2.0, 0.0);
  EXPECT_GT(std::abs(command().angular.z), 0.0);
}

TEST_F(HybridControllerTest, NearGoalDoesNotBypassObstacleEvaluation)
{
  setGoal(0.25, 0.0);
  blockTranslation();
  EXPECT_DOUBLE_EQ(command().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command().angular.z, 0.0);
}

TEST_F(HybridControllerTest, HeadingAcrossPiUsesShortestTurn)
{
  const double heading = 170.0 * M_PI / 180.0;
  pose.pose.orientation.z = std::sin(heading / 2.0);
  pose.pose.orientation.w = std::cos(heading / 2.0);
  const double target = -170.0 * M_PI / 180.0;
  setGoal(0.25 * std::cos(target), 0.25 * std::sin(target));
  blockTranslation();
  EXPECT_GT(command().angular.z, 0.0);
}

TEST_F(HybridControllerTest, VelocityLimitChangesOnTheNextCycleWithoutReconfigure)
{
  ASSERT_TRUE(set({{"max_linear_velocity", 0.2}}).successful);
  EXPECT_NEAR(command().linear.x, 0.2, 1e-9);
  ASSERT_TRUE(set({{"max_linear_velocity", 0.6}}).successful);
  EXPECT_NEAR(command().linear.x, 0.6, 1e-9);
  ASSERT_TRUE(set({{"max_linear_velocity", 0.0}, {"max_angular_velocity", 0.0}}).successful);
  EXPECT_DOUBLE_EQ(command().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command().angular.z, 0.0);
}

TEST_F(HybridControllerTest, InvalidAtomicUpdateLeavesThePreviousControlIntact)
{
  ASSERT_TRUE(set({{"max_linear_velocity", 0.2}}).successful);
  EXPECT_FALSE(set({{"max_linear_velocity", 0.9}, {"sampling_rate", 0.0}}).successful);
  EXPECT_NEAR(command().linear.x, 0.2, 1e-9);
  EXPECT_FALSE(set({{"max_angular_velocity", -1.0}}).successful);
  EXPECT_FALSE(set({{"max_linear_velocity", std::numeric_limits<double>::quiet_NaN()}}).successful);
  EXPECT_FALSE(set({{"prediction_horizon", std::numeric_limits<double>::infinity()}}).successful);
  EXPECT_FALSE(set({{"angular_sample_size", 3.5}}).successful);
  EXPECT_FALSE(set({{"linear_sample_size", 200.0}, {"angular_sample_size", 200.0}}).successful);
  EXPECT_FALSE(set({{"lookahead_distance", 20.0}}).successful);
  EXPECT_NEAR(command().linear.x, 0.2, 1e-9);
}

TEST_F(HybridControllerTest, AnotherCallbacksVetoDoesNotChangeCachedControl)
{
  ASSERT_TRUE(set({{"max_linear_velocity", 0.2}}).successful);
  EXPECT_NEAR(command().linear.x, 0.2, 1e-9);
  reject_transaction = true;
  EXPECT_FALSE(set({{"max_linear_velocity", 0.7}}).successful);
  EXPECT_NEAR(command().linear.x, 0.2, 1e-9);
}

TEST_F(HybridControllerTest, RelatedParametersCanChangeAtomically)
{
  ASSERT_TRUE(set({{"prediction_horizon", 0.05}, {"sampling_rate", 0.01}}).successful);
  EXPECT_GT(command().linear.x, 0.0);
  auto result = node->set_parameters_atomically({rclcpp::Parameter("HybridRLFollowPath.rl_subgoal_topic", "/elsewhere")});
  EXPECT_FALSE(result.successful);
}

TEST_F(HybridControllerTest, ParametersCanChangeWhileTheControlLoopIsRunning)
{
  ASSERT_TRUE(set({{"max_linear_velocity", 0.2}}).successful);
  std::atomic<bool> finished{false};
  std::atomic<bool> valid{true};
  std::atomic<int> cycles{0};
  std::thread compute([&]() {
      while (!finished) {
        const auto velocity = command();
        if (std::abs(velocity.linear.x - 0.2) > 1e-9 && std::abs(velocity.linear.x - 0.6) > 1e-9) {
          valid = false;
        }
        ++cycles;
      }
    });
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(set({{"max_linear_velocity", i % 2 ? 0.2 : 0.6}}).successful);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  finished = true;
  compute.join();
  EXPECT_TRUE(valid);
  EXPECT_GT(cycles.load(), 0);
}

TEST_F(HybridControllerTest, NewRouteWaitsForFreshRlSubgoal)
{
  EXPECT_GT(command().linear.x, 0.0);
  nav_msgs::msg::Path path;
  path.poses = {pose};
  path.poses[0].pose.position.y = 2.0;
  controller.setPlan(path);
  EXPECT_DOUBLE_EQ(command().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command().angular.z, 0.0);
  cabot_navigation2::HybridControllerTestPeer::rlGoal(controller, 0, 2);
  EXPECT_GT(command().angular.z, 0.0);
}

TEST_F(HybridControllerTest, PersonInFrontChoosesTheClearLeftExit)
{
  cabot_navigation2::HybridControllerTestPeer::people(controller, {{1.1, 0.0}, {0.8, -0.8}});
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_GT(result.angular.z, 0.0);
  EXPECT_LE(result.angular.z, 0.25);
}

TEST_F(HybridControllerTest, PersonInFrontChoosesTheClearRightExit)
{
  cabot_navigation2::HybridControllerTestPeer::people(controller, {{1.1, 0.0}, {0.8, 0.8}});
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_LT(result.angular.z, 0.0);
  EXPECT_GE(result.angular.z, -0.25);
}

TEST_F(HybridControllerTest, NoClearExitKeepsTheRobotStopped)
{
  cabot_navigation2::HybridControllerTestPeer::people(controller, {{0.8, 0.0}});
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}

TEST_F(HybridControllerTest, AvoidanceDoesNotAccumulateIntoATurnAround)
{
  cabot_navigation2::HybridControllerTestPeer::people(controller, {{1.1, 0.0}, {0.8, -0.8}});
  double yaw = 0.0;
  for (int i = 0; i < 60; ++i) {
    const auto result = command();
    EXPECT_GE(result.linear.x, 0.0);
    if (result.linear.x > 0.0) {break;}
    EXPECT_LE(std::abs(result.angular.z), 0.25);
    yaw += result.angular.z * 0.2;
    EXPECT_LE(std::abs(yaw), 0.7 + 1e-6);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
  }
  EXPECT_GT(yaw, 0.0);
}

TEST_F(HybridControllerTest, AvoidanceCanBeDisabledAndRateLimitedLive)
{
  cabot_navigation2::HybridControllerTestPeer::people(controller, {{1.1, 0.0}, {0.8, -0.8}});
  ASSERT_TRUE(set({{"avoidance_angular_velocity", 0.1}}).successful);
  EXPECT_NEAR(command().angular.z, 0.1, 1e-9);
  ASSERT_TRUE(set({{"avoidance_max_angle", 0.0}}).successful);
  EXPECT_DOUBLE_EQ(command().angular.z, 0.0);
  EXPECT_FALSE(set({{"avoidance_max_angle", 2.0}}).successful);
  EXPECT_FALSE(set({{"avoidance_min_clearance", 0.0}}).successful);
}

TEST_F(HybridControllerTest, AvoidanceChecksTheSweptFootprintBeforeTurning)
{
  std::vector<geometry_msgs::msg::Point> footprint;
  for (const auto & xy : std::vector<std::pair<double, double>>{
    {0.3, 0.2}, {0.3, -0.2}, {-0.3, -0.2}, {-0.3, 0.2}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    footprint.push_back(p);
  }
  costmap->setRobotFootprint(footprint);
  unsigned int x, y;
  ASSERT_TRUE(map->worldToMap(0.25, 0.25, x, y));
  map->setCost(x, y, 254);
  cabot_navigation2::HybridControllerTestPeer::people(controller, {{1.1, 0.0}});
  const auto result = command();
  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_LT(result.angular.z, 0.0);
  EXPECT_GE(result.angular.z, -0.25);
}
