// Copyright (c) 2024  Carnegie Mellon University
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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cabot_bt/plugins/condition/check_path.hpp>

// Test Fixture
class CheckPathConditionTest : public ::testing::Test
{
protected:
  CheckPathConditionTest()
  : node_(std::make_shared<rclcpp::Node>("test_node"))
  {
    conf_.blackboard = BT::Blackboard::create();
    conf_.blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    BT::assignDefaultRemapping<cabot_bt::CheckPathCondition>(conf_);
    check_path_condition_ =
      std::make_shared<cabot_bt::CheckPathCondition>("CheckPathCondition", conf_);
  }

  // Helper function to build a Path message from a YAML file
  nav_msgs::msg::Path buildPathFromYaml(const std::string & yaml_path)
  {
    std::string package_name = "cabot_bt";
    std::string package_share_directory;
    try {
      package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
      RCLCPP_INFO(this->node_->get_logger(), "Package share directory: %s", package_share_directory.c_str());
    } catch (const std::runtime_error & e) {
    }
    nav_msgs::msg::Path path;

    // Load YAML file
    YAML::Node yaml_node = YAML::LoadFile(package_share_directory + "/test/data/" + yaml_path);

    // Populate the Path message from YAML data
    for (const auto & pose : yaml_node["poses"]) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = pose["header"]["frame_id"].as<std::string>();
      pose_stamped.pose.position.x = pose["pose"]["position"]["x"].as<double>();
      pose_stamped.pose.position.y = pose["pose"]["position"]["y"].as<double>();
      pose_stamped.pose.position.z = pose["pose"]["position"]["z"].as<double>();
      pose_stamped.pose.orientation.x = pose["pose"]["orientation"]["x"].as<double>();
      pose_stamped.pose.orientation.y = pose["pose"]["orientation"]["y"].as<double>();
      pose_stamped.pose.orientation.z = pose["pose"]["orientation"]["z"].as<double>();
      pose_stamped.pose.orientation.w = pose["pose"]["orientation"]["w"].as<double>();
      path.poses.push_back(pose_stamped);
    }

    return path;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<cabot_bt::CheckPathCondition> check_path_condition_;
  BT::NodeConfiguration conf_;
};

TEST_F(CheckPathConditionTest, TestSimilarEnough) {
  // Build two dummy path messages from YAML
  auto path = buildPathFromYaml("path.yaml");
  auto target_path = buildPathFromYaml("plan.yaml");

  conf_.blackboard->set<nav_msgs::msg::Path>("path", target_path);
  conf_.blackboard->set<double>("average_threshold", 0.3);
  check_path_condition_->path_callback(std::make_shared<nav_msgs::msg::Path>(path));
  auto result = check_path_condition_->check_path();

  // Check expected result
  EXPECT_TRUE(result);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
