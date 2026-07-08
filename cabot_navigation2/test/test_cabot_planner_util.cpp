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

#include <cmath>
#include <stdlib.h>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cabot_navigation2/cabot_planner_util.hpp>
#include <cabot_navigation2/navcog_path_util.hpp>

namespace cabot_navigation2_test
{
class CabotPlannerUtilTest : public testing::Test
{
public:
  CabotPlannerUtilTest()
  {
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

protected:
};

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

TEST_F(CabotPlannerUtilTest, ObstacleDistanceTest) {
  cabot_navigation2::Obstacle o1(0, 0, 254, 0, 0, false);
  cabot_navigation2::Obstacle o2(10, 10, 253, 0, 0, false);
  o2.lethal = &o1;
  cabot_navigation2::Point p1(10, 0);
  cabot_navigation2::Point p2(0, 10);
  cabot_navigation2::Point p3(-10, 0);
  cabot_navigation2::Point p4(0, -10);

  EXPECT_GT(o2.distance(p1), 0);
  EXPECT_GT(o2.distance(p2), 0);
  EXPECT_LT(o2.distance(p3), 0);
  EXPECT_LT(o2.distance(p4), 0);

  printf("####%.2f\n", o2.distance(p1));
  printf("####%.2f\n", o2.distance(p2));
  printf("####%.2f\n", o2.distance(p3));
  printf("####%.2f\n", o2.distance(p4));
}

TEST_F(CabotPlannerUtilTest, SmoothStartKeepsShortPathGoal) {
  nav_msgs::msg::Path navcog_path;
  navcog_path.header.frame_id = "map";

  auto path_yaw = std::atan2(-0.140, -2.715);
  navcog_path.poses.push_back(makePose(26.329, -26.238, path_yaw));
  navcog_path.poses.push_back(makePose(23.614, -26.378, path_yaw));

  auto path = cabot_navigation2::normalizedPath(navcog_path, 0.05);
  auto start = makePose(32.783, -29.231, path_yaw);

  auto adjusted = cabot_navigation2::adjustedPathByStart(path, start, true);

  EXPECT_LT(cabot_navigation2::distance(adjusted.poses.back(), path.poses.back()), 0.001);
}

}  // namespace cabot_navigation2_test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
