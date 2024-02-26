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

#include <gtest/gtest.h>
#include <stdlib.h>

#include <cabot_navigation2/cabot_planner_util.hpp>

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

}  // namespace cabot_navigation2_test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
