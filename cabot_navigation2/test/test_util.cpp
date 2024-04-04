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
#include <cabot_navigation2/util.hpp>

namespace cabot_navigation2_test
{
class UtilTestFixture : public testing::Test
{
public:
  UtilTestFixture()
  {
  }

  void SetUp() override
  {
    l1 = Safety::Line(
      Safety::Point(0, 0),
      Safety::Point(10, 10)
    );
    l2 = Safety::Line(
      Safety::Point(0, 10),
      Safety::Point(10, 0)
    );
    l3 = Safety::Line(
      Safety::Point(0, 10),
      Safety::Point(4, 6)
    );
    l4 = Safety::Line(
      Safety::Point(20, 20),
      Safety::Point(30, 30)
    );
    l5 = Safety::Line(
      Safety::Point(10, 0),
      Safety::Point(20, 10)
    );
  }

  void TearDown() override
  {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

protected:
  Safety::Line l1, l2, l3, l4, l5;
};

TEST_F(UtilTestFixture, IntersectTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(l1.intersect_segment(l2));
}

TEST_F(UtilTestFixture, NotIntersectTest) {
  EXPECT_FALSE(l1.intersect_segment(l3));
}

TEST_F(UtilTestFixture, OnLineSegmentTest) {
  EXPECT_FALSE(l1.intersect_segment(l1));
}

TEST_F(UtilTestFixture, OnLineTest) {
  EXPECT_FALSE(l1.intersect_segment(l4));
}

TEST_F(UtilTestFixture, ParalelTest) {
  EXPECT_FALSE(l1.intersect_segment(l5));
}

TEST_F(UtilTestFixture, ClosestPointTest) {
  auto l = Safety::Line(Safety::Point(132, 158), Safety::Point(136, 158));
  auto p = l.closestPoint(Safety::Point(134.01, 80));
  printf("####%.2f %.2f\n", p.x, p.y);
  EXPECT_LT(p.x, 140);
  EXPECT_GT(p.y, 150);
}
}  // namespace cabot_navigation2_test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
