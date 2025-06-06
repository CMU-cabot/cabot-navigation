// Copyright (c) 2025  Carnegie Mellon University
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

#ifndef CABOT_NAVIGATION2__TEMPORAL_OBSTACLE_LAYER_HPP_
#define CABOT_NAVIGATION2__TEMPORAL_OBSTACLE_LAYER_HPP_

#include <limits>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cabot_navigation2
{

class TemporalObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  TemporalObstacleLayer();
  virtual ~TemporalObstacleLayer();

  void onInitialize() override;
  void initMaps(unsigned int size_x, unsigned int size_y) override;
  void resetMaps() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void raytraceFreespace(
    const nav2_costmap_2d::Observation & clearing_observation,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
  unsigned char * temp_costmap_;
  unsigned char decaying_rate_;
  unsigned char rising_rate_;
};

}  // namespace cabot_navigation2

#endif  // CABOT_NAVIGATION2__TEMPORAL_OBSTACLE_LAYER_HPP_
