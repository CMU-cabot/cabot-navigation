// Copyright (c) 2025  Carnegie Mellon University, IBM Corporation, and others
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

#ifndef CABOT_NAVIGATION2__LIMIT_FOV_OBSTACLE_LAYER_HPP_
#define CABOT_NAVIGATION2__LIMIT_FOV_OBSTACLE_LAYER_HPP_

#include <algorithm>

#include <nav2_costmap_2d/obstacle_layer.hpp>

namespace cabot_navigation2
{

class LimitFovObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  LimitFovObstacleLayer();
  ~LimitFovObstacleLayer() override;

protected:
  void raytraceFreespace(
    const nav2_costmap_2d::Observation & clearing_observation,
    double * min_x, double * min_y,
    double * max_x,
    double * max_y) override;

private:
  template<class ActionType>
  inline void raytraceSuperCoverLine(
    ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
    unsigned int max_length = UINT_MAX, unsigned int min_length = 0)
  {
    int dx_full = x1 - x0;
    int dy_full = y1 - y0;
    double dist = std::hypot(dx_full, dy_full);
    if (dist < min_length) {
      return;
    }

    unsigned int min_x0, min_y0;
    if (dist > 0.0) {
      min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
      min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
    } else {
      min_x0 = x0;
      min_y0 = y0;
    }

    int dx = x1 - min_x0;
    int dy = y1 - min_y0;
    unsigned int abs_dx = std::abs(dx);
    unsigned int abs_dy = std::abs(dy);
    int sign_dx = (dx > 0) ? 1 : (dx < 0 ? -1 : 0);
    int sign_dy = (dy > 0) ? 1 : (dy < 0 ? -1 : 0);

    int x = min_x0;
    int y = min_y0;
    at(y * size_x_ + x);

    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
    unsigned int offset_x = std::min(abs_dx, (unsigned int)(scale * abs_dx));
    unsigned int offset_y = std::min(abs_dy, (unsigned int)(scale * abs_dy));
    for (int ix = 0, iy = 0; ix < offset_x || iy < offset_y; ) {
      int direction = (1 + 2 * ix) * offset_y - (1 + 2 * iy) * offset_x;
      if (direction == 0) {
        // move diagonal
        x += sign_dx;
        y += sign_dy;
        ix++;
        iy++;

        // when moving diagonal direction, fill both horizontal and vertical cells too
        at(y * size_x_ + x - sign_dx);
        at((y - sign_dy) * size_x_ + x);
      } else if (direction < 0) {
        // move horizontal
        x += sign_dx;
        ix++;
      } else {
        // move vertical
        y += sign_dy;
        iy++;
      }
      at(y * size_x_ + x);
    }
  }
};
}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__LIMIT_FOV_OBSTACLE_LAYER_HPP_
