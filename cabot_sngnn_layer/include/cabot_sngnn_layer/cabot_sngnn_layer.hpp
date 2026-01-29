#ifndef CABOT_SNGNN_LAYER__CABOT_SNGNN_LAYER_HPP_
#define CABOT_SNGNN_LAYER__CABOT_SNGNN_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"

namespace cabot_sngnn_layer
{

class SNGNNLayer : public nav2_costmap_2d::Layer
{
public:
  SNGNNLayer();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  virtual void reset() {
    current_ = false;
  }
  virtual bool isClearable() {
    return false;
  }

protected:
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
  std::mutex mutex_;
  double resolution_;
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
};

}  // namespace cabot_sngnn_layer

#endif  // CABOT_SNGNN_LAYER__CABOT_SNGNN_LAYER_HPP_
