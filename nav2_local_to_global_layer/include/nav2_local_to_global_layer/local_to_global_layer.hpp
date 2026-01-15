#ifndef NAV2_LOCAL_TO_GLOBAL_LAYER__LOCAL_TO_GLOBAL_LAYER_HPP_
#define NAV2_LOCAL_TO_GLOBAL_LAYER__LOCAL_TO_GLOBAL_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_local_to_global_layer
{

class LocalToGlobalLayer : public nav2_costmap_2d::Layer
{
public:
  LocalToGlobalLayer();
  virtual ~LocalToGlobalLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  virtual void reset() { current_ = false; }
  virtual bool isClearable() { return false; }

private:
  void incomingCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  unsigned char interpretValue(int8_t value);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_local_costmap_;
  std::string topic_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_local_to_global_layer

#endif  // NAV2_LOCAL_TO_GLOBAL_LAYER__LOCAL_TO_GLOBAL_LAYER_HPP_