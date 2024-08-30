#ifndef GROUP_OBSTACLE_LAYER_HPP_
#define GROUP_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include <nav2_costmap_2d/costmap_layer.hpp>

#include "lidar_process_msgs/msg/group_time_array.hpp"
#include "lidar_process_msgs/msg/group_array.hpp"
#include "lidar_process_msgs/msg/group.hpp"

namespace cabot_navigation2
{

class GroupObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  GroupObstacleLayer();
  ~GroupObstacleLayer() override;

  void activate() override;
  void deactivate() override;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, 
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  void onFootprintChanged() override;

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double update_width, update_height;

  void groupCallBack(const lidar_process_msgs::msg::GroupTimeArray::SharedPtr group_series);

  // Indicates that the entire costmap should be recalculated next time.
  bool need_recalculation_;

  std::string group_topic_;
  rclcpp::Subscription<lidar_process_msgs::msg::GroupTimeArray>::SharedPtr group_sub_;
  lidar_process_msgs::msg::GroupTimeArray::SharedPtr last_group_;

  std::mutex mutex_;
};

}  // namespace cabot_navigation2

#endif  // GROUP_OBSTACLE_LAYER_HPP_