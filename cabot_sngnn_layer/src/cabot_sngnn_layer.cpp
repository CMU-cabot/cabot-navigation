#include "cabot_sngnn_layer/cabot_sngnn_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace cabot_sngnn_layer
{

SNGNNLayer::SNGNNLayer() : last_map_(nullptr), robot_x_(0), robot_y_(0), robot_yaw_(0)
{
}

void SNGNNLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
      throw std::runtime_error{"Failed to lock node"};
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue("/sngnn_costmap"));
  // SNGNNNode defaults to 0.05. We should parameterize this to keep synchronization.
  declareParameter("map_resolution", rclcpp::ParameterValue(0.05)); 

  node->get_parameter(name_ + "." + "enabled", enabled_);
  
  std::string topic;
  node->get_parameter(name_ + "." + "topic", topic);
  
  // Cache resolution
  node->get_parameter(name_ + "." + "map_resolution", resolution_);

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&SNGNNLayer::incomingMap, this, std::placeholders::_1));
  
  current_ = true;
}

void SNGNNLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  std::lock_guard<std::mutex> guard(mutex_);
  last_map_ = new_map;
  current_ = true;
}

void SNGNNLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;
  
  std::lock_guard<std::mutex> guard(mutex_);
  if (!last_map_) {
    return;
  }

  double ox = last_map_->info.origin.position.x;
  double oy = last_map_->info.origin.position.y;
  double width = last_map_->info.width * last_map_->info.resolution;
  double height = last_map_->info.height * last_map_->info.resolution;
  
  double x0 = ox;
  double y0 = oy;
  double x1 = ox + width;
  double y1 = oy + height;

  double px[4] = {x0, x1, x1, x0};
  double py[4] = {y0, y0, y1, y1};

  double c = cos(robot_yaw);
  double s = sin(robot_yaw);

  for (int k = 0; k < 4; k++) {
    double wx = robot_x + px[k] * c - py[k] * s;
    double wy = robot_y + px[k] * s + py[k] * c;
    *min_x = std::min(*min_x, wx);
    *min_y = std::min(*min_y, wy);
    *max_x = std::max(*max_x, wx);
    *max_y = std::max(*max_y, wy);
  }
}

void SNGNNLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> guard(mutex_);
  if (!last_map_) return;

  unsigned int width = last_map_->info.width;
  unsigned int height = last_map_->info.height;
  double res = last_map_->info.resolution;
  double ox = last_map_->info.origin.position.x;
  double oy = last_map_->info.origin.position.y;
  
  const auto& data = last_map_->data;
  double c = cos(robot_yaw_);
  double s = sin(robot_yaw_);

  for (unsigned int y = 0; y < height; ++y) {
    // ローカル座標の計算 (yループの外に出せる部分は出すとなお良いですが、このままでもOK)
    double ly = oy + (y + 0.5) * res;

    for (unsigned int x = 0; x < width; ++x) {
      // 【重要】ここの (height - 1 - y) が画像の上下反転を吸収しており、正しい実装です。
      int8_t val = data[(height - 1 - y) * width + x]; 
      
      if (val < 0) continue; 

      // しきい値の調整を検討してください (例: 200 -> 128)
      unsigned int map_cost = (unsigned int)(val * 2.52);
      if (map_cost > 252) map_cost = 252;
      
      // SNGNNが明確に障害物と判定したもの以外は無視する設定になっています
      if (map_cost < 180) { 
        continue; // FREE_SPACEで上書きするより、何もしない方がマスターコストマップの他のレイヤーを阻害しない場合があります
        // map_cost = FREE_SPACE; 
      }

      double lx = ox + (x + 0.5) * res;
      
      // 回転と移動
      double wx = robot_x_ + lx * c - ly * s;
      double wy = robot_y_ + lx * s + ly * c;
      
      unsigned int mx, my;
      if (master_grid.worldToMap(wx, wy, mx, my)) {
        unsigned char existing = master_grid.getCost(mx, my);
        if (existing == NO_INFORMATION || map_cost > existing) {
          master_grid.setCost(mx, my, static_cast<unsigned char>(map_cost));
        }
      }
    }
  }
}

}  // namespace cabot_sngnn_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cabot_sngnn_layer::SNGNNLayer, nav2_costmap_2d::Layer)
