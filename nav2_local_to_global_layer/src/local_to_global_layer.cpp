#include "nav2_local_to_global_layer/local_to_global_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_local_to_global_layer::LocalToGlobalLayer, nav2_costmap_2d::Layer)

namespace nav2_local_to_global_layer
{

LocalToGlobalLayer::LocalToGlobalLayer() {}
LocalToGlobalLayer::~LocalToGlobalLayer() {}

void LocalToGlobalLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in LocalToGlobalLayer"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue("/local_costmap/costmap"));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_name_);

  RCLCPP_INFO(node->get_logger(), 
    "Initializing LocalToGlobalLayer: %s. Listening to topic: %s", 
    name_.c_str(), topic_name_.c_str());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::SystemDefaultsQoS(),
    std::bind(&LocalToGlobalLayer::incomingCostmapCallback, this, std::placeholders::_1));
    
  current_ = true;
  RCLCPP_INFO(node->get_logger(), "LocalToGlobalLayer initialized successfully.");
}

void LocalToGlobalLayer::incomingCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Log every 10th message to avoid spamming, but confirm data is arriving
  static int count = 0;
  if (count++ % 10 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("LocalToGlobalLayer"), 
      "Received local costmap update #%d. Size: %ux%u", count, msg->info.width, msg->info.height);
  }
  last_local_costmap_ = msg;
}

void LocalToGlobalLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  if (!last_local_costmap_) {
    static rclcpp::Clock wall_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("LocalToGlobalLayer"), wall_clock, 5000, 
      "updateBounds called but no local costmap data received yet.");
    return;
  }

  double local_resolution = last_local_costmap_->info.resolution;
  double local_width = last_local_costmap_->info.width * local_resolution;
  double local_height = last_local_costmap_->info.height * local_resolution;

  geometry_msgs::msg::PointStamped local_origin, global_origin;
  local_origin.header = last_local_costmap_->header;
  local_origin.point = last_local_costmap_->info.origin.position;

  try {
    global_origin = tf_buffer_->transform(local_origin, layered_costmap_->getGlobalFrameID(), tf2::durationFromSec(0.1));
    
    *min_x = std::min(*min_x, global_origin.point.x);
    *min_y = std::min(*min_y, global_origin.point.y);
    *max_x = std::max(*max_x, global_origin.point.x + local_width);
    *max_y = std::max(*max_y, global_origin.point.y + local_height);

    // LOG the updated bounds for debugging
    RCLCPP_WARN(rclcpp::get_logger("LocalToGlobalLayer"), 
      "Updated bounds: min_x=%.2f, min_y=%.2f, max_x=%.2f, max_y=%.2f", 
      *min_x, *min_y, *max_x, *max_y);

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger("LocalToGlobalLayer"), "TF Lookup failed in updateBounds: %s", ex.what());
  }
}

void LocalToGlobalLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_ || !last_local_costmap_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(),
      last_local_costmap_->header.frame_id,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("LocalToGlobalLayer"), "Could not transform local to global: %s", ex.what());
    return;
  }

  unsigned int local_w = last_local_costmap_->info.width;
  unsigned int local_h = last_local_costmap_->info.height;
  double local_res = last_local_costmap_->info.resolution;
  double local_origin_x = last_local_costmap_->info.origin.position.x;
  double local_origin_y = last_local_costmap_->info.origin.position.y;

  int points_added = 0;

  for (unsigned int y = 0; y < local_h; ++y) {
    for (unsigned int x = 0; x < local_w; ++x) {
      int8_t val = last_local_costmap_->data[y * local_w + x];
      unsigned char cost = interpretValue(val);

      if (cost == nav2_costmap_2d::NO_INFORMATION) continue;

      double lx = local_origin_x + (x + 0.5) * local_res;
      double ly = local_origin_y + (y + 0.5) * local_res;

      geometry_msgs::msg::Point pt_local, pt_global;
      pt_local.x = lx;
      pt_local.y = ly;
      
      tf2::doTransform(pt_local, pt_global, transform);

      
      unsigned int mx, my;

      // if (mx >= master_grid.getSizeInCellsX() - 1 || my >= master_grid.getSizeInCellsY() -1) {
      //   continue; // Skip points outside the master grid
      // }

      // continue;

      if (master_grid.worldToMap(pt_global.x, pt_global.y, mx, my)) {
        master_grid.setCost(mx, my, cost);
        // master_grid.setCost(mx+1, my, cost);
        // master_grid.setCost(mx+1, my+1, cost);
        // master_grid.setCost(mx, my+1, cost);
        points_added+=4;
      }
    }
  }
  
  // Debug log to see if points are actually hitting the master grid
  static int update_count = 0;
  if (update_count++ % 20 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("LocalToGlobalLayer"), 
      "updateCosts: Added %d points to the global costmap.", points_added);
  }
}

unsigned char LocalToGlobalLayer::interpretValue(int8_t value)
{
  //return nav2_costmap_2d::FREE_SPACE;

  if (value == -1) return nav2_costmap_2d::LETHAL_OBSTACLE;
  if (value == 0) return nav2_costmap_2d::FREE_SPACE;
  if (value == 100) return nav2_costmap_2d::LETHAL_OBSTACLE;
  return static_cast<unsigned char>(1 + (252 * (value - 1)) / 98);
}

}  // namespace nav2_local_to_global_layer