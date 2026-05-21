// Copyright (c) 2023  Carnegie Mellon University
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

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define NUM_MODES (2)
std::string MODE_NAMES[2] = {"init", "track"};  // NOLINT

typedef struct FloorData
{
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_points_sub;
} FloorData;

class MultiFloorTopicProxy : public rclcpp::Node
{
public:
  MultiFloorTopicProxy()
  : Node("multi_floor_topic_proxy"),
    current_floor(0),
    current_area(0),
    current_mode(0),
    current_frame_id("")
  {
    std::string map_config_file = this->declare_parameter("map_config_file", "");
    bool verbose = this->declare_parameter("verbose", false);

    YAML::Node config = YAML::LoadFile(map_config_file);
    if (!config["map_list"]) {
      RCLCPP_INFO(this->get_logger(), "map_list not found in YAML file");
      return;
    }

    std::string imu_topic_name = this->get_node_topics_interface()->resolve_topic_name("imu");
    std::string points2_topic_name = this->get_node_topics_interface()->resolve_topic_name("points2");
    std::string odom_topic_name = this->get_node_topics_interface()->resolve_topic_name("odom");

    auto latched_qos = rclcpp::QoS(10).transient_local();

    auto map_list = config["map_list"];
    std::unordered_map<int, int> floor_count;
    for (YAML::const_iterator it = map_list.begin(); it != map_list.end(); ++it) {
      YAML::Node map_dict = *it;

      int floor = static_cast<int>(map_dict["floor"].as<double>());
      // Resolve missing area/node_id/frame_id with the same defaults as
      // multi_floor_manager.py:extend_node_parameter_dictionary().
      int area = resolve_area(map_dict, floor, floor_count);
      std::string node_id = resolve_node_id(map_dict, floor, area);
      std::string frame_id = resolve_frame_id(map_dict, node_id);

      subscribe_map_if_needed(map_dict, node_id, frame_id, latched_qos);

      for (int mode = 0; mode < NUM_MODES; mode++) {
        auto mode_str = MODE_NAMES[mode];

        std::string key = getKey(floor, area, mode);

        FloorData floordata = {nullptr, nullptr, nullptr};
        RCLCPP_INFO(
          this->get_logger(), "floor = %d, area = %d, mode=%d, key=%s, node_id=%s, frame_id=%s",
          floor, area, mode, key.c_str(), node_id.c_str(), frame_id.c_str());

        floordata.imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(node_id + "/" + mode_str + imu_topic_name, 1000);
        floordata.points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_id + "/" + mode_str + points2_topic_name, 100);
        floordata.odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(node_id + "/" + mode_str + odom_topic_name, 100);
        floordata.scan_matched_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          node_id + "/" + mode_str + "/scan_matched_points2", 10,
          std::bind(&MultiFloorTopicProxy::scan_matched_points2_callback, this, _1));
        floor_map[key] = floordata;
      }
    }

    current_floor_sub = this->create_subscription<std_msgs::msg::Int64>("current_floor", latched_qos, std::bind(&MultiFloorTopicProxy::current_floor_callback, this, _1));
    current_area_sub = this->create_subscription<std_msgs::msg::Int64>("current_area", latched_qos, std::bind(&MultiFloorTopicProxy::current_area_callback, this, _1));
    current_mode_sub = this->create_subscription<std_msgs::msg::Int64>("current_mode", latched_qos, std::bind(&MultiFloorTopicProxy::current_mode_callback, this, _1));
    current_frame_sub = this->create_subscription<std_msgs::msg::String>("current_frame", latched_qos, std::bind(&MultiFloorTopicProxy::current_frame_callback, this, _1));

    rclcpp::SensorDataQoS sensor_qos;
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu", sensor_qos, std::bind(&MultiFloorTopicProxy::imu_callback, this, _1));
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("points2", sensor_qos, std::bind(&MultiFloorTopicProxy::points_callback, this, _1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", sensor_qos, std::bind(&MultiFloorTopicProxy::odom_callback, this, _1));
    scan_matched_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_matched_points2", 10);
  }

  std::string getKey(const int & floor, const int & area, const int & mode) const
  {
    return std::to_string(floor) + "-" + std::to_string(area) + "-" + std::to_string(mode);
  }

  void scan_matched_points2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    scan_matched_points_pub->publish(*msg);
  }

  void current_floor_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    this->current_floor = msg->data;
    RCLCPP_INFO(this->get_logger(), "floor=%d, area=%d, mode=%d", current_floor, current_area, current_mode);
  }

  void current_area_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    this->current_area = msg->data;
    RCLCPP_INFO(this->get_logger(), "floor=%d, area=%d, mode=%d", current_floor, current_area, current_mode);
  }

  void current_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    this->current_mode = msg->data;
    RCLCPP_INFO(this->get_logger(), "floor=%d, area=%d, mode=%d", current_floor, current_area, current_mode);
  }

  void current_frame_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    this->current_frame_id = msg->data;
    RCLCPP_INFO(this->get_logger(), "current_frame=%s", current_frame_id.c_str());
    publish_cached_map(current_frame_id);
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string & frame_id)
  {
    latest_maps[frame_id] = msg;
    if (frame_id == current_frame_id) {
      publish_map(*msg);
    }
  }

  void publish_cached_map(const std::string & frame_id)
  {
    auto search = latest_maps.find(frame_id);
    if (search == latest_maps.end()) {
      return;
    }
    publish_map(*search->second);
  }

  void publish_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    if (!map_pub) {
      return;
    }
    map_pub->publish(msg);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::string key = getKey(this->current_floor, this->current_area, this->current_mode);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "imu called");
    auto search = floor_map.find(key);
    if (search == floor_map.end()) {
      return;
    }

    double norm_q_tolerance = 0.1;
    double norm_acc_threshold = 0.1;

    auto acc = msg->linear_acceleration;
    auto q = msg->orientation;
    auto norm_acc = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    auto norm_q = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm_acc_threshold > norm_acc || std::abs(norm_q - 1.0) > norm_q_tolerance) {
      RCLCPP_INFO(
        this->get_logger(),
        "imu input is invalid. (linear_acceleration=(%.5f,%.5f,%.5f), orientation=(%.5f,%.5f,%.5f,%.5f))",
        acc.x, acc.y, acc.z, q.x, q.y, q.z, q.w);
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "imu publish");
    search->second.imu_pub->publish(*msg);
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::string key = getKey(this->current_floor, this->current_area, this->current_mode);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "points called");
    auto search = floor_map.find(key);
    if (search == floor_map.end()) {
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "points publish");
    search->second.points_pub->publish(*msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::string key = getKey(this->current_floor, this->current_area, this->current_mode);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "odom called");
    auto search = floor_map.find(key);
    if (search == floor_map.end()) {
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "odom publish");
    search->second.odom_pub->publish(*msg);
  }

private:
  int resolve_area(
    const YAML::Node & map_dict, const int floor, std::unordered_map<int, int> & floor_count)
  {
    int area = 0;
    if (map_dict["area"] && !map_dict["area"].IsNull()) {
      area = map_dict["area"].as<int>();
    } else {
      area = floor_count[floor];
    }
    floor_count[floor] += 1;
    return area;
  }

  std::string resolve_node_id(const YAML::Node & map_dict, const int floor, const int area)
  {
    if (map_dict["node_id"] && !map_dict["node_id"].IsNull()) {
      return map_dict["node_id"].as<std::string>();
    }

    return "carto_" + std::to_string(floor) + "_" + std::to_string(area);
  }

  std::string resolve_frame_id(const YAML::Node & map_dict, const std::string & node_id)
  {
    if (map_dict["frame_id"] && !map_dict["frame_id"].IsNull()) {
      return map_dict["frame_id"].as<std::string>();
    }

    return "map_" + node_id;
  }

  bool has_map_filename(const YAML::Node & map_dict) const
  {
    return map_dict["map_filename"] && !map_dict["map_filename"].IsNull() &&
           !map_dict["map_filename"].as<std::string>().empty();
  }

  void subscribe_map_if_needed(
    const YAML::Node & map_dict, const std::string & node_id, const std::string & frame_id,
    const rclcpp::QoS & latched_qos)
  {
    if (has_map_filename(map_dict)) {
      return;
    }

    std::string map_topic = "/" + node_id + "/map";
    // Create /map publisher only when relaying maps without static map files.
    if (!map_pub) {
      map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched_qos);
    }
    map_subs[frame_id] = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, latched_qos,
      [this, frame_id](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map_callback(msg, frame_id);
      });
    RCLCPP_INFO(
      this->get_logger(), "remap map topic source=%s, frame_id=%s",
      map_topic.c_str(), frame_id.c_str());
  }

  std::unordered_map<std::string, FloorData> floor_map;

  int current_floor;
  int current_area;
  int current_mode;
  std::string current_frame_id;

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr current_floor_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr current_area_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr current_mode_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_frame_sub;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_points_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
  std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subs;
  std::unordered_map<std::string, nav_msgs::msg::OccupancyGrid::SharedPtr> latest_maps;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiFloorTopicProxy>());
  rclcpp::shutdown();
  return 0;
}
