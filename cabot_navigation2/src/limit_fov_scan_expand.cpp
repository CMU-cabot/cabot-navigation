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

#include <time.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace cabot_navigation2
{

class LimitFovScanExpand : public rclcpp::Node
{
public:
  LimitFovScanExpand()
  : Node("limit_fov_scan_expand")
  {
    this->declare_parameter("input_topic", rclcpp::ParameterValue(std::string("scan")));
    std::string input_topic = "scan";
    if (this->get_parameter<std::string>("input_topic", input_topic)) {
      RCLCPP_INFO(this->get_logger(), "input_topic=%s", input_topic.c_str());
    }

    this->declare_parameter("output_topic", rclcpp::ParameterValue(std::string("scan_expand")));
    std::string output_topic = "scan_expand";
    if (this->get_parameter<std::string>("output_topic", output_topic)) {
      RCLCPP_INFO(this->get_logger(), "output_topic=%s", output_topic.c_str());
    }

    this->declare_parameter("expand_angle", rclcpp::ParameterValue(0.0872));
    if (this->get_parameter<double>("expand_angle", expand_angle_)) {
      RCLCPP_INFO(this->get_logger(), "expand_angle=%d", expand_angle_);
    }

    rclcpp::SensorDataQoS sensor_qos;
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, sensor_qos, std::bind(&LimitFovScanExpand::scan_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, sensor_qos);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr input_msg)
  {
    int input_ranges_size = std::ceil((input_msg->angle_max - input_msg->angle_min) / input_msg->angle_increment);
    int expand_ranges_size = std::ceil(expand_angle_ / input_msg->angle_increment);

    // check if the min angle and max angle are free
    bool is_angle_min_free = (input_msg->ranges[0] > input_msg->range_max) ? true : false;
    bool is_angle_max_free = (input_msg->ranges[input_ranges_size - 1] > input_msg->range_max) ? true : false;

    // calculate output range size and check range value for free area
    double free_range = std::numeric_limits<double>::infinity();
    int output_ranges_size = input_ranges_size;
    if (is_angle_min_free) {
      output_ranges_size += expand_ranges_size;
      free_range = input_msg->ranges[0];
    }
    if (is_angle_max_free) {
      output_ranges_size += expand_ranges_size;
      free_range = input_msg->ranges[input_ranges_size - 1];
    }

    // create the scan message that expand fov if the min angle and max angle are free
    auto output_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    output_msg->header = input_msg->header;
    if (is_angle_min_free) {
      output_msg->angle_min = input_msg->angle_min - input_msg->angle_increment * expand_ranges_size;
    } else {
      output_msg->angle_min = input_msg->angle_min;
    }
    if (is_angle_max_free) {
      output_msg->angle_max = input_msg->angle_max + input_msg->angle_increment * expand_ranges_size;
    } else {
      output_msg->angle_max = input_msg->angle_max;
    }
    output_msg->angle_increment = input_msg->angle_increment;
    output_msg->time_increment = input_msg->time_increment;
    output_msg->scan_time = input_msg->scan_time;
    output_msg->range_min = input_msg->range_min;
    output_msg->range_max = input_msg->range_max;

    output_msg->ranges.reserve(output_ranges_size);
    if (is_angle_min_free) {
      for (int i = 0; i < expand_ranges_size; i++) {
        output_msg->ranges.push_back(free_range);
      }
    }
    for (int i = 0; i < input_ranges_size; i++) {
      output_msg->ranges.push_back(input_msg->ranges[i]);
    }
    if (is_angle_max_free) {
      for (int i = 0; i < expand_ranges_size; i++) {
        output_msg->ranges.push_back(free_range);
      }
    }

    pub_->publish(std::move(output_msg));
  }

  double expand_angle_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

}  // namespace cabot_navigation2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cabot_navigation2::LimitFovScanExpand>());
  rclcpp::shutdown();
  return 0;
}
