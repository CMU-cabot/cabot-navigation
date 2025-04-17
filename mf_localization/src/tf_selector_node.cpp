// Copyright (c) 2025  IBM Corporation
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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class TFSelecterNode : public rclcpp::Node
{
public:
  TFSelecterNode()
  : Node("tf_select_node")
  {
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_in", 10,
      std::bind(&TFSelecterNode::tf_callback, this, std::placeholders::_1));

    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_out", 10);
  }

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    tf2_msgs::msg::TFMessage filtered_msg;

    for (const auto & transform : msg->transforms) {
      if (transform.header.frame_id == "odom" &&
          transform.child_frame_id == "base_footprint") {
        filtered_msg.transforms.push_back(transform);
      }
      if (transform.header.frame_id == "base_footprint" &&
          transform.child_frame_id == "base_control_shift") {
        filtered_msg.transforms.push_back(transform);
      }
    }

    if (!filtered_msg.transforms.empty()) {
      tf_pub_->publish(filtered_msg);
    }
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFSelecterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}