// Copyright (c) 2026 Carnegie Mellon University
// SPDX-License-Identifier: MIT
#ifndef MF_LOCALIZATION_RVIZ__CONTROLLER_PANEL_HPP_
#define MF_LOCALIZATION_RVIZ__CONTROLLER_PANEL_HPP_

#include <chrono>
#include <memory>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>

namespace mf_localization_rviz
{
class ControllerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ControllerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  using Switch = rcl_interfaces::srv::SetParametersAtomically;
  void apply();
  void refresh();
  QComboBox * mode_;
  QLabel * current_;
  QLabel * result_;
  QPushButton * apply_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Switch>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<Switch>::SharedFuture pending_;
  std::chrono::steady_clock::time_point deadline_;
  int64_t request_id_{0};
  bool initialized_{false};
};
}  // namespace mf_localization_rviz
#endif
