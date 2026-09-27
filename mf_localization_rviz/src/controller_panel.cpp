// Copyright (c) 2026 Carnegie Mellon University
// SPDX-License-Identifier: MIT
#include "mf_localization_rviz/controller_panel.hpp"
#include <QHBoxLayout>
#include <QDockWidget>
#include <QMainWindow>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mf_localization_rviz
{
ControllerPanel::ControllerPanel(QWidget * parent) : Panel(parent)
{
  mode_ = new QComboBox;
  mode_->addItems({"follow", "crowdattn", "mpc", "rl", "hybrid", "sm"});
  mode_->setObjectName("controllerMode");
  current_ = new QLabel("Current: waiting for controller");
  result_ = new QLabel("Cancel navigation and stop before applying.");
  result_->setWordWrap(true);
  apply_ = new QPushButton("Apply");
  apply_->setObjectName("applyController");
  apply_->setEnabled(false);
  auto row = new QHBoxLayout;
  row->addWidget(mode_);
  row->addWidget(apply_);
  auto layout = new QVBoxLayout;
  layout->addWidget(current_);
  layout->addLayout(row);
  layout->addWidget(result_);
  setLayout(layout);
  connect(apply_, &QPushButton::clicked, this, &ControllerPanel::apply);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &ControllerPanel::refresh);
}

void ControllerPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node_->get_node_base_interface());
  client_ = node_->create_client<Switch>(
    "/cabot/set_controller", rmw_qos_profile_services_default, group_);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/cabot/controller_mode", rclcpp::QoS(10).transient_local().reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      current_->setText("Current: " + QString::fromStdString(msg->data));
      if (!initialized_) {
        mode_->setCurrentText(QString::fromStdString(msg->data));
        initialized_ = true;
      }
    }, options);
  timer_->start(100);
  // Run after RViz restores the saved dock layout, including hidden side docks.
  QTimer::singleShot(500, this, [this]() {
    auto dock = qobject_cast<QDockWidget *>(parentWidget());
    auto main = qobject_cast<QMainWindow *>(window());
    if (dock && main) {
      main->addDockWidget(Qt::TopDockWidgetArea, dock);
      dock->show();
    }
  });
}

void ControllerPanel::apply()
{
  if (!client_ || !client_->service_is_ready() || pending_.valid()) {
    return;
  }
  auto request = std::make_shared<Switch::Request>();
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = "controller";
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  parameter.value.string_value = mode_->currentText().toStdString();
  request->parameters.push_back(parameter);
  auto future = client_->async_send_request(request);
  request_id_ = future.request_id;
  pending_ = future.share();
  deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(120);
  apply_->setEnabled(false);
  mode_->setEnabled(false);
  result_->setText("Loading " + mode_->currentText() + "…");
}

void ControllerPanel::refresh()
{
  executor_.spin_some(std::chrono::milliseconds(5));
  if (pending_.valid()) {
    if (pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto result = pending_.get()->result;
        result_->setText((result.successful ? "Applied: " : "Not changed: ") +
          QString::fromStdString(result.reason));
      } catch (const std::exception & error) {
        result_->setText("Service error: " + QString::fromUtf8(error.what()));
      }
      pending_ = {};
    } else if (std::chrono::steady_clock::now() > deadline_) {
      client_->remove_pending_request(request_id_);
      pending_ = {};
      result_->setText("Response timed out. Check Current before retrying.");
    }
  }
  mode_->setEnabled(!pending_.valid());
  apply_->setEnabled(initialized_ && client_->service_is_ready() && !pending_.valid());
}
}  // namespace mf_localization_rviz

PLUGINLIB_EXPORT_CLASS(mf_localization_rviz::ControllerPanel, rviz_common::Panel)
