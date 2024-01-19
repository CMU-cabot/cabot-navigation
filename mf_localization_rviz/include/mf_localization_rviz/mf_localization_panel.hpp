// Copyright (c) 2021  IBM Corporation
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

#ifndef MF_LOCALIZATION_RVIZ__MF_LOCALIZATION_PANEL_HPP_
#define MF_LOCALIZATION_RVIZ__MF_LOCALIZATION_PANEL_HPP_

#include <QPushButton>
#include <QLabel>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cabot_msgs/msg/pose_log.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include "mf_localization_msgs/msg/mf_localize_status.hpp"
#include "mf_localization_msgs/srv/restart_localization.hpp"
#include "mf_localization_msgs/srv/floor_change.hpp"

namespace rviz_common
{

class RVIZ_COMMON_PUBLIC MultifloorLocalizationPanel : public Panel
{
  Q_OBJECT

public:
  explicit MultifloorLocalizationPanel(QWidget * parent = 0);
  virtual ~MultifloorLocalizationPanel();
  void onInitialize() override;
  void load(const rviz_common::Config & config);
  void save(rviz_common::Config config) const;
  void pose_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg);
  void model_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
  void floor_callback(const std_msgs::msg::Int64::SharedPtr msg);
  void floor_raw_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void area_callback(const std_msgs::msg::Int64::SharedPtr msg);
  void status_callback(const mf_localization_msgs::msg::MFLocalizeStatus::SharedPtr msg);
  void updateMessage();

private Q_SLOTS:
  void updateTopic();
  void updateService();
  void sendRestartLocalization();
  void sendFloorUp();
  void sendFloorDown();
  void sendPromptMemo();

private:
  void sendFloorChange(int diff);
  QPushButton * restart_button_;
  QPushButton * up_button_;
  QPushButton * down_button_;
  QPushButton * memo_button_;
  QLabel * statusLabel_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<cabot_msgs::msg::PoseLog>::SharedPtr pose_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr floor_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr floor_raw_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr area_sub_;
  rclcpp::Subscription<mf_localization_msgs::msg::MFLocalizeStatus>::SharedPtr status_sub_;
  rclcpp::Client<mf_localization_msgs::srv::RestartLocalization>::SharedPtr restart_localization_client_;
  rclcpp::Client<mf_localization_msgs::srv::FloorChange>::SharedPtr floor_change_client_;

  cabot_msgs::msg::PoseLog::SharedPtr pose_msg_;
  gazebo_msgs::msg::ModelStates::SharedPtr model_msg_;
  std_msgs::msg::Int64::SharedPtr floor_msg_;
  std_msgs::msg::Float64::SharedPtr floor_raw_msg_;
  std_msgs::msg::Int64::SharedPtr area_msg_;
  mf_localization_msgs::msg::MFLocalizeStatus::SharedPtr status_msg_;
};
}  // namespace rviz_common

#endif  // MF_LOCALIZATION_RVIZ__MF_LOCALIZATION_PANEL_HPP_
