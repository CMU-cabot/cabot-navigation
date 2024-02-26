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

#include <tf2/LinearMath/Quaternion.h>
#include <QFont>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QInputDialog>
#include <memory>
#include "mf_localization_rviz/mf_localization_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rviz_common
{

MultifloorLocalizationPanel::MultifloorLocalizationPanel(QWidget * parent)
: Panel(parent)
{
  // init components
  restart_button_ = new QPushButton("Restart Localization");
  up_button_ = new QPushButton("Up");
  down_button_ = new QPushButton("Down");
  memo_button_ = new QPushButton("Memo");

  auto label = new QLabel("Floor:");
  label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  statusLabel_ = new QLabel("");
  statusLabel_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  QFont font("Monospace");
  font.setStyleHint(QFont::TypeWriter);
  statusLabel_->setFont(font);
  updateMessage();

  // layout
  QVBoxLayout * vlayout = new QVBoxLayout;
  QHBoxLayout * hlayout1 = new QHBoxLayout;
  QHBoxLayout * hlayout2 = new QHBoxLayout;
  QHBoxLayout * hlayout3 = new QHBoxLayout;
  QHBoxLayout * hlayout4 = new QHBoxLayout;

  hlayout1->addWidget(restart_button_);
  hlayout2->addWidget(label);
  hlayout2->addWidget(up_button_);
  hlayout2->addWidget(down_button_);
  hlayout3->addWidget(statusLabel_);
  hlayout4->addWidget(memo_button_);

  auto lineA = new QFrame;
  lineA->setFrameShape(QFrame::HLine);
  lineA->setFrameShadow(QFrame::Sunken);
  auto lineB = new QFrame;
  lineB->setFrameShape(QFrame::HLine);
  lineB->setFrameShadow(QFrame::Sunken);

  vlayout->addLayout(hlayout1);
  vlayout->addLayout(hlayout2);
  vlayout->addWidget(lineA);
  vlayout->addLayout(hlayout3);
  vlayout->addWidget(lineB);
  vlayout->addLayout(hlayout4);

  setLayout(vlayout);

  QObject::connect(restart_button_, SIGNAL(clicked()), this, SLOT(sendRestartLocalization()));
  QObject::connect(up_button_, SIGNAL(clicked()), this, SLOT(sendFloorUp()));
  QObject::connect(down_button_, SIGNAL(clicked()), this, SLOT(sendFloorDown()));
  QObject::connect(memo_button_, SIGNAL(clicked()), this, SLOT(sendPromptMemo()));
}

MultifloorLocalizationPanel::~MultifloorLocalizationPanel()
{
}

void MultifloorLocalizationPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  updateTopic();
  updateService();
}

void MultifloorLocalizationPanel::updateTopic()
{
  pub_ = node_->create_publisher<std_msgs::msg::String>("memo", 1);
  pose_sub_ = node_->create_subscription<cabot_msgs::msg::PoseLog>(
    "/cabot/pose_log", 10,
    std::bind(&MultifloorLocalizationPanel::pose_callback, this, std::placeholders::_1));
  model_sub_ = node_->create_subscription<gazebo_msgs::msg::ModelStates>(
    "/gazebo/model_states", 10,
    std::bind(&MultifloorLocalizationPanel::model_callback, this, std::placeholders::_1));
  floor_sub_ = node_->create_subscription<std_msgs::msg::Int64>(
    "current_floor", 10,
    std::bind(&MultifloorLocalizationPanel::floor_callback, this, std::placeholders::_1));
  floor_raw_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "current_floor_raw", 10,
    std::bind(&MultifloorLocalizationPanel::floor_raw_callback, this, std::placeholders::_1));
  area_sub_ = node_->create_subscription<std_msgs::msg::Int64>(
    "current_area", 10,
    std::bind(&MultifloorLocalizationPanel::area_callback, this, std::placeholders::_1));
  status_sub_ = node_->create_subscription<mf_localization_msgs::msg::MFLocalizeStatus>(
    "localize_status", 10,
    std::bind(&MultifloorLocalizationPanel::status_callback, this, std::placeholders::_1));
}

void MultifloorLocalizationPanel::pose_callback(const cabot_msgs::msg::PoseLog::SharedPtr msg)
{
  pose_msg_ = msg;
  updateMessage();
}

void MultifloorLocalizationPanel::model_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  model_msg_ = msg;
  updateMessage();
}

void MultifloorLocalizationPanel::floor_callback(const std_msgs::msg::Int64::SharedPtr msg)
{
  floor_msg_ = msg;
  updateMessage();
}

void MultifloorLocalizationPanel::floor_raw_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  floor_raw_msg_ = msg;
  updateMessage();
}

void MultifloorLocalizationPanel::area_callback(const std_msgs::msg::Int64::SharedPtr msg)
{
  area_msg_ = msg;
  updateMessage();
}

void MultifloorLocalizationPanel::status_callback(const mf_localization_msgs::msg::MFLocalizeStatus::SharedPtr msg)
{
  status_msg_ = msg;
  updateMessage();
}

// https://stackoverflow.com/a/26221725
template<typename ... Args>
std::string string_format(const std::string & format, Args ... args)
{
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1;  // Extra space for '\0'
  if (size_s <= 0) {throw std::runtime_error("Error during formatting.");}
  size_t size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args ...);
  return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}
// end

void MultifloorLocalizationPanel::updateMessage()
{
  std::string floor = " ";
  std::string floor_str = "   ";
  if (floor_msg_) {
    floor = string_format("%+2d", floor_msg_->data);
    if (floor_msg_->data < 0) {
      floor_str = string_format("B%dF", -floor_msg_->data);
    } else {
      floor_str = string_format("%2dF", floor_msg_->data + 1);
    }
  }
  std::string floor_raw = "     ";
  if (floor_raw_msg_) {
    floor_raw = string_format("%+5.2f", floor_raw_msg_->data);
  }
  std::string area = " ";
  if (area_msg_) {
    area = string_format("%d", area_msg_->data);
  }
  std::string status = "";
  if (status_msg_) {
    if (status_msg_->status == 0) {
      status = "UNKNOWN";
    } else if (status_msg_->status == 1) {
      status = "LOCATING";
    } else if (status_msg_->status == 2) {
      status = "TRACKING";
    } else if (status_msg_->status == 3) {
      status = "UNRELIABLE";
    }
  }
  std::string x = "         ";
  std::string y = "         ";
  std::string a = "    ";
  std::string lat = "         ";
  std::string lng = "         ";
  if (pose_msg_) {
    x = string_format("%+7.2f   ", pose_msg_->pose.position.x);
    y = string_format("%+7.2f   ", pose_msg_->pose.position.y);
    auto q = pose_msg_->pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    a = string_format("%+4.0f", yaw / M_PI * 180);
    lat = string_format("%+10.5f", pose_msg_->lat);
    lng = string_format("%+10.5f", pose_msg_->lng);
  }
  std::string model = "";
  if (model_msg_) {
    unsigned int i;
    for (i = 0; i < model_msg_->name.size(); i++) {
      if (model_msg_->name[i] == "mobile_base") {
        auto gx = string_format("%+7.2f   ", model_msg_->pose[i].position.x);
        auto gy = string_format("%+7.2f   ", model_msg_->pose[i].position.y);
        auto gz = string_format("%+7.2f   ", model_msg_->pose[i].position.z);
        model = string_format("\nGazebo : (%s, %s, %s)", gx.c_str(), gy.c_str(), gz.c_str());
        break;
      }
    }
  }
  QString qString = QString::fromStdString(
    string_format(
      "Status : %s\n"
      "Floor  : %s (f=%s, fr=%s, a=%s)\n"
      "Pose   : (%s, %s) @ %s deg\n"
      "LatLng : (%s, %s)"
      "%s",
      status.c_str(),
      floor_str.c_str(), floor.c_str(), floor_raw.c_str(), area.c_str(),
      x.c_str(), y.c_str(), a.c_str(),
      lat.c_str(), lng.c_str(),
      model.c_str()
  ));
  statusLabel_->setText(qString);
}

void MultifloorLocalizationPanel::updateService()
{
  restart_localization_client_ =
    node_->create_client<mf_localization_msgs::srv::RestartLocalization>("restart_localization");

  floor_change_client_ =
    node_->create_client<mf_localization_msgs::srv::FloorChange>("floor_change");
}

void MultifloorLocalizationPanel::sendRestartLocalization()
{
  RCLCPP_INFO(node_->get_logger(), "sendRestartLocalization");
  auto req = std::make_shared<mf_localization_msgs::srv::RestartLocalization::Request>();
  restart_localization_client_->async_send_request(req);
}

void MultifloorLocalizationPanel::sendFloorUp()
{
  sendFloorChange(+1);
}

void MultifloorLocalizationPanel::sendFloorDown()
{
  sendFloorChange(-1);
}

void MultifloorLocalizationPanel::sendPromptMemo()
{
  bool ok;
  QInputDialog * dialog = new QInputDialog(this);
  dialog->setLabelText(tr("Input Debug Memo"));
  dialog->setInputMode(QInputDialog::InputMode::TextInput);
  dialog->resize(500, 100);
  ok = dialog->exec();
  QString text = dialog->textValue();

  // QString text = QInputDialog::getText(this, tr("Input Debug Memo"),
  // tr("memo:"),
  // tr(""), &ok);

  if (ok && !text.isEmpty()) {
    std_msgs::msg::String msg;
    msg.data = text.toStdString();

    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "DebugMemo: %s", text.toStdString().c_str());
  }
}


void MultifloorLocalizationPanel::sendFloorChange(int diff)
{
  RCLCPP_INFO(node_->get_logger(), "sendFloorChange");
  auto req = std::make_shared<mf_localization_msgs::srv::FloorChange::Request>();
  req->diff.data = diff;
  floor_change_client_->async_send_request(req);
}

void MultifloorLocalizationPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void MultifloorLocalizationPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}


}  // end namespace rviz_common

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_common::MultifloorLocalizationPanel, rviz_common::Panel)
