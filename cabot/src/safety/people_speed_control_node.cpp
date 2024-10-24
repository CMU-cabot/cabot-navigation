// Copyright (c) 2020, 2022  Carnegie Mellon University
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

// People speed control
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <limits>
#include <sstream>

#include <cabot/util.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace CaBotSafety
{
// utility struct and functions
// this could be moved to somewhere else
class PeopleSpeedControlNode : public rclcpp::Node
{
public:
  std::string people_topic_;
  std::string vis_topic_;
  std::string limit_topic_;
  std::string odom_topic_;
  std::string plan_topic_;
  std::string event_topic_;
  std::string set_social_distance_topic_;
  std::string get_social_distance_topic_;

  std::string map_frame_;
  std::string robot_base_frame_;

  double max_speed_;
  double min_speed_;
  double max_acc_;
  double delay_;
  double social_distance_x_;
  double social_distance_y_;
  double no_people_topic_max_speed_;
  bool no_people_flag_;

  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_social_distance_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr get_social_distance_pub_;
  std::mutex thread_sync_;
  tf2_ros::TransformListener * tfListener;
  tf2_ros::Buffer * tfBuffer;
  nav_msgs::msg::Odometry last_odom_;
  nav_msgs::msg::Path last_plan_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  rclcpp::TimerBase::SharedPtr people_topic_check_timer_;
  rclcpp::Time last_people_message_time_;

  explicit PeopleSpeedControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("people_speed_control_node", options),
    people_topic_("/people"),
    vis_topic_("/visualize"),
    limit_topic_("/people_limit"),
    odom_topic_("/odom"),
    plan_topic_("/plan"),
    event_topic_("/event"),
    set_social_distance_topic_("/set_social_distance"),
    get_social_distance_topic_("/get_social_distance"),
    map_frame_("map"),
    robot_base_frame_("base_footprint"),
    max_speed_(1.0),
    min_speed_(0.1),
    max_acc_(0.5),
    delay_(0.5),
    social_distance_x_(2.0),
    social_distance_y_(1.0),
    no_people_topic_max_speed_(0.5),
    no_people_flag_(false)
  {
    RCLCPP_INFO(get_logger(), "PeopleSpeedControlNodeClass Constructor");
    tfBuffer = new tf2_ros::Buffer(get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer, this);

    RCLCPP_INFO(get_logger(), "People speed control - %s", __FUNCTION__);

    people_topic_ = declare_parameter("people_topic", people_topic_);
    people_sub_ =
      create_subscription<people_msgs::msg::People>(
      people_topic_, 10,
      std::bind(&PeopleSpeedControlNode::peopleCallback, this, std::placeholders::_1));

    odom_topic_ = declare_parameter("odom_topic", odom_topic_);
    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, std::bind(&PeopleSpeedControlNode::odomCallback, this, std::placeholders::_1));

    plan_topic_ = declare_parameter("plan_topic", plan_topic_);
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, 10, std::bind(
        &PeopleSpeedControlNode::planCallback, this,
        std::placeholders::_1));

    vis_topic_ = declare_parameter("visualize_topic", vis_topic_);
    vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(vis_topic_, 100);

    limit_topic_ = declare_parameter("limit_topic", limit_topic_);
    limit_pub_ = create_publisher<std_msgs::msg::Float32>(limit_topic_, rclcpp::SystemDefaultsQoS().transient_local());

    event_topic_ = declare_parameter("event_topic", event_topic_);
    event_pub_ = create_publisher<std_msgs::msg::String>(event_topic_, 100);

    max_speed_ = declare_parameter("max_speed_", max_speed_);
    min_speed_ = declare_parameter("min_speed_", min_speed_);
    max_acc_ = declare_parameter("max_acc_", max_acc_);
    social_distance_x_ = declare_parameter("social_distance_x", social_distance_x_);
    social_distance_y_ = declare_parameter("social_distance_y", social_distance_y_);
    no_people_topic_max_speed_ = declare_parameter("no_people_topic_max_speed", no_people_topic_max_speed_);

    RCLCPP_INFO(
      get_logger(), "PeopleSpeedControl with max_speed=%.2f, social_distance=(%.2f, %.2f)",
      max_speed_, social_distance_x_, social_distance_y_);

    set_social_distance_topic_ = declare_parameter("set_social_distance_topic", set_social_distance_topic_);
    set_social_distance_sub_ = create_subscription<geometry_msgs::msg::Point>(
      set_social_distance_topic_, 10,
      std::bind(&PeopleSpeedControlNode::setSocialDistanceCallback, this, std::placeholders::_1));

    get_social_distance_topic_ = declare_parameter("get_social_distance_topic", get_social_distance_topic_);
    get_social_distance_pub_ = create_publisher<geometry_msgs::msg::Point>(get_social_distance_topic_, rclcpp::SystemDefaultsQoS().transient_local());
    geometry_msgs::msg::Point msg;
    msg.x = social_distance_x_;
    msg.y = social_distance_y_;
    get_social_distance_pub_->publish(msg);

    callback_handler_ =
      add_on_set_parameters_callback(std::bind(&PeopleSpeedControlNode::param_set_callback, this, std::placeholders::_1));

    last_people_message_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    people_topic_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PeopleSpeedControlNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "PeopleSpeedControl with max_speed=%.2f, social_distance=(%.2f, %.2f)",
      max_speed_, social_distance_x_, social_distance_y_);
  }

  ~PeopleSpeedControlNode()
  {
    RCLCPP_INFO(get_logger(), "PeopleSpeedControlNodeClass Destructor");
    people_sub_.reset();
    odom_sub_.reset();
    plan_sub_.reset();
    vis_pub_.reset();
    limit_pub_.reset();
    event_pub_.reset();
    set_social_distance_sub_.reset();
    get_social_distance_pub_.reset();
    delete tfListener;
    delete tfBuffer;
    callback_handler_.reset();
    people_topic_check_timer_.reset();
  }

  // check if the last people message comes within one second
  void timer_callback()
  {
    // ignore if there is no people message from the beginning (simulation without people)
    if (last_people_message_time_.nanoseconds() == 0) {
      return;
    }
    rclcpp::Time now = this->get_clock()->now();
    int64_t nseconds_since_last_message = (now - last_people_message_time_).nanoseconds();

    if (no_people_flag_) {
      if (nseconds_since_last_message < 1000000000) {
        // restored to OK
        no_people_flag_ = false;
      } else {
        // keep NO_PEOPLE
        std_msgs::msg::Float32 msg;
        msg.data = no_people_topic_max_speed_;
        limit_pub_->publish(msg);
      }
    } else {
      if (nseconds_since_last_message > 1000000000) {
        // OK -> NO_PEOPLE
        no_people_flag_ = true;
      }
      // else keep OK
    }
  }

  rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
    for (auto && param : params) {
      if (!has_parameter(param.get_name())) {
        continue;
      }
      RCLCPP_DEBUG(get_logger(), "change param %s", param.get_name().c_str());

      if (param.get_name() == "max_speed_") {
        max_speed_ = param.as_double();
      }
      if (param.get_name() == "min_speed_") {
        min_speed_ = param.as_double();
      }
      if (param.get_name() == "max_acc_") {
        max_acc_ = param.as_double();
      }
      if (param.get_name() == "social_distance_x") {
        social_distance_x_ = param.as_double();
      }
      if (param.get_name() == "social_distance_y") {
        social_distance_y_ = param.as_double();
      }
    }
    results->successful = true;
    return *results;
  }

private:
  void peopleCallback(const people_msgs::msg::People::SharedPtr input)
  {
    last_people_message_time_ = this->get_clock()->now();
    if (last_plan_.poses.size() == 0) {
      auto & clk = *this->get_clock();
      RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "PeopleSpeedControl no plan");
      return;
    }

    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = tfBuffer->lookupTransform(
        "base_footprint", input->header.frame_id,
        rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return;
    }
    tf2::Stamped<tf2::Transform> transform_tf2;
    tf2::fromMsg(transform_msg, transform_tf2);
    auto rotation_tf2 = transform_tf2;
    rotation_tf2.setOrigin(tf2::Vector3(0, 0, 0));

    double speed_limit = max_speed_;

    std::lock_guard<std::mutex> lock(thread_sync_);
    for (auto it = input->people.begin(); it != input->people.end(); it++) {
      tf2::Vector3 p_frame(it->position.x, it->position.y, 0);
      tf2::Vector3 v_frame(it->velocity.x, it->velocity.y, 0);

      auto p_local = transform_tf2 * p_frame;
      auto v_local = rotation_tf2 * v_frame;

      double x = p_local.x();
      double y = p_local.y();
      double vx = v_local.x();
      double vy = v_local.y();
      double vr = last_odom_.twist.twist.linear.x;

      // velocity obstacle
      double RPy = atan2(y, x);
      double dist = hypot(x, y);
      if (vx > 0 || vy > 0) {
        double pr = 1.0;
        double s = atan2(pr, dist) + M_PI_2;
        double Px1 = x + cos(RPy + s) * pr;
        double Py1 = y + sin(RPy + s) * pr;
        double Px2 = x + cos(RPy - s) * pr;
        double Py2 = y + sin(RPy - s) * pr;
        double v1 = vy / -(Py1 / Px1);
        double t1 = -Py1 / vy;
        double v2 = vy / -(Py2 / Px2);
        double t2 = -Py2 / vy;
        bool swapped = false;
        if ((0 < t1 && t1 < 5) || (0 < t2 && t2 < 5)) {
          v1 -= vx;
          v2 -= vx;
          if (t1 < t2) {
            double temp = v1;
            v1 = v2;
            v2 = temp;
            temp = t1;
            t1 = t2;
            t2 = temp;
            swapped = true;
          }
          speed_limit = std::max(0.0, v1);
          if (v1 < v2 && v2 < 1.0) {
            speed_limit = max_speed_;
          }
          RCLCPP_INFO(
            get_logger(),
            "PeopleSpeedControl collision cone (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)"
            " - %.2f - %.2f, (%d), %.2f, %.2f",
            x, y, vx, vy, Px1, Py1, Px2, Py2, t1, t2, v1, v2, vr, speed_limit, swapped, RPy, s);
        }
      }

      // social distance
      double sdx = abs(social_distance_x_ * cos(RPy));
      double sdy = abs(social_distance_y_ * sin(RPy));
      double min_path_dist = 100;

      if (abs(RPy) > M_PI_2) {
        RCLCPP_INFO(get_logger(), "PeopleSpeedControl person is back %.2f", RPy);
        continue;
      }
      auto max_v = [](double D, double A, double d)
        {
          return (-2 * A * d + sqrt(4 * A * A * d * d + 8 * A * D)) / 2;
        };

      for (auto pose : last_plan_.poses) {
        tf2::Vector3 p_frame(pose.pose.position.x, pose.pose.position.y, 0);
        auto p_local = transform_tf2 * p_frame;
        auto dx = p_local.x() - x;
        auto dy = p_local.y() - y;
        auto dist = sqrt(dx * dx + dy * dy);
        if (dist < min_path_dist) {
          min_path_dist = dist;
        }
      }
      if (min_path_dist > social_distance_y_) {
        speed_limit = std::min(speed_limit, max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));
      } else {
        speed_limit = std::min(speed_limit, max_v(std::max(0.0, dist - social_distance_x_), max_acc_, delay_));
      }

      if (speed_limit < min_speed_) {
        speed_limit = 0;
      }

      RCLCPP_INFO(
        get_logger(), "PeopleSpeedControl people_limit %s, %.2f %.2f (%.2f %.2f) - %.2f (%.2f)",
        it->name.c_str(), min_path_dist, dist, social_distance_x_, social_distance_y_, speed_limit,
        max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));

      if (speed_limit < max_speed_) {
        std_msgs::msg::String msg;

        if (fabs(speed_limit) < 0.01) {
          msg.data = "navigation;event;people_speed_stopped";
        } else if (vx > 0.25 && speed_limit < max_speed_ * 0.75) {
          msg.data = "navigation;event;people_speed_following";
        }

        if (!msg.data.empty()) {
          RCLCPP_INFO(get_logger(), "PeopleSpeedControl %s", msg.data.c_str());
          event_pub_->publish(msg);
        }
      }
    }

    std_msgs::msg::Float32 msg;
    msg.data = speed_limit;
    // RCLCPP_INFO(get_logger(), "limit = %.2f", speed_limit);
    limit_pub_->publish(msg);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    last_odom_ = *input;
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr input)
  {
    RCLCPP_INFO(get_logger(), "PeopleSpeedControl got plan");
    last_plan_ = *input;
  }

  void setSocialDistanceCallback(const geometry_msgs::msg::Point::SharedPtr input)
  {
    std::lock_guard<std::mutex> lock(thread_sync_);

    social_distance_x_ = input->x;
    social_distance_y_ = input->y;

    geometry_msgs::msg::Point msg;
    msg.x = social_distance_x_;
    msg.y = social_distance_y_;
    get_social_distance_pub_->publish(msg);

    RCLCPP_ERROR(get_logger(), "PeopleSpeedControl social distance with topic is deprecated");
    RCLCPP_INFO(
      get_logger(), "PeopleSpeedControl setSocialDistanceCallback social_distance=(%.2f, %.2f)",
      social_distance_x_, social_distance_y_);
  }
};  // class PeopleSpeedControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::PeopleSpeedControlNode)
