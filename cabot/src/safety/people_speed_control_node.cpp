// Copyright (c) 2020, 2022, 2024  Carnegie Mellon University and Miraikan
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
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

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

namespace
{
constexpr double pr = 1.0;
constexpr double epsilon = std::numeric_limits<double>::epsilon();
}  // namespace

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
  double social_distance_min_radius_;
  double social_distance_min_angle_;  // [rad]
  double social_distance_max_angle_;  // [rad]
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
    social_distance_min_radius_(0.45),
    social_distance_min_angle_(-M_PI_2),
    social_distance_max_angle_(M_PI_2),
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
    social_distance_min_radius_ = declare_parameter("social_distance_min_radius", social_distance_min_radius_);
    social_distance_min_angle_ = declare_parameter("social_distance_min_angle", social_distance_min_angle_);
    social_distance_max_angle_ = declare_parameter("social_distance_max_angle", social_distance_max_angle_);
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
      if (param.get_name() == "social_distance_min_radius") {
        social_distance_min_radius_ = param.as_double();
      }
      if (param.get_name() == "social_distance_min_angle") {
        social_distance_min_angle_ = param.as_double();
      }
      if (param.get_name() == "social_distance_max_angle") {
        social_distance_max_angle_ = param.as_double();
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

    geometry_msgs::msg::TransformStamped map_to_robot_msg;
    tf2::Stamped<tf2::Transform> map_to_robot_tf2;
    if (!lookupTransform("map", "base_footprint", map_to_robot_msg, map_to_robot_tf2)) {
      return;
    }

    geometry_msgs::msg::TransformStamped robot_to_map_global_msg;
    tf2::Stamped<tf2::Transform> robot_to_map_global_transform_tf2;
    if (!lookupTransform("base_footprint", input->header.frame_id, robot_to_map_global_msg, robot_to_map_global_transform_tf2)) {
      return;
    }

    auto robot_to_map_global_rotation_tf2 = robot_to_map_global_transform_tf2;
    robot_to_map_global_rotation_tf2.setOrigin(tf2::Vector3(0, 0, 0));

    std::lock_guard<std::mutex> lock(thread_sync_);

    std::vector<std::pair<tf2::Vector3, tf2::Vector3>> transformed_people;
    for (const auto & person : input->people) {
      tf2::Vector3 p_frame(person.position.x, person.position.y, 0);
      tf2::Vector3 v_frame(person.velocity.x, person.velocity.y, 0);

      auto p_local = robot_to_map_global_transform_tf2 * p_frame;
      auto v_local = robot_to_map_global_rotation_tf2 * v_frame;

      transformed_people.emplace_back(p_local, v_local);
    }

    // Social Distance
    double social_speed_limit = max_speed_;
    for (size_t i = 0; i < input->people.size(); ++i) {
      const auto & person = input->people[i];
      const auto & p_local = transformed_people[i].first;
      const auto & v_local = transformed_people[i].second;

      double x = p_local.x();
      double y = p_local.y();
      // ignore people if it is in the minimum radius
      if (std::hypot(x, y) <= social_distance_min_radius_) {
        RCLCPP_INFO(get_logger(), "PeopleSpeedControl ignore %s, (%.2f %.2f)", person.name.c_str(), x, y);
        continue;
      }

      double vx = v_local.x();

      double RPy = atan2(y, x);
      double dist = hypot(x, y);
      double min_path_dist = 100;

      if (RPy < social_distance_min_angle_ || social_distance_max_angle_ < RPy) {
        RCLCPP_INFO(
          get_logger(), "PeopleSpeedControl person %s (RPy=%.2f) is out of angle range [%.2f, %.2f]",
          person.name.c_str(), RPy, social_distance_min_angle_, social_distance_max_angle_);
        continue;
      }

      auto max_v = [](double D, double A, double d)
        {
          return (-2 * A * d + sqrt(4 * A * A * d * d + 8 * A * D)) / 2;
        };

      for (auto pose : last_plan_.poses) {
        tf2::Vector3 p_frame(pose.pose.position.x, pose.pose.position.y, 0);
        auto p_local = robot_to_map_global_transform_tf2 * p_frame;
        auto dx = p_local.x() - x;
        auto dy = p_local.y() - y;
        auto dist = sqrt(dx * dx + dy * dy);
        if (dist < min_path_dist) {
          min_path_dist = dist;
        }
      }
      if (min_path_dist > social_distance_y_) {
        social_speed_limit = std::min(social_speed_limit, max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));
      } else {
        social_speed_limit = std::min(social_speed_limit, max_v(std::max(0.0, dist - social_distance_x_), max_acc_, delay_));
      }

      if (social_speed_limit < min_speed_) {
        social_speed_limit = 0;
      }

      RCLCPP_INFO(
        get_logger(), "PeopleSpeedControl people_limit %s, %.2f %.2f (%.2f %.2f) - %.2f (%.2f)",
        person.name.c_str(), min_path_dist, dist, social_distance_x_, social_distance_y_, social_speed_limit,
        max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));

      if (social_speed_limit < max_speed_) {
        std_msgs::msg::String msg;

        if (fabs(social_speed_limit) < 0.01) {
          msg.data = "navigation;event;people_speed_stopped";
        } else if (vx > 0.25 && social_speed_limit < max_speed_ * 0.75) {
          msg.data = "navigation;event;people_speed_following";
        }

        if (!msg.data.empty()) {
          RCLCPP_INFO(get_logger(), "PeopleSpeedControl %s", msg.data.c_str());
          event_pub_->publish(msg);
        }
      }
    }

    // Velocity Obstacle
    std::vector<std::pair<double, double>> vo_intervals;
    for (size_t i = 0; i < input->people.size(); ++i) {
      const auto & p_local = transformed_people[i].first;
      const auto & v_local = transformed_people[i].second;

      double x = p_local.x();
      double y = p_local.y();
      double vx = v_local.x();
      double vy = v_local.y();

      double RPy = atan2(y, x);
      double dist = hypot(x, y);
      double s = asin(pr / dist);
      double theta_right = normalizedAngle(RPy - s);
      double theta_left = normalizedAngle(RPy + s);

      addVOMarker(dist, vx, vy, theta_right, theta_left, map_to_robot_tf2);

      if (isWithinVelocityObstacle(vx, vy, theta_right, theta_left)) {
        continue;
      }

      auto compute_velocity = [&](double theta) -> std::optional<double> {
          double t = -vy / sin(theta);
          return (t >= 0) ? std::make_optional(vx + t * cos(theta)) : std::nullopt;
        };

      if (std::abs(theta_right) < epsilon || std::abs(theta_right - M_PI) < epsilon) {
        addVOInterval(compute_velocity(theta_left), vo_intervals);
      } else if (std::abs(theta_left) < epsilon || std::abs(theta_left - M_PI) < epsilon) {
        addVOInterval(compute_velocity(theta_right), vo_intervals);
      } else {
        auto v_right = compute_velocity(theta_right);
        auto v_left = compute_velocity(theta_left);

        if (v_right && v_left) {
          double v_min = std::min(v_right.value(), v_left.value());
          double v_max = std::max(v_right.value(), v_left.value());
          if (0.0 < v_min && v_min < max_speed_) {
            vo_intervals.emplace_back(v_min, std::min(v_max, max_speed_));
          }
        } else {
          addVOInterval(v_right, vo_intervals);
          addVOInterval(v_left, vo_intervals);
        }
      }
    }

    double people_speed_limit = computeSafeSpeedLimit(social_speed_limit, vo_intervals);

    std_msgs::msg::Float32 msg;
    msg.data = people_speed_limit;
    // RCLCPP_INFO(get_logger(), "limit = %.2f", people_speed_limit);
    limit_pub_->publish(msg);

    addSpeedLimitMarker(map_to_robot_tf2, people_speed_limit);
    CaBotSafety::commit(vis_pub_);
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

  void addVOMarker(
    double dist, double vx, double vy, double theta_right, double theta_left,
    const tf2::Stamped<tf2::Transform> & map_to_robot_tf2)
  {
    double visualized_dist = dist + pr;

    CaBotSafety::Point origin_point(vx, vy);
    CaBotSafety::Point right_point(vx + cos(theta_right) * visualized_dist, vy + sin(theta_right) * visualized_dist);
    CaBotSafety::Point left_point(vx + cos(theta_left) * visualized_dist, vy + sin(theta_left) * visualized_dist);

    origin_point.transform(map_to_robot_tf2);
    right_point.transform(map_to_robot_tf2);
    left_point.transform(map_to_robot_tf2);

    std::array<CaBotSafety::Point, 3> points = {origin_point, right_point, left_point};
    CaBotSafety::add_triangle(this->get_clock()->now(), points, 1.0, 0.0, 0.0, 0.0, 0.6);
  }

  void addSpeedLimitMarker(
    const tf2::Stamped<tf2::Transform> & map_to_robot_tf2, double people_speed_limit)
  {
    tf2::Transform robot_pose(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));
    robot_pose *= map_to_robot_tf2;

    CaBotSafety::add_point(this->get_clock()->now(), robot_pose, 0.2, 0, 1, 0, 1);

    CaBotSafety::Point start_point(0.0, 0.0);
    start_point.transform(map_to_robot_tf2);
    CaBotSafety::Point end_point(people_speed_limit, 0.0);
    end_point.transform(map_to_robot_tf2);

    CaBotSafety::Line arrow_line(start_point, end_point);
    CaBotSafety::add_arrow(this->get_clock()->now(), arrow_line, 0.1, 1, 0, 0, 1);

    char buff[100];
    snprintf(buff, sizeof(buff), "limit - %.2fm/s", people_speed_limit);
    CaBotSafety::add_text(this->get_clock()->now(), buff, start_point);
  }

  void addVOInterval(
    std::optional<double> v, std::vector<std::pair<double, double>> & vo_intervals)
  {
    if (v && 0.0 < v.value() && v.value() < max_speed_) {
      vo_intervals.emplace_back(v.value(), max_speed_);
    }
  }

  std::vector<std::pair<double, double>> mergeIntervals(const std::vector<std::pair<double, double>> & intervals)
  {
    if (intervals.empty()) {
      return {};
    }
    if (intervals.size() == 1) {
      return intervals;
    }
    std::vector<std::pair<double, double>> sorted_intervals = intervals;
    std::sort(
      sorted_intervals.begin(), sorted_intervals.end(),
      [](const std::pair<double, double> & a, const std::pair<double, double> & b) {
        return a.first < b.first;
      });
    std::vector<std::pair<double, double>> merged;
    std::pair<double, double> current = sorted_intervals[0];
    for (const auto & sorted_interval : sorted_intervals) {
      if (current.second >= sorted_interval.first) {
        current.second = std::max(current.second, sorted_interval.second);
      } else {
        merged.emplace_back(current);
        current = sorted_interval;
      }
    }
    merged.emplace_back(current);
    return merged;
  }

  std::vector<std::pair<double, double>> substractIntervals(
    const std::pair<double, double> & interval_a,
    const std::vector<std::pair<double, double>> & intervals_b)
  {
    std::vector<std::pair<double, double>> result;
    double current_start = interval_a.first;
    double current_end = interval_a.second;
    for (const auto & b : intervals_b) {
      if (b.second <= current_start) {
        continue;
      }
      if (b.first >= current_end) {
        continue;
      }
      if (b.first > current_start) {
        result.emplace_back(current_start, std::min(current_end, b.first));
      }
      current_start = std::max(current_start, b.second);
      if (current_start >= current_end) {
        break;
      }
    }
    if (current_start < current_end) {
      result.emplace_back(current_start, current_end);
    }
    return result;
  }

  bool isWithinVelocityObstacle(double vx, double vy, double theta_right, double theta_left)
  {
    if (std::abs(vx) < epsilon && std::abs(vy) < epsilon) {
      return true;
    }

    double inv_person_vel_angle = normalizedAngle(atan2(-vy, -vx));
    if (theta_left < theta_right) {
      return inv_person_vel_angle <= theta_left || inv_person_vel_angle >= theta_right;
    }
    return theta_right <= inv_person_vel_angle && inv_person_vel_angle <= theta_left;
  }

  bool lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform_msg,
    tf2::Stamped<tf2::Transform> & transform_tf2)
  {
    try {
      transform_msg = tfBuffer->lookupTransform(
        target_frame, source_frame,
        rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));
      tf2::fromMsg(transform_msg, transform_tf2);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return false;
    }
    return true;
  }

  double findMaxValue(const std::vector<std::pair<double, double>> & intervals)
  {
    double max_value = intervals[0].second;
    for (const auto & interval : intervals) {
      if (interval.second > max_value) {
        max_value = interval.second;
      }
    }
    return max_value;
  }

  double computeSafeSpeedLimit(
    double social_speed_limit, const std::vector<std::pair<double, double>> & intervals)
  {
    if (social_speed_limit <= 0.0) {
      return social_speed_limit;
    }

    auto merged_intervals = mergeIntervals(intervals);
    double vr = last_odom_.twist.twist.linear.x;
    double max_speed_limit = social_speed_limit;

    for (const auto & merged_interval : merged_intervals) {
      if (merged_interval.first < vr && vr < merged_interval.second) {
        max_speed_limit = std::min(vr, max_speed_);
      }
    }

    auto substract_intervals = substractIntervals({0.0, max_speed_limit}, merged_intervals);
    if (substract_intervals.empty()) {
      return social_speed_limit;
    }
    return findMaxValue(substract_intervals);
  }

  inline double normalizedAngle(double theta)
  {
    while (theta > M_PI) {
      theta -= 2.0 * M_PI;
    }
    while (theta <= -M_PI) {
      theta += 2.0 * M_PI;
    }
    return theta;
  }
};  // class PeopleSpeedControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::PeopleSpeedControlNode)
