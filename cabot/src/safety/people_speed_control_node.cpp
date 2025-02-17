// Copyright (c) 2020, 2024  Carnegie Mellon University and Miraikan
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
#include <rcutils/logging.h>

#include <cmath>
#include <limits>
#include <optional>
#include <queue>
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
constexpr double epsilon = std::numeric_limits<double>::epsilon();  // machine epsilon
}  // namespace

namespace CaBotSafety
{
// utility struct and functions
// this could be moved to somewhere else

struct VOData
{
  double dist;
  double rel_x;
  double rel_y;
  double pvx;
  double pvy;
  double theta_right;
  double theta_left;
  double vo_intersection_min;
  double vo_intersection_max;
};

struct CompareVOIntersectionMax
{
  bool operator()(const VOData & a, const VOData & b)
  {
    return a.vo_intersection_max < b.vo_intersection_max;
  }
};

class PeopleSpeedControlNode : public rclcpp::Node
{
public:
  std::string people_topic_;
  std::string vis_topic_;
  std::string limit_topic_;
  std::string social_distance_limit_topic_;
  std::string pure_velocity_obstacle_limit_topic_;
  std::string combined_speed_limit_topic_;
  std::string odom_topic_;
  std::string plan_topic_;
  std::string event_topic_;
  std::string set_social_distance_topic_;
  std::string get_social_distance_topic_;

  std::string map_frame_;
  std::string robot_base_frame_;

  double max_speed_;
  double sd_min_speed_;
  double vo_min_speed_;
  double max_acc_;
  double max_dec_;
  double delay_;
  double social_distance_x_;
  double social_distance_y_;
  double social_distance_min_radius_;
  double social_distance_min_angle_;  // [rad]
  double social_distance_max_angle_;  // [rad]
  double no_people_topic_max_speed_;
  double collision_time_horizon_;
  double person_speed_threshold_;
  double forward_approach_angle_threshold_;
  double speed_hysteresis_margin_;
  double robot_radius_;
  double person_radius_;
  double min_margin_radius_;
  double max_margin_radius_;
  double min_radius_;
  double max_radius_;
  bool no_people_flag_;
  bool use_velocity_obstacle_;
  int not_collision_count_;

  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr social_distance_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pure_velocity_obstacle_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr combined_speed_limit_pub_;
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
    social_distance_limit_topic_("/social_distance_limit"),
    pure_velocity_obstacle_limit_topic_("/pure_velocity_obstacle_limit"),
    combined_speed_limit_topic_("/combined_speed_limit"),
    odom_topic_("/odom"),
    plan_topic_("/plan"),
    event_topic_("/event"),
    set_social_distance_topic_("/set_social_distance"),
    get_social_distance_topic_("/get_social_distance"),
    map_frame_("map"),
    robot_base_frame_("base_footprint"),
    max_speed_(1.0),
    sd_min_speed_(0.1),
    vo_min_speed_(0.1),
    max_acc_(0.5),
    max_dec_(-1.0),
    delay_(0.5),
    social_distance_x_(2.0),
    social_distance_y_(1.0),
    social_distance_min_radius_(0.45),
    social_distance_min_angle_(-M_PI_2),
    social_distance_max_angle_(M_PI_2),
    no_people_topic_max_speed_(0.5),
    collision_time_horizon_(5.0),
    person_speed_threshold_(0.5),
    forward_approach_angle_threshold_(M_PI / 6.0),
    speed_hysteresis_margin_(1.0),
    robot_radius_(0.5),
    person_radius_(0.2),
    min_margin_radius_(0.0),
    max_margin_radius_(0.3),
    no_people_flag_(false),
    use_velocity_obstacle_(true),
    not_collision_count_(0)
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

    social_distance_limit_topic_ = declare_parameter("social_distance_limit_topic", social_distance_limit_topic_);
    social_distance_limit_pub_ =
      create_publisher<std_msgs::msg::Float32>(social_distance_limit_topic_, rclcpp::SystemDefaultsQoS().transient_local());

    use_velocity_obstacle_ = declare_parameter("use_velocity_obstacle", use_velocity_obstacle_);
    pure_velocity_obstacle_limit_topic_ = declare_parameter("pure_velocity_obstacle_limit_topic", pure_velocity_obstacle_limit_topic_);
    pure_velocity_obstacle_limit_pub_ = create_publisher<std_msgs::msg::Float32>(
      pure_velocity_obstacle_limit_topic_,
      rclcpp::SystemDefaultsQoS().transient_local());
    combined_speed_limit_topic_ = declare_parameter("combined_speed_limit_topic", combined_speed_limit_topic_);
    combined_speed_limit_pub_ = create_publisher<std_msgs::msg::Float32>(
      combined_speed_limit_topic_,
      rclcpp::SystemDefaultsQoS().transient_local());

    event_topic_ = declare_parameter("event_topic", event_topic_);
    event_pub_ = create_publisher<std_msgs::msg::String>(event_topic_, 100);

    max_speed_ = declare_parameter("max_speed_", max_speed_);
    sd_min_speed_ = declare_parameter("sosial_distance_min_speed_", sd_min_speed_);
    vo_min_speed_ = declare_parameter("velocity_obstacle_min_speed_", vo_min_speed_);
    max_acc_ = declare_parameter("max_acc_", max_acc_);
    max_dec_ = declare_parameter("max_dec_", max_dec_);
    social_distance_x_ = declare_parameter("social_distance_x", social_distance_x_);
    social_distance_y_ = declare_parameter("social_distance_y", social_distance_y_);
    social_distance_min_radius_ = declare_parameter("social_distance_min_radius", social_distance_min_radius_);
    social_distance_min_angle_ = declare_parameter("social_distance_min_angle", social_distance_min_angle_);
    social_distance_max_angle_ = declare_parameter("social_distance_max_angle", social_distance_max_angle_);
    no_people_topic_max_speed_ = declare_parameter("no_people_topic_max_speed", no_people_topic_max_speed_);
    collision_time_horizon_ = declare_parameter("collision_time_horizon", collision_time_horizon_);
    person_speed_threshold_ = declare_parameter("person_speed_threshold", person_speed_threshold_);
    forward_approach_angle_threshold_ = declare_parameter("forward_approach_angle_threshold", forward_approach_angle_threshold_);
    speed_hysteresis_margin_ = declare_parameter("speed_hysteresis_margin", speed_hysteresis_margin_);

    robot_radius_ = declare_parameter("robot_radius", robot_radius_);
    person_radius_ = declare_parameter("person_radius", person_radius_);
    min_margin_radius_ = declare_parameter("min_margin_radius", min_margin_radius_);
    max_margin_radius_ = declare_parameter("max_margin_radius", max_margin_radius_);
    min_radius_ = robot_radius_ + person_radius_ + min_margin_radius_;
    max_radius_ = robot_radius_ + person_radius_ + max_margin_radius_;

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
    social_distance_limit_pub_.reset();
    pure_velocity_obstacle_limit_pub_.reset();
    combined_speed_limit_pub_.reset();
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

      if (param.get_name() == "use_velocity_obstacle") {
        use_velocity_obstacle_ = param.as_bool();
      }
      if (param.get_name() == "max_speed_") {
        max_speed_ = param.as_double();
      }
      if (param.get_name() == "sd_min_speed_") {
        sd_min_speed_ = param.as_double();
      }
      if (param.get_name() == "vo_min_speed_") {
        vo_min_speed_ = param.as_double();
      }
      if (param.get_name() == "max_acc_") {
        max_acc_ = param.as_double();
      }
      if (param.get_name() == "max_dec_") {
        max_dec_ = param.as_double();
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
      if (param.get_name() == "collision_time_horizon") {
        collision_time_horizon_ = param.as_double();
      }
      if (param.get_name() == "person_speed_threshold") {
        person_speed_threshold_ = param.as_double();
      }
      if (param.get_name() == "forward_approach_angle_threshold") {
        forward_approach_angle_threshold_ = param.as_double();
      }
      if (param.get_name() == "speed_hysteresis_margin") {
        speed_hysteresis_margin_ = param.as_double();
      }
      if (param.get_name() == "robot_radius") {
        robot_radius_ = param.as_double();
        min_radius_ = robot_radius_ + person_radius_ + min_margin_radius_;
        max_radius_ = robot_radius_ + person_radius_ + max_margin_radius_;
      }
      if (param.get_name() == "person_radius") {
        person_radius_ = param.as_double();
        min_radius_ = robot_radius_ + person_radius_ + min_margin_radius_;
        max_radius_ = robot_radius_ + person_radius_ + max_margin_radius_;
      }
      if (param.get_name() == "min_margin_radius") {
        min_margin_radius_ = param.as_double();
        min_radius_ = robot_radius_ + person_radius_ + min_margin_radius_;
        max_radius_ = robot_radius_ + person_radius_ + max_margin_radius_;
      }
      if (param.get_name() == "max_margin_radius") {
        max_margin_radius_ = param.as_double();
        min_radius_ = robot_radius_ + person_radius_ + min_margin_radius_;
        max_radius_ = robot_radius_ + person_radius_ + max_margin_radius_;
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

    int logger_level = rcutils_logging_get_logger_level(this->get_logger().get_name());

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

    double social_distance_speed_limit = max_speed_;

    // Social Distance
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
        social_distance_speed_limit = std::min(social_distance_speed_limit, max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));
      } else {
        social_distance_speed_limit = std::min(social_distance_speed_limit, max_v(std::max(0.0, dist - social_distance_x_), max_acc_, delay_));
      }

      if (social_distance_speed_limit < sd_min_speed_) {
        social_distance_speed_limit = 0;
      }

      RCLCPP_INFO(
        get_logger(), "PeopleSpeedControl people_limit %s, %.2f %.2f (%.2f %.2f) - %.2f (%.2f)",
        person.name.c_str(), min_path_dist, dist, social_distance_x_, social_distance_y_, social_distance_speed_limit,
        max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));

      if (social_distance_speed_limit < max_speed_) {
        std_msgs::msg::String msg;

        if (fabs(social_distance_speed_limit) < 0.01) {
          msg.data = "navigation;event;people_speed_stopped";
        } else if (vx > 0.25 && social_distance_speed_limit < max_speed_ * 0.75) {
          msg.data = "navigation;event;people_speed_following";
        }

        if (!msg.data.empty()) {
          RCLCPP_INFO(get_logger(), "PeopleSpeedControl %s", msg.data.c_str());
          event_pub_->publish(msg);
        }
      }
    }

    // Velocity Obstacle
    std::priority_queue<VOData, std::vector<VOData>, CompareVOIntersectionMax> vo_data_queue;
    for (size_t i = 0; i < input->people.size(); ++i) {
      const auto & p_local = transformed_people[i].first;
      const auto & v_local = transformed_people[i].second;

      double rel_x = p_local.x();
      double rel_y = p_local.y();
      double pvx = v_local.x();
      double pvy = v_local.y();

      double rel_th = atan2(rel_y, rel_x);
      double dist = hypot(rel_x, rel_y);

      if (dist < max_radius_) {
        vo_data_queue.push({dist, rel_x, rel_y, pvx, pvy, 0.0, 0.0, 0.0, max_speed_});
        continue;
      }

      double s = asin(max_radius_ / dist);
      double theta_right = normalizedAngle(rel_th - s);
      double theta_left = normalizedAngle(rel_th + s);

      if (logger_level <= RCUTILS_LOG_SEVERITY_DEBUG) {
        addVOMarker(dist, pvx, pvy, theta_right, theta_left, map_to_robot_tf2);
      }

      // people walking below the speed threshold are not subject to velocity obstacle speed limits
      if (hypot(pvx, pvy) < person_speed_threshold_) {
        continue;
      }

      // compute the intersection points between the x-axis and the velocity obstacle cone
      auto vo_intersection = computeVOIntersection(pvx, pvy, rel_th, theta_right, theta_left);
      if (vo_intersection.empty()) {
        continue;
      }
      if (vo_intersection.size() != 2) {
        RCLCPP_ERROR(get_logger(), "Invalid number of intersection points in velocity obstacle calculation.");
        continue;
      }

      const double vo_intersection_min = std::min(vo_intersection[0], vo_intersection[1]);
      const double vo_intersection_max = std::max(vo_intersection[0], vo_intersection[1]);

      vo_data_queue.push({dist, rel_x, rel_y, pvx, pvy, theta_right, theta_left, vo_intersection_min, vo_intersection_max});
    }

    // velocity obstacle speed limit without considering social distance constraints
    double pure_velocity_obstacle_speed_limit = computeSafeSpeedLimit(max_speed_, vo_data_queue);

    // velocity obstacle speed limit constrained by social distance restrictions
    double combined_speed_limit = computeSafeSpeedLimit(social_distance_speed_limit, vo_data_queue);

    // final speed limit
    double people_speed_limit = use_velocity_obstacle_ ? combined_speed_limit : social_distance_speed_limit;

    publishLimits(social_distance_speed_limit, pure_velocity_obstacle_speed_limit, combined_speed_limit, people_speed_limit);

    if (logger_level <= RCUTILS_LOG_SEVERITY_DEBUG) {
      addSpeedLimitMarker(map_to_robot_tf2, people_speed_limit);
      CaBotSafety::commit(vis_pub_);
    }
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
    double visualized_dist = dist + max_radius_;

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

    CaBotSafety::add_circle(this->get_clock()->now(), 20, start_point, min_radius_, 0.05, 0.5, 0.5, 0, 1);
    CaBotSafety::add_circle(this->get_clock()->now(), 20, start_point, max_radius_, 0.05, 1, 0, 0, 1);

    char buff[100];
    snprintf(buff, sizeof(buff), "limit - %.2fm/s", people_speed_limit);

    CaBotSafety::add_text(this->get_clock()->now(), buff, start_point);
  }

  void publishLimits(
    const double social_distance_speed_limit, const double pure_velocity_obstacle_speed_limit, const double combined_speed_limit,
    const double people_speed_limit)
  {
    social_distance_limit_pub_->publish(std_msgs::msg::Float32().set__data(social_distance_speed_limit));
    pure_velocity_obstacle_limit_pub_->publish(std_msgs::msg::Float32().set__data(pure_velocity_obstacle_speed_limit));
    combined_speed_limit_pub_->publish(std_msgs::msg::Float32().set__data(combined_speed_limit));
    limit_pub_->publish(std_msgs::msg::Float32().set__data(people_speed_limit));
    // RCLCPP_INFO(get_logger(), "limit = %.2f", people_speed_limit);
  }

  std::vector<double> computeVOIntersection(
    const double vx_p, const double vy_p, const double rel_th, const double theta_right,
    const double theta_left)
  {
    constexpr double pseudo_infinity = std::numeric_limits<double>::max();

    // compute the parametric value (t) where the velocity obstacle cone intersects the x-axis
    auto computeParametricValue = [&](double theta) -> std::optional<double> {
        if (std::fabs(std::sin(theta)) < 1e-6) {
          return std::nullopt;
        }
        double t = -vy_p / std::sin(theta);
        if (t < 0.0) {
          return std::nullopt;
        }
        return t;
      };

    // determine the pseudo boundary based on the angle range
    auto getPseudoBoundary = [](double theta) -> double {
        return (-M_PI_2 <= theta && theta < M_PI_2) ? pseudo_infinity : -pseudo_infinity;
      };

    // compute the velocity at the intersection point
    auto computeVelocityAtIntersection = [&](double t, double theta) -> double {
        return vx_p + t * std::cos(theta);
      };

    std::optional<double> t_right = computeParametricValue(theta_right);
    std::optional<double> t_left = computeParametricValue(theta_left);

    std::vector<double> intersection;

    if (t_right && t_left) {
      double vo_right = computeVelocityAtIntersection(t_right.value(), theta_right);
      double vo_left = computeVelocityAtIntersection(t_left.value(), theta_left);

      if (std::fabs(vo_right - vo_left) < epsilon) {
        intersection.push_back(vo_right);
        if (-M_PI_2 <= rel_th && rel_th < M_PI_2) {
          intersection.push_back(pseudo_infinity);
        } else {
          intersection.push_back(-pseudo_infinity);
        }
      } else {
        intersection.push_back(vo_right);
        intersection.push_back(vo_left);
      }
    } else if (t_right) {
      intersection.push_back(computeVelocityAtIntersection(t_right.value(), theta_right));
      intersection.push_back(getPseudoBoundary(theta_left));
    } else if (t_left) {
      intersection.push_back(getPseudoBoundary(theta_right));
      intersection.push_back(computeVelocityAtIntersection(t_left.value(), theta_left));
    }

    return intersection;
  }

  bool willCollideWithinTime(double x_rel, double y_rel, double vx_rel, double vy_rel)
  {
    double a = vx_rel * vx_rel + vy_rel * vy_rel;
    double b = 2.0 * (x_rel * vx_rel + y_rel * vy_rel);
    double c = x_rel * x_rel + y_rel * y_rel - max_radius_ * max_radius_;

    double discriminant = b * b - 4.0 * a * c;

    if (discriminant < 0.0) {
      return false;
    }

    double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
    double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);

    if ((t1 >= 0.0 && t1 <= collision_time_horizon_) || (t2 >= 0.0 && t2 <= collision_time_horizon_)) {
      return true;
    }

    return false;
  }

  bool checkCollisionInRange(double vo_min, double vo_max, double rel_x, double rel_y, double pvx, double pvy)
  {
    constexpr double v_step = 0.1;  // step size for velocity iteration
    for (double rvx_curr = vo_min; rvx_curr <= vo_max; rvx_curr += v_step) {
      double rel_vx = pvx - rvx_curr;
      if (willCollideWithinTime(rel_x, rel_y, rel_vx, pvy)) {
        return true;
      }
    }
    return false;
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

  double computeSafeSpeedLimit(
    double speed_limit, const std::priority_queue<VOData, std::vector<VOData>, CompareVOIntersectionMax> & vo_data_queue)
  {
    const double rvx = last_odom_.twist.twist.linear.x;

    auto vo_data_queue_copy = vo_data_queue;
    while (!vo_data_queue_copy.empty()) {
      const auto & vo_data = vo_data_queue_copy.top();
      const auto & [dist, rel_x, rel_y, pvx, pvy, theta_right, theta_left, vo_intersection_min, vo_intersection_max] = vo_data;
      vo_data_queue_copy.pop();

      // People walking below the speed threshold are not subject to velocity obstacle speed limits
      if (dist < max_radius_) {
        double candidate_speed_limit = (rel_x > min_radius_) ? vo_min_speed_ : 0.0;
        speed_limit = std::min(speed_limit, candidate_speed_limit);
        continue;
      }

      if (vo_intersection_min > 0.0) {
        // Case 1: The velocity obstacle intersection range is positive
        if (vo_intersection_min < rvx && rvx < vo_intersection_max) {
          double max_rvx = (vo_intersection_max == std::numeric_limits<double>::max()) ? rvx + speed_hysteresis_margin_ : vo_intersection_max;
          if (checkCollisionInRange(vo_intersection_min, max_rvx, rel_x, rel_y, pvx, pvy)) {
            speed_limit = std::min(speed_limit, vo_intersection_min);
          }
        } else if (vo_intersection_min < speed_limit && speed_limit < vo_intersection_max) {
          if (checkCollisionInRange(vo_intersection_min, speed_limit, rel_x, rel_y, pvx, pvy)) {
            speed_limit = std::min(speed_limit, vo_intersection_min);
          }
        }
      } else if (vo_intersection_max >= 0.0) {
        // Case 2: The velocity obstacle intersection range includes zero or negative values
        double rel_vx = pvx - rvx;
        if (rel_x > 0.0 && willCollideWithinTime(rel_x, rel_y, rel_vx, pvy)) {
          if (vo_intersection_max >= std::min(speed_limit, rvx)) {
            double candidate_speed_limit = (rel_x > max_radius_) ? std::max(0.0, pvx + sqrt(-2.0 * max_dec_ * (rel_x - min_radius_))) : vo_min_speed_;
            speed_limit = std::min(speed_limit, candidate_speed_limit);
          }
        }
      }
    }

    return speed_limit;
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
