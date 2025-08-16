// Copyright (c) 2020, 2025  Carnegie Mellon University and Miraikan
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

// LiDAR speed control
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <limits>
#include <sstream>

#include <cabot/util.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace CaBotSafety
{
class LiDARSpeedControlNode : public rclcpp::Node
{
public:
  std::string laser_topic_;
  std::string footprint_topic_;
  std::string vis_topic_;
  std::string limit_topic_;

  std::string map_frame_;
  std::string robot_base_frame_;

  bool check_blind_space_;
  bool check_front_obstacle_;
  bool speed_limit_in_the_last_frame_;

  double max_speed_;
  double min_speed_;
  double max_acc_;
  double delay_;
  double urgent_max_acc_;
  // double min_distance_;
  double center_min_distance_;
  double side_min_distance_;
  double front_region_width_;
  double blind_spot_min_range_;

  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::LaserScan last_scan_;
  double last_min_x_distance_;
  double distance_noise_threshold_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
  tf2_ros::TransformListener * tfListener;
  tf2_ros::Buffer * tfBuffer;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  // message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
  // tf::MessageFilter<sensor_msgs::msg::LaserScan> laser_notifier_;

  explicit LiDARSpeedControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("lidar_speed_control_node", options),
    laser_topic_("/scan"),
    footprint_topic_("/global_costmap/footprint"),
    vis_topic_("visualize"),
    limit_topic_("lidar_limit"),
    map_frame_("map"),
    robot_base_frame_("base_footprint"),
    check_blind_space_(true),
    check_front_obstacle_(true),
    speed_limit_in_the_last_frame_(false),
    max_speed_(1.0),
    min_speed_(0.0),
    max_acc_(0.6),
    delay_(0.2),
    urgent_max_acc_(1.2),
    center_min_distance_(1.25),
    side_min_distance_(0.50),
    front_region_width_(0.50),
    blind_spot_min_range_(0.25),
    last_min_x_distance_(100),
    distance_noise_threshold_(0.03)
  {
    RCLCPP_INFO(get_logger(), "LiDARSpeedControlNodeClass Constructor");
    tfBuffer = new tf2_ros::Buffer(get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer, this);
    onInit();
  }

  ~LiDARSpeedControlNode()
  {
    RCLCPP_INFO(get_logger(), "LiDARSpeedControlNodeClass Destructor");
    scan_sub_.reset();
    footprint_sub_.reset();
    vis_pub_.reset();
    limit_pub_.reset();
    callback_handler_.reset();
    delete tfListener;
    delete tfBuffer;
  }

private:
  rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
    for (auto && param : params) {
      if (!this->has_parameter(param.get_name())) {
        continue;
      }
      RCLCPP_DEBUG(get_logger(), "change param %s", param.get_name().c_str());

      if (param.get_name() == "check_front_obstacle") {
        check_front_obstacle_ = param.as_bool();
      }
      if (param.get_name() == "check_blind_space") {
        check_blind_space_ = param.as_bool();
      }
      if (param.get_name() == "center_min_distance") {
        center_min_distance_ = param.as_double();
      }
      if (param.get_name() == "side_min_distance") {
        side_min_distance_ = param.as_double();
      }
    }
    results->successful = true;
    return *results;
  }

  void onInit()
  {
    RCLCPP_INFO(get_logger(), "LiDAR speed control - %s", __FUNCTION__);

    laser_topic_ = declare_parameter("laser_topic", laser_topic_);
    scan_sub_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LiDARSpeedControlNode::laserCallback, this, std::placeholders::_1));

    footprint_sub_ =
      create_subscription<geometry_msgs::msg::Polygon>(
      footprint_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      std::bind(&LiDARSpeedControlNode::footprintCallback, this, std::placeholders::_1));

    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&LiDARSpeedControlNode::odomCallback, this, std::placeholders::_1));

    vis_topic_ = declare_parameter("visualize_topic", vis_topic_);
    vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(vis_topic_, 100);

    limit_topic_ = declare_parameter("limit_topic", limit_topic_);
    limit_pub_ = create_publisher<std_msgs::msg::Float32>(limit_topic_, rclcpp::SystemDefaultsQoS().transient_local());

    check_blind_space_ = declare_parameter("check_blind_space", check_blind_space_);
    check_front_obstacle_ = declare_parameter("check_front_obstacle", check_front_obstacle_);
    max_speed_ = declare_parameter("max_speed", max_speed_);
    min_speed_ = declare_parameter("min_speed", min_speed_);
    max_acc_ = declare_parameter("max_acc", max_acc_);
    center_min_distance_ = declare_parameter("center_min_distance", center_min_distance_);
    side_min_distance_ = declare_parameter("side_min_distance", side_min_distance_);
    blind_spot_min_range_ = declare_parameter("blind_spot_min_range", blind_spot_min_range_);
    distance_noise_threshold_ = declare_parameter("distance_noise_threshold", distance_noise_threshold_);

    RCLCPP_INFO(
      get_logger(), "LiDARSpeedControl with check_blind_space=%s, check_front_obstacle=%s, max_speed=%.2f",
      check_blind_space_ ? "true" : "false", check_front_obstacle_ ? "true" : "false", max_speed_);

    callback_handler_ =
      this->add_on_set_parameters_callback(std::bind(&LiDARSpeedControlNode::param_set_callback, this, std::placeholders::_1));
  }

  struct BlindSpot : CaBotSafety::Point
  {
    rclcpp::Time last_confirmed;
    int count;
  };

  std::vector<BlindSpot> blind_spots;

  double BLIND_SPOT_MIN_SIZE = 1.0;
  double BLIND_SPOT_MAX_ANGLE = -0.1;
  double BLIND_SPOT_MAX_DISTANCE = 5.0;
  double BLIND_SPOT_MIN_COUNT = 5;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr input)
  {
    double frame_period = 0.1;
    if (last_scan_.header.stamp.sec != 0) {
      rclcpp::Time stamp(input->header.stamp);
      rclcpp::Time last_stamp(last_scan_.header.stamp);
      frame_period = std::max(0.1, (stamp - last_stamp).seconds());
    }
    last_scan_ = *input;
    double inf = std::numeric_limits<double>::infinity();
    double speed_limit = max_speed_;

    tf2::Transform robot_pose(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));

    // transform map -> base_footprint
    geometry_msgs::msg::TransformStamped map_to_robot_msg;
    try {
      map_to_robot_msg = tfBuffer->lookupTransform(
        "map", "base_footprint",
        rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return;
    }
    tf2::Stamped<tf2::Transform> map_to_robot_tf2;
    tf2::fromMsg(map_to_robot_msg, map_to_robot_tf2);

    robot_pose *= map_to_robot_tf2;

    // RCLCPP_INFO(get_logger(), "%.2f,%.2f,%.2f", robot_pose.getOrigin().x(),
    //              robot_pose.getOrigin().y(), tf2::getYaw(robot_pose.getRotation()));

    // clear(vis_pub_);
    CaBotSafety::add_point(get_clock()->now(), robot_pose, 0.2, 0, 1, 0, 1);

    geometry_msgs::msg::TransformStamped robot_to_lidar_msg;
    tf2::Stamped<tf2::Transform> robot_to_lidar_tf2;
    try {
      robot_to_lidar_msg = tfBuffer->lookupTransform(
        robot_base_frame_, input->header.frame_id,
        input->header.stamp, rclcpp::Duration(std::chrono::duration<double>(1.0)));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return;
    }
    tf2::fromMsg(robot_to_lidar_msg, robot_to_lidar_tf2);

    CaBotSafety::Line robot(robot_pose);
    RCLCPP_DEBUG(get_logger(), "%.2f,%.2f,%.2f,%.2f", robot.s.x, robot.s.y, robot.e.x, robot.e.y);

    if (check_front_obstacle_) {  // Check front obstacle (mainly for avoinding from speeding up near the wall
                                  // get some points in front of the robot
      auto max_v = [](double D, double A, double d)
        {
          D = std::max(0.0, D);
          return (-2 * A * d + sqrt(4 * A * A * d * d + 8 * A * D)) / 2;
        };
      auto max_D = [](double v, double A, double d)
        {
          return ((v + A * d) * (v + A * d) - A * A * d * d) / 2 / A;
        };

      const double rect_height = max_D(max_speed_, max_acc_, delay_);

      Point region_ur_point(side_min_distance_ + rect_height, front_region_width_ / 2.0);
      Point region_uc_point(center_min_distance_ + rect_height, 0.0);
      Point region_ul_point(side_min_distance_ + rect_height, -front_region_width_ / 2.0);
      Point region_lr_point(0.0, front_region_width_ / 2.0);
      Point region_ll_point(0.0, -front_region_width_ / 2.0);

      region_ur_point.transform(map_to_robot_tf2);
      region_uc_point.transform(map_to_robot_tf2);
      region_ul_point.transform(map_to_robot_tf2);
      region_lr_point.transform(map_to_robot_tf2);
      region_ll_point.transform(map_to_robot_tf2);

      std::array<Point, 3> front_check_region_0 = {region_ur_point, region_uc_point, region_ul_point};
      std::array<Point, 3> front_check_region_1 = {region_ur_point, region_ul_point, region_ll_point};
      std::array<Point, 3> front_check_region_2 = {region_ll_point, region_lr_point, region_ur_point};

      CaBotSafety::add_triangle(get_clock()->now(), front_check_region_0, 1.0, 0.0, 0.0, 0.0, 0.6);
      CaBotSafety::add_triangle(get_clock()->now(), front_check_region_1, 1.0, 0.0, 0.0, 0.0, 0.6);
      CaBotSafety::add_triangle(get_clock()->now(), front_check_region_2, 1.0, 0.0, 0.0, 0.0, 0.6);

      double min_limit = 100;
      Point min_point;
      for (uint64_t i = 0; i < input->ranges.size(); i++) {
        double angle = input->angle_min + input->angle_increment * i;
        double range = input->ranges[i];
        if (range == inf) {
          range = input->range_max;
        }

        double x = range * cos(angle);
        double y = range * sin(angle);

        Point point_in_robot_frame(x, y);
        Point point_in_map_frame(x, y);
        point_in_robot_frame.transform(robot_to_lidar_tf2);
        point_in_map_frame.transform(map_to_robot_tf2 * robot_to_lidar_tf2);

        // Check if the point is in the front region
        if (point_in_robot_frame.x > 0.0 && fabs(point_in_robot_frame.y) < front_region_width_ / 2.0) {
          double r = fabs(point_in_robot_frame.y) / (front_region_width_ / 2.0);
          double min_distance = r * side_min_distance_ + (1 - r) * center_min_distance_;
          double local_speed_limit = max_v(point_in_robot_frame.x - min_distance, max_acc_, delay_);
          if (local_speed_limit < max_speed_) {
            CaBotSafety::add_point(get_clock()->now(), point_in_map_frame, 0.2, 1, 1, 0, 1);
          }

          if (local_speed_limit < min_limit) {
            min_point = point_in_robot_frame;
            min_limit = local_speed_limit;
          }
        }
      }
      // calculate the speed
      double r = fabs(min_point.y) / (front_region_width_ / 2.0);
      double min_distance = r * side_min_distance_ + (1 - r) * center_min_distance_;
      double temp_limit = std::min(max_speed_, std::max(min_speed_, max_v(min_point.x - min_distance, max_acc_, delay_)));

      RCLCPP_INFO(
        get_logger(), "limit by front fobstacle (%.2f), temp_limit=%.2f, min_distance_=%.2f, r=%.2f"
        "min_point=(%.2f, %.2f), speed_limit_in_the_last_frame_=%s",
        speed_limit, temp_limit, min_distance, r, min_point.x, min_point.y, speed_limit_in_the_last_frame_ ? "true" : "false");
      speed_limit = temp_limit;

      /*
      if (temp_limit < max_speed_) {

        if (speed_limit_in_the_last_frame_) {
          if (last_min_x_distance_ + distance_noise_threshold_ < min_point.x) {
            // wait for the next frame, because object may be relatively moving away
            // speed_limit = temp_limit;
            speed_limit = std::min(max_speed_, last_odom_.twist.twist.linear.x + max_acc_ * frame_period);
          } else {
            // critical
            speed_limit = std::max(temp_limit, last_odom_.twist.twist.linear.x - urgent_max_acc_ * frame_period);
          }
        } else {
          // noop, wait for the next frame
        }
        speed_limit_in_the_last_frame_ = true;
        last_min_x_distance_ = min_point.x;
      } else {
        speed_limit_in_the_last_frame_ = false;
        last_min_x_distance_ = 100;
      }
      */
    }

    if (check_blind_space_) {  // Check blind space
      Point prev;
      for (uint64_t i = 0; i < input->ranges.size(); i++) {
        double angle = input->angle_min + input->angle_increment * i;
        double range = input->ranges[i];
        if (range < blind_spot_min_range_) {
          continue;
        }
        if (range == inf) {
          range = input->range_max;
        }

        Point curr(range * cos(angle), range * sin(angle));
        curr.transform(map_to_robot_tf2 * robot_to_lidar_tf2);

        if (prev.x != 0 && prev.y != 0) {
          CaBotSafety::Line line(prev, curr);
          double len = line.length();
          double cross = robot.cross(line) / robot.length() / line.length();
          double ds = robot.s.distanceTo(prev);
          double de = robot.e.distanceTo(curr);
          Point closest = (ds < de) ? prev : curr;
          double dist = std::min(ds, de);

          if (len > BLIND_SPOT_MIN_SIZE &&
            cross < BLIND_SPOT_MAX_ANGLE &&
            dist < BLIND_SPOT_MAX_DISTANCE)
          {
            // found a spot

            uint64_t j = 0;
            for (; j < blind_spots.size(); j++) {
              double d = blind_spots[j].distanceTo(closest);
              if (d < 0.3) { // hulostic
                             // same spot and update it
                blind_spots[j].count += 1;
                blind_spots[j].x = closest.x;
                blind_spots[j].y = closest.y;
                blind_spots[j].last_confirmed = get_clock()->now();
                break;
              }
            }

            if (j == blind_spots.size()) {
              // different spot in the list
              BlindSpot bs;
              bs.x = closest.x;
              bs.y = closest.y;
              bs.count = 1;
              bs.last_confirmed = get_clock()->now();
              blind_spots.push_back(bs);
            }
          }
        }
        prev = curr;
      }

      // do something after filter

      for (uint64_t i = 0; i < blind_spots.size(); i++) {
        BlindSpot bs = blind_spots[i];
        if (bs.count < BLIND_SPOT_MIN_COUNT) {
          continue;
        }

        CaBotSafety::add_point(get_clock()->now(), bs, 0.2, 1, 0, 0, 1);

        CaBotSafety::Point closestToRobot = robot.closestPoint(bs);
        CaBotSafety::Line l1(bs, closestToRobot);
        CaBotSafety::Line l2(robot.s, closestToRobot);

        // if robot pass the spot, remove it
        if (robot.dot(l2) < 0) {
          blind_spots[i].count = 0;
          continue;
        }

        CaBotSafety::add_line(get_clock()->now(), l1, 0.05, 1, 0, 0, 1);
        CaBotSafety::add_arrow(get_clock()->now(), l2, 0.1, 1, 0, 0, 1);

        // calculate speed limit
        // v = -a*t0 + sqrt(a^2*t0^2+2Da)
        double delay = 0.1;  // sec (t0)
        double critical_distance = l1.length() + l2.length();  // = 2D
        double limit = -max_acc_ * delay + sqrt(
          max_acc_ * max_acc_ * delay * delay +
          critical_distance * max_acc_);
        // update speed limit
        if (limit < speed_limit) {
          speed_limit = limit;
          if (speed_limit < max_speed_) {
            RCLCPP_INFO(get_logger(), "limit by blind spot (%.2f), critical_distance=%.2f", speed_limit, critical_distance);
          }
        }
      }

      // remove unused spot
      for (int i = blind_spots.size() - 1; i >= 0; i--) {
        BlindSpot bs = blind_spots[i];
        rclcpp::Duration timediff = (get_clock()->now() - bs.last_confirmed);
        if (timediff > rclcpp::Duration(std::chrono::duration<double>(0.1 * std::max(5, bs.count)))) {
          blind_spots.erase(blind_spots.begin() + i);
        }
      }
    }

    // Publishing the speed limit
    std_msgs::msg::Float32 msg;
    msg.data = speed_limit;
    RCLCPP_INFO(get_logger(), "limit = %.2f", speed_limit);

    // Publishing the visualization
    char buff[100];
    snprintf(buff, sizeof(buff), "limit - %.2fm/s", speed_limit);
    std::string buffAsStdStr = buff;
    CaBotSafety::add_text(get_clock()->now(), buff, robot.s);
    limit_pub_->publish(msg);
    CaBotSafety::commit(vis_pub_);
  }

  void footprintCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
  {
    if (msg->points.empty()) {
      return;
    }
    const auto & p_x = msg->points[0].x;
    const auto & p_y = msg->points[0].y;
    front_region_width_ = 2.0 * std::hypot(p_x, p_y);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
  }
};  // class LiDARSpeedControlNode

}  // namespace CaBotSafety

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::LiDARSpeedControlNode)
