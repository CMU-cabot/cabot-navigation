// Copyright (c) 2024  Carnegie Mellon University
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

#ifndef PEDESTRIAN_PLUGIN__PEDESTRIAN_PLUGIN_MANAGER_HPP_
#define PEDESTRIAN_PLUGIN__PEDESTRIAN_PLUGIN_MANAGER_HPP_
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <map>
#include <memory>
#include <string>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pedestrian_plugin_msgs/msg/collision.hpp>
#include <pedestrian_plugin_msgs/msg/metric.hpp>
#include <pedestrian_plugin_msgs/msg/agent.hpp>
#include <pedestrian_plugin_msgs/msg/agents.hpp>
#include <pedestrian_plugin_msgs/srv/plugin_update.hpp>

PyObject * PyInit_ros(void);

namespace gazebo
{

class PedestrianPlugin;

class ParamValue
{
public:
  using ValueType = std::variant<std::monostate, int, double, std::string>;
  static ParamValue null();
  static ParamValue create(const std::string & type, const std::string & value);
  bool isNull() const;
  template<typename T>
  T get() const
  {
    if constexpr (std::is_same_v<T, ValueType>) {
      return value_;
    } else {
      if (std::holds_alternative<T>(value_)) {
        return std::get<T>(value_);
      } else {
        throw std::bad_variant_access();
      }
    }
  }
  PyObject * python_object();

private:
  ParamValue();
  explicit ParamValue(ValueType value);
  ValueType value_;
};

class PedestrianPluginParams
{
public:
  void addParam(std::string name, std::string type, std::string value);
  ParamValue getParam(std::string name);

  using iterator = std::map<std::string, ParamValue>::const_iterator;
  iterator begin() const
  {
    return paramMap_.begin();
  }
  iterator end() const
  {
    return paramMap_.end();
  }

private:
  std::map<std::string, ParamValue> paramMap_;
};

class PedestrianPluginManager
{
public:
  static PedestrianPluginManager & getInstance()
  {
    static PedestrianPluginManager instance;   // Guaranteed to be destroyed and instantiated on first use
    return instance;
  }
  PedestrianPluginManager();
  ~PedestrianPluginManager();
  size_t addPlugin(std::string name, PedestrianPlugin * plugin);
  void removePlugin(std::string name);
  void publishPeopleIfReady();
  void updateRobotPose(geometry_msgs::msg::Pose robot_pose);
  void updatePersonMessage(std::string name, people_msgs::msg::Person person);
  void updateStamp(builtin_interfaces::msg::Time stamp);
  void updateRobotAgent(pedestrian_plugin_msgs::msg::Agent robotAgent);
  void updateHumanAgent(std::string name, pedestrian_plugin_msgs::msg::Agent humanAgent);
  rclcpp::Logger get_logger();
  void process_collision(std::string actor_name, double distance);
  void process_metric(std::string name, double value);

  std::recursive_mutex mtx;

private:
  void handle_plugin_update(
    const std::shared_ptr<pedestrian_plugin_msgs::srv::PluginUpdate::Request> request,
    std::shared_ptr<pedestrian_plugin_msgs::srv::PluginUpdate::Response> response);

  gazebo_ros::Node::SharedPtr node_;
  geometry_msgs::msg::Pose::SharedPtr robot_pose_;
  std::map<std::string, PedestrianPlugin *> pluginMap_;
  std::map<std::string, people_msgs::msg::Person> peopleMap_;
  std::map<std::string, bool> peopleReadyMap_;
  builtin_interfaces::msg::Time::SharedPtr stamp_;
  pedestrian_plugin_msgs::msg::Agent::SharedPtr robotAgent_;
  std::map<std::string, pedestrian_plugin_msgs::msg::Agent> humanAgentsMap_;
  rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pub_;
  rclcpp::Publisher<pedestrian_plugin_msgs::msg::Collision>::SharedPtr collision_pub_;
  rclcpp::Publisher<pedestrian_plugin_msgs::msg::Metric>::SharedPtr metric_pub_;
  rclcpp::Publisher<pedestrian_plugin_msgs::msg::Agent>::SharedPtr robot_pub_;
  rclcpp::Publisher<pedestrian_plugin_msgs::msg::Agents>::SharedPtr human_pub_;
  rclcpp::Service<pedestrian_plugin_msgs::srv::PluginUpdate>::SharedPtr service_;
};

}  // namespace gazebo

#endif  // PEDESTRIAN_PLUGIN__PEDESTRIAN_PLUGIN_MANAGER_HPP_
