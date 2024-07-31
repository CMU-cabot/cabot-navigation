/*******************************************************************************
 * Copyright (c) 2024  Carnegie Mellon University and Miraikan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

#include "pedestrian_plugin/python_utils.hpp"
#include "pedestrian_plugin/pedestrian_plugin.hpp"
#include "pedestrian_plugin/pedestrian_plugin_manager.hpp"

using namespace gazebo;  // NOLINT

// exporting ros python module
PyObject * ros_info(PyObject * self, PyObject * args)
{
  const char * message;
  if (PyArg_ParseTuple(args, "s", &message)) {
    RCLCPP_INFO(PedestrianPluginManager::getInstance().get_logger(), message);
  }
  Py_RETURN_NONE;
}

PyObject * ros_collision(PyObject * self, PyObject * args)
{
  PyObject * unicode_obj;
  double distance;
  if (!PyArg_ParseTuple(args, "Ud", &unicode_obj, &distance)) {
    RCLCPP_INFO(PedestrianPluginManager::getInstance().get_logger(), "error in parsing tuple");
    return NULL;
  }
  std::string name = PythonUtils::PyUnicodeObject_ToStdString(unicode_obj);
  PedestrianPluginManager::getInstance().process_collision(name, distance);
  Py_RETURN_NONE;
}

PyObject * ros_metric(PyObject * self, PyObject * args)
{
  PyObject * unicode_obj;
  double value;
  if (!PyArg_ParseTuple(args, "Ud", &unicode_obj, &value)) {
    RCLCPP_INFO(PedestrianPluginManager::getInstance().get_logger(), "error in parsing tuple");
    return NULL;
  }
  std::string name = PythonUtils::PyUnicodeObject_ToStdString(unicode_obj);
  PedestrianPluginManager::getInstance().process_metric(name, value);
  Py_RETURN_NONE;
}

PyMethodDef RosMethods[] = {{"info", ros_info, METH_VARARGS, "call RCLCPP_INFO"},
  {"collision", ros_collision, METH_VARARGS, "publish collision message"},
  {"metric", ros_metric, METH_VARARGS, "publish metric message"},
  {NULL, NULL, 0, NULL}};

PyModuleDef RosModule = {PyModuleDef_HEAD_INIT, "ros", NULL, -1, RosMethods, NULL, NULL, NULL, NULL};

PyObject * PyInit_ros(void) {return PyModule_Create(&RosModule);}

ParamValue::ParamValue()
: value_(std::monostate{}) {}

ParamValue::ParamValue(ValueType value)
: value_(std::move(value)) {}

bool ParamValue::isNull() const {return std::holds_alternative<std::monostate>(value_);}

ParamValue ParamValue::null() {return ParamValue();}

ParamValue ParamValue::create(const std::string & type, const std::string & value)
{
  if (type == "int") {
    return ParamValue(std::stoi(value));
  } else if (type == "float") {
    return ParamValue(std::stod(value));
  } else if (type == "str") {
    return ParamValue(value);
  } else {
    return ParamValue();
  }
}

PyObject * ParamValue::python_object()
{
  const auto & val = get<ParamValue::ValueType>();

  if (std::holds_alternative<int>(val)) {
    return PyLong_FromLong(std::get<int>(val));
  } else if (std::holds_alternative<double>(val)) {
    return PyFloat_FromDouble(std::get<double>(val));
  } else if (std::holds_alternative<std::string>(val)) {
    return PyUnicode_FromString(std::get<std::string>(val).c_str());
  } else {
    // If it's std::monostate or an unsupported type, return Py_None
    Py_INCREF(Py_None);
    return Py_None;
  }
}

void PedestrianPluginParams::addParam(std::string name, std::string type, std::string value)
{
  paramMap_.insert({name, ParamValue::create(type, value)});
}

ParamValue PedestrianPluginParams::getParam(std::string name)
{
  auto it = paramMap_.find(name);
  if (it == paramMap_.end()) {
    return it->second;
  }
  return ParamValue::null();
}

PedestrianPluginManager::PedestrianPluginManager()
: node_(gazebo_ros::Node::Get())
{
  people_pub_ = node_->create_publisher<people_msgs::msg::People>("/people", 10);
  collision_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Collision>("/collision_person", 10);
  metric_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Metric>("/metric", 10);
  robot_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Agent>("/robot_states", 10);
  human_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Agents>("/human_states", 10);
  service_ = node_->create_service<pedestrian_plugin_msgs::srv::PluginUpdate>(
    "/pedestrian_plugin_update",
    std::bind(&PedestrianPluginManager::handle_plugin_update, this, std::placeholders::_1, std::placeholders::_2));
}

PedestrianPluginManager::~PedestrianPluginManager() {}

size_t PedestrianPluginManager::addPlugin(std::string name, PedestrianPlugin * plugin)
{
  pluginMap_.insert({name, plugin});
  return pluginMap_.size();
}

void PedestrianPluginManager::removePlugin(std::string name) {pluginMap_.erase(name);}

void PedestrianPluginManager::publishPeopleIfReady()
{
  if (peopleReadyMap_.size() == pluginMap_.size()) {
    people_msgs::msg::People msg;
    for (auto it : peopleMap_) {
      msg.people.push_back(it.second);
    }
    msg.header.stamp = *stamp_;
    msg.header.frame_id = "map_global";
    people_pub_->publish(msg);

    // publish robot and human messages
    robotAgent_->header.stamp = *stamp_;
    robotAgent_->header.frame_id = "map_global";
    robot_pub_->publish(*robotAgent_);
    pedestrian_plugin_msgs::msg::Agents humanAgentsMsg;
    for (auto it : humanAgentsMap_) {
      humanAgentsMsg.agents.push_back(it.second);
    }
    humanAgentsMsg.header.stamp = *stamp_;
    humanAgentsMsg.header.frame_id = "map_global";
    human_pub_->publish(humanAgentsMsg);

    peopleReadyMap_.clear();
  }
}

void PedestrianPluginManager::updateRobotPose(geometry_msgs::msg::Pose robot_pose)
{
  if (robot_pose_ == nullptr) {
    robot_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
  }
  robot_pose_->position = robot_pose.position;
  robot_pose_->orientation = robot_pose.orientation;
}

void PedestrianPluginManager::updatePersonMessage(std::string name, people_msgs::msg::Person person)
{
  peopleMap_.insert_or_assign(name, person);
  peopleReadyMap_.insert({name, true});
}

void PedestrianPluginManager::updateStamp(builtin_interfaces::msg::Time stamp)
{
  if (stamp_ == nullptr) {
    stamp_ = std::make_shared<builtin_interfaces::msg::Time>();
  }
  *stamp_ = stamp;
}

void PedestrianPluginManager::updateRobotAgent(pedestrian_plugin_msgs::msg::Agent robotAgent)
{
  if (robotAgent_ == nullptr) {
    robotAgent_ = std::make_shared<pedestrian_plugin_msgs::msg::Agent>();
  }
  *robotAgent_ = robotAgent;
}

void PedestrianPluginManager::updateHumanAgent(std::string name, pedestrian_plugin_msgs::msg::Agent humanAgent)
{
  humanAgentsMap_.insert_or_assign(name, humanAgent);
}

rclcpp::Logger PedestrianPluginManager::get_logger() {return node_->get_logger();}

void PedestrianPluginManager::process_collision(std::string actor_name, double distance)
{
  if (robot_pose_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "no robot_pose");
    return;
  }
  auto it = peopleMap_.find(actor_name);
  if (it == peopleMap_.end()) {
    RCLCPP_ERROR(get_logger(), "cannot find person msg for %s", actor_name.c_str());
    return;
  }
  people_msgs::msg::Person collided_person = it->second;
  pedestrian_plugin_msgs::msg::Collision msg;
  msg.robot_pose = *robot_pose_;
  msg.collided_person = collided_person;
  msg.distance = distance;
  collision_pub_->publish(msg);
}

void PedestrianPluginManager::process_metric(std::string name, double value)
{
  pedestrian_plugin_msgs::msg::Metric msg;
  msg.name = name;
  msg.value = value;
  metric_pub_->publish(msg);
}

// Private methods

void PedestrianPluginManager::handle_plugin_update(
  const std::shared_ptr<pedestrian_plugin_msgs::srv::PluginUpdate::Request> request,
  std::shared_ptr<pedestrian_plugin_msgs::srv::PluginUpdate::Response> response)
{
  RCLCPP_INFO(get_logger(), "pedestrian plugin update service is called");
  for (const pedestrian_plugin_msgs::msg::Plugin & update : request->plugins) {
    std::string name = update.name;
    auto it = pluginMap_.find(name);
    if (it == pluginMap_.end()) {
      RCLCPP_ERROR(get_logger(), "could not find actor(%s)'s plugin", name.c_str());
      continue;
    }
    PedestrianPlugin * plugin = it->second;

    std::string module = update.module;
    PedestrianPluginParams update_params;
    update_params.addParam("module", "str", module);
    for (const pedestrian_plugin_msgs::msg::PluginParam & param : update.params) {
      update_params.addParam(param.name, param.type, param.value);
    }

    RCLCPP_INFO(get_logger(), "update parameters %s", name.c_str());
    plugin->update_parameters(update_params);
  }
  for (const auto & it : pluginMap_) {
    response->plugin_names.push_back(it.first);
  }
  response->message = "Plugin Updated";
}
