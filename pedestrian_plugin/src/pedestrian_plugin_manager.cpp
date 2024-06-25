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

#include "pedestrian_plugin/geometry_utils.hpp"
#include "pedestrian_plugin/math_utils.hpp"
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
: node_(gazebo_ros::Node::Get()),
  tf_listener_(nullptr),
  tf_buffer_(nullptr)
{
  if (!node_->has_parameter("pedestrian_plugin.min_range")) {
    node_->declare_parameter<double>("pedestrian_plugin.min_range", 0.0);
  }
  if (!node_->has_parameter("pedestrian_plugin.max_range")) {
    node_->declare_parameter<double>(
      "pedestrian_plugin.max_range", 20.0);
  }
  if (!node_->has_parameter("pedestrian_plugin.min_angle")) {
    node_->declare_parameter<double>(
      "pedestrian_plugin.min_angle", -M_PI);
  }
  if (!node_->has_parameter("pedestrian_plugin.max_angle")) {
    node_->declare_parameter<double>(
      "pedestrian_plugin.max_angle", M_PI);
  }
  if (!node_->has_parameter("pedestrian_plugin.occlusion_radius")) {
    node_->declare_parameter<double>("pedestrian_plugin.occlusion_radius", 0.25);
  }
  if (!node_->has_parameter("pedestrian_plugin.divider_distance_m")) {
    node_->declare_parameter<double>("pedestrian_plugin.divider_distance_m", 0.05);
  }
  if (!node_->has_parameter("pedestrian_plugin.divider_angle_deg")) {
    node_->declare_parameter<double>("pedestrian_plugin.divider_angle_deg", 1.0);
  }

  min_range_ = node_->get_parameter("pedestrian_plugin.min_range").as_double();
  max_range_ = node_->get_parameter("pedestrian_plugin.max_range").as_double();
  min_angle_ = node_->get_parameter("pedestrian_plugin.min_angle").as_double();
  max_angle_ = node_->get_parameter("pedestrian_plugin.max_angle").as_double();
  occlusion_radius_ = node_->get_parameter("pedestrian_plugin.occlusion_radius").as_double();
  divider_distance_m_ = node_->get_parameter("pedestrian_plugin.divider_distance_m").as_double();
  divider_angle_deg_ = node_->get_parameter("pedestrian_plugin.divider_angle_deg").as_double();

  setOffsetTable();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  people_pub_ = node_->create_publisher<people_msgs::msg::People>("/people", 10);
  collision_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Collision>("/collision", 10);
  metric_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Metric>("/metric", 10);
  robot_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Agent>("/robot_states", 10);
  human_pub_ = node_->create_publisher<pedestrian_plugin_msgs::msg::Agents>("/human_states", 10);
  service_ = node_->create_service<pedestrian_plugin_msgs::srv::PluginUpdate>(
    "/pedestrian_plugin_update",
    std::bind(&PedestrianPluginManager::handle_plugin_update, this, std::placeholders::_1, std::placeholders::_2));
  dyn_params_handler_ = node_->add_on_set_parameters_callback(
    std::bind(&PedestrianPluginManager::dynamicParametersCallback, this, std::placeholders::_1));
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
  if (tf_buffer_ && tf_listener_) {
    std::string base_control_shift_link = "base_control_shift";
    std::string lidar_link = "lidar_link";
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
        base_control_shift_link, lidar_link, tf2::TimePointZero);

      const geometry_msgs::msg::Transform & transform = transform_stamped.transform;
      tf2::fromMsg(transform, control_to_lidar_tf_);

      tf_listener_.reset();
      tf_buffer_.reset();
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        node_->get_logger(), "Could not transform %s to %s: %s",
        base_control_shift_link.c_str(), lidar_link.c_str(), ex.what());
      return;
    }
  }

  if (!initialized_base_control_shift_link_) {return;}

  if (peopleReadyMap_.size() == pluginMap_.size()) {
    people_msgs::msg::People msg;
    tf2::Transform base_to_lidar_tf;

    base_to_lidar_tf = base_to_control_tf_ * control_to_lidar_tf_;
    const auto robot_point =
      robotAgent_->position.position + getPointFromTransform(base_to_lidar_tf);
    const auto robot_yaw =
      math::normalizeRad(robotAgent_->yaw + getYawFromTransform(base_to_lidar_tf));

    auto visible_people = getNonOccludedPeople(robot_point, peopleMap_);
    auto filtered_people = filterByDistanceAndAngle(robot_yaw, robot_point, visible_people);

    msg.header.stamp = *stamp_;
    msg.header.frame_id = "map_global";
    msg.people = filtered_people;
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

  if (initialized_base_control_shift_link_) {return;}
  if (!initializedBaseControlShiftLink()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize base_control_shift link");
  }
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

void PedestrianPluginManager::setOffsetTable()
{
  int dist_divisions = std::round(max_range_ / divider_distance_m_) + 1;
  offset_table_.clear();
  offset_table_.shrink_to_fit();
  offset_table_.resize(dist_divisions);

  for (int i = 0; i < dist_divisions; ++i) {
    if (i * divider_distance_m_ <= occlusion_radius_) {
      offset_table_[i] = -1;
    } else {
      double occlusion_angle_deg =
        math::rad2deg(std::asin(occlusion_radius_ / (i * divider_distance_m_)));
      offset_table_[i] = std::floor(occlusion_angle_deg / divider_angle_deg_);
    }
  }
}

void PedestrianPluginManager::retrieveBaseControlShiftLinkPose()
{
  if (base_control_shift_link_) {
    base_to_control_tf_ = getTransformFromPose3d(base_control_shift_link_->WorldPose());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "base_control_shift link is not initialized properly.");
  }
}

bool PedestrianPluginManager::isWithinRange(
  const geometry_msgs::msg::Point & robot_point, const geometry_msgs::msg::Point & person_point)
{
  double dist = math::getDistance(
    robot_point.x, robot_point.y, person_point.x, person_point.y);

  return dist >= min_range_ && dist <= max_range_;
}

bool PedestrianPluginManager::initializedBaseControlShiftLink()
{
  if (robotAgent_ == nullptr) {
    return false;
  }

  model_ = gazebo::physics::get_world()->ModelByName(robotAgent_->name);
  if (model_) {
    base_control_shift_link_ = model_->GetLink("base_control_shift");
    if (base_control_shift_link_) {
      initialized_base_control_shift_link_ = true;
      return true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "base_control_shift link not found!");
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Model not found!");
    return false;
  }
}

bool PedestrianPluginManager::isWithinFOV(
  const double robot_yaw,
  const geometry_msgs::msg::Point & robot_point,
  const geometry_msgs::msg::Point & person_point)
{
  double angle_to_person = math::getAngleRad(
    robot_point.x, robot_point.y, person_point.x, person_point.y);
  double relative_angle = math::normalizeRad(angle_to_person - robot_yaw);

  return relative_angle >= min_angle_ && relative_angle <= max_angle_;
}

bool PedestrianPluginManager::isPersonVisible(
  int dist_idx, int angle_idx, std::vector<bool> & visibility_table,
  const std::vector<int> & offset_table)
{
  int ray_offset = offset_table[dist_idx];
  int angle_divisions = visibility_table.size();
  bool is_person_visible = true;
  if (ray_offset == -1) {  // when robot and person overlap
    is_person_visible = false;
    for (auto it = visibility_table.begin(); it != visibility_table.end(); ++it) {
      *it = false;
    }
  } else {
    for (int offset = -ray_offset; offset <= ray_offset; ++offset) {
      int check_idx = (angle_idx + offset + angle_divisions) % angle_divisions;
      if (!visibility_table[check_idx]) {
        is_person_visible = false;
      }
      visibility_table[check_idx] = false;
    }
  }

  return is_person_visible;
}

std::vector<people_msgs::msg::Person> PedestrianPluginManager::getNonOccludedPeople(
  const geometry_msgs::msg::Point & robot_point,
  const std::map<std::string, people_msgs::msg::Person> & people_map)
{
  std::vector<gazebo::PersonInfo> people_info_list;
  people_info_list.reserve(people_map.size());

  int angle_divisions = std::ceil(360.0 / divider_angle_deg_);
  std::vector<bool> visibility_table(angle_divisions, true);

  for (const auto & it : people_map) {
    const double dist = math::getDistance(
      robot_point.x, robot_point.y, it.second.position.x, it.second.position.y);
    const double deg = math::getAngleDeg(
      robot_point.x, robot_point.y, it.second.position.x, it.second.position.y);

    if (dist > max_range_) {continue;}

    gazebo::PersonInfo p;
    p.person = it.second;
    p.distance_m = math::roundDistance(dist, divider_distance_m_);
    p.angle_deg = math::roundAngleDeg(deg, divider_angle_deg_);

    people_info_list.push_back(p);
  }

  std::sort(
    people_info_list.begin(), people_info_list.end(),
    [](const gazebo::PersonInfo & a, const gazebo::PersonInfo & b) {
      return a.distance_m < b.distance_m;
    });

  std::vector<people_msgs::msg::Person> visible_people;

  for (const auto & person_info : people_info_list) {
    int dist_idx = math::getDistanceIndex(person_info.distance_m, divider_distance_m_);
    int angle_idx = math::getAngleIndex(person_info.angle_deg, divider_angle_deg_);
    bool is_visible = isPersonVisible(dist_idx, angle_idx, visibility_table, offset_table_);

    if (is_visible) {
      visible_people.push_back(person_info.person);
    }
  }

  return visible_people;
}

std::vector<people_msgs::msg::Person> PedestrianPluginManager::filterByDistanceAndAngle(
  const double robot_yaw,
  const geometry_msgs::msg::Point & robot_point,
  const std::vector<people_msgs::msg::Person> & people)
{
  std::vector<people_msgs::msg::Person> filtered_people;
  for (const auto & person : people) {
    if (!isWithinRange(robot_point, person.position)) {
      continue;
    }
    if (!isWithinFOV(robot_yaw, robot_point, person.position)) {
      continue;
    }
    filtered_people.push_back(person);
  }
  return filtered_people;
}

rcl_interfaces::msg::SetParametersResult PedestrianPluginManager::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  using rcl_interfaces::msg::ParameterType;

  rcl_interfaces::msg::SetParametersResult result;

  RCLCPP_INFO(node_->get_logger(), "Call for pedestrian_plugin params");

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "pedestrian_plugin.min_range") {
        min_range_ = parameter.as_double();
        RCLCPP_INFO(node_->get_logger(), "Change min_range parameter: %lf", min_range_);
      } else if (name == "pedestrian_plugin.max_range") {
        max_range_ = parameter.as_double();
        RCLCPP_INFO(node_->get_logger(), "Change max_range parameter: %lf", max_range_);
      } else if (name == "pedestrian_plugin.min_angle") {
        min_angle_ = parameter.as_double();
        RCLCPP_INFO(node_->get_logger(), "Change min_angle parameter: %lf", min_angle_);
      } else if (name == "pedestrian_plugin.max_angle") {
        max_angle_ = parameter.as_double();
        RCLCPP_INFO(node_->get_logger(), "Change max_angle parameter: %lf", max_angle_);
      } else if (name == "pedestrian_plugin.occlusion_radius") {
        occlusion_radius_ = parameter.as_double();
        RCLCPP_INFO(
          node_->get_logger(), "Change occlusion_radius parameter: %lf", occlusion_radius_);
      } else if (name == "pedestrian_plugin.divider_distance_m") {
        divider_distance_m_ = parameter.as_double();
        RCLCPP_INFO(node_->get_logger(), "Change divider_distance_m: %lf", divider_distance_m_);
      } else if (name == "pedestrian_plugin.divider_angle_deg") {
        divider_angle_deg_ = parameter.as_double();
        RCLCPP_INFO(node_->get_logger(), "Change divider_angle_deg: %lf", divider_angle_deg_);
      }
    }
  }

  setOffsetTable();

  result.successful = true;
  return result;
}

inline double PedestrianPluginManager::getYawFromTransform(const tf2::Transform & transform)
{
  tf2::Quaternion rotation = transform.getRotation();
  double roll, pitch, yaw;

  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

  return yaw;
}

inline geometry_msgs::msg::Point PedestrianPluginManager::getPointFromTransform(
  const tf2::Transform & transform)
{
  geometry_msgs::msg::Point point;
  tf2::Vector3 origin = transform.getOrigin();

  point.x = origin.x();
  point.y = origin.y();
  point.z = origin.z();

  return point;
}

inline tf2::Transform PedestrianPluginManager::getTransformFromPose3d(
  const ignition::math::Pose3d & pose)
{
  tf2::Vector3 origin(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  tf2::Quaternion rotation(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());

  tf2::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(rotation);

  return transform;
}
