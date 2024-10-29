// Copyright (c) 2024  Carnegie Mellon University and Miraikan
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

#include <functional>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/common/Profiler.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/BoxShape.hh"
#include "pedestrian_plugin/obstacle_plugin.hpp"
#include "pedestrian_plugin/python_module_loader.hpp"
#include "pedestrian_plugin/python_utils.hpp"

using namespace gazebo;  // NOLINT

GZ_REGISTER_MODEL_PLUGIN(ObstaclePlugin)

// ObstaclePlugin implementation

ObstaclePlugin::ObstaclePlugin()
: manager(ObstaclePluginManager::getInstance())
{
}

ObstaclePlugin::~ObstaclePlugin()
{
  manager.removePlugin(this->name);
}

void ObstaclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->model = _model;
  this->name = this->model->GetName();
  this->world = this->model->GetWorld();

  this->connections.push_back(
    event::Events::ConnectWorldUpdateBegin(
      std::bind(&ObstaclePlugin::OnUpdate, this, std::placeholders::_1)));

  obstacle_id = manager.addPlugin(this->name, this);
  RCLCPP_INFO(manager.get_logger(), "Loading Pedestrign plugin...");

  ObstaclePluginParams temp_params;
  sdf::ElementPtr child = this->sdf->GetFirstElement();
  while (child) {
    std::string key = child->GetName();
    std::string type = "str";
    sdf::ParamPtr typeAttr = child->GetAttribute("type");
    if (typeAttr) {
      type = typeAttr->GetAsString();
    }
    std::string value = child->Get<std::string>();
    RCLCPP_INFO(
      manager.get_logger(), "plugin param %s (type=%s) value=%s",
      key.c_str(), type.c_str(), value.c_str());

    if (key == "robot") {
      if (value != "") {
        robotModel = this->world->ModelByName(value);
      }
    } else {
      temp_params.addParam(key, type, value);
    }
    child = child->GetNextElement();
  }
  update_parameters(temp_params);
  this->Reset();
}

void ObstaclePlugin::apply_parameters()
{
  RCLCPP_INFO(manager.get_logger(), "apply_parameters");
  needs_to_apply_params = false;

  for (const auto & it : plugin_params) {
    RCLCPP_INFO(manager.get_logger(), "param %s", it.first.c_str());
    if (it.first == "module") {
      this->module_name = it.second.get<std::string>();
      RCLCPP_INFO(manager.get_logger(), "module name is %s", this->module_name.c_str());
      global_python_loader->reset();
      global_python_loader->loadModule(this->module_name);
    }
    if (it.first == "init_x") {
      this->x = it.second.get<double>();
    }
    if (it.first == "init_y") {
      this->y = it.second.get<double>();
    }
    if (it.first == "init_z") {
      this->z = it.second.get<double>();
    }
    if (it.first == "init_a") {
      this->yaw = it.second.get<double>() / 180.0 * M_PI;
    }
  }
}

void ObstaclePlugin::update_parameters(ObstaclePluginParams params)
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  plugin_params = params;
  needs_to_apply_params = true;
}

void ObstaclePlugin::Reset()
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  RCLCPP_INFO(manager.get_logger(), "Reset");

  auto pose = this->model->WorldPose();
  auto rpy = pose.Rot().Euler();
  this->x = pose.Pos().X();
  this->y = pose.Pos().Y();
  this->z = pose.Pos().Z();
  this->roll = rpy.X();
  this->pitch = rpy.Y();
  this->yaw = rpy.Z();
  this->dist = 0;

  gazebo::physics::ShapePtr shape_ptr = this->model->GetLink(this->name + "-link")->GetCollision(this->name + "-collision")->GetShape();
  physics::BoxShapePtr box_ptr = boost::dynamic_pointer_cast<physics::BoxShape>(shape_ptr);
  this->width = box_ptr->Size().X();
  this->height = box_ptr->Size().Y();
  this->depth = box_ptr->Size().Z();

  apply_parameters();
}


void ObstaclePlugin::OnUpdate(const common::UpdateInfo & _info)
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  IGN_PROFILE("ObstaclePlugin::Update");
  IGN_PROFILE_BEGIN("Update");

  if (needs_to_apply_params) {
    apply_parameters();
  }

  auto dt = (_info.simTime - this->lastUpdate).Double();
  if (dt > 1 || dt < 0) {  // reset, initialize
    this->lastUpdate = _info.simTime;
    return;
  }
  if (dt < 0.05) {  // 20hz
    return;
  }
  auto stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  manager.updateStamp(stamp);
  manager.publishObstaclesIfReady();

  this->lastUpdate = _info.simTime;

  PyObject * pFunc = global_python_loader->getFunc(this->module_name, "onUpdate");

  if (pFunc != NULL) {
    PyObject * pArgs = PyTuple_New(0);
    PyObject * pDict = PyDict_New();

    // add plugin parameters to the arguments
    for (const auto & pair : this->plugin_params) {
      auto value = pair.second;
      PyObject * temp = value.python_object();
      PyDict_SetItemString(pDict, pair.first.c_str(), temp);
      Py_DECREF(temp);
    }

    // set robot values to pDict
    double robot_radius;
    if (robotModel) {
      robot_radius = PythonUtils::getDictItemAsDouble(pDict, "robot_radius", 0.45);

      PyObject * pRobotPose = PyDict_New();
      ignition::math::Vector3d rPos = robotModel->WorldPose().Pos();
      ignition::math::Quaterniond rRot = robotModel->WorldPose().Rot();
      ignition::math::Vector3d rRpy = rRot.Euler();
      ignition::math::Vector3d linearVel = robotModel->WorldLinearVel();
      auto vel_x = linearVel.X();
      auto vel_y = linearVel.Y();
      auto vel_linear = linearVel.Length();
      ignition::math::Vector3d angularVel = robotModel->WorldAngularVel();
      auto vel_theta = angularVel.Z();
      PythonUtils::setDictItemAsFloat(pRobotPose, "x", rPos.X());
      PythonUtils::setDictItemAsFloat(pRobotPose, "y", rPos.Y());
      PythonUtils::setDictItemAsFloat(pRobotPose, "z", rPos.Z());
      PythonUtils::setDictItemAsFloat(pRobotPose, "roll", rRpy.X());
      PythonUtils::setDictItemAsFloat(pRobotPose, "pitch", rRpy.Y());
      PythonUtils::setDictItemAsFloat(pRobotPose, "yaw", rRpy.Z());
      PythonUtils::setDictItemAsFloat(pRobotPose, "vel_linear", vel_linear);
      PythonUtils::setDictItemAsFloat(pRobotPose, "vel_theta", vel_theta);
      geometry_msgs::msg::Pose robot_pose;
      robot_pose.position.x = rPos.X();
      robot_pose.position.y = rPos.Y();
      robot_pose.position.z = rPos.Z();
      robot_pose.orientation.x = rRot.X();
      robot_pose.orientation.y = rRot.Y();
      robot_pose.orientation.z = rRot.Z();
      robot_pose.orientation.w = rRot.W();
      manager.updateRobotPose(robot_pose);

      // update robot agent
      pedestrian_plugin_msgs::msg::Agent robotAgent;
      robotAgent.type = pedestrian_plugin_msgs::msg::Agent::ROBOT;
      robotAgent.behavior_state = pedestrian_plugin_msgs::msg::Agent::ACTIVE;
      robotAgent.name = robotModel->GetName();
      robotAgent.position = robot_pose;
      robotAgent.yaw = rRpy.Z();
      robotAgent.velocity.linear.x = vel_x;
      robotAgent.velocity.linear.y = vel_y;
      robotAgent.velocity.angular.z = vel_theta;
      robotAgent.linear_vel = vel_linear;
      robotAgent.angular_vel = vel_theta;
      robotAgent.radius = robot_radius;  // default robot raduis
      manager.updateRobotAgent(robotAgent);

      PyDict_SetItemString(pDict, "robot", pRobotPose);
      Py_DECREF(pRobotPose);
    }

    // set actor values to pDict
    PythonUtils::setDictItemAsFloat(pDict, "x", this->x);
    PythonUtils::setDictItemAsFloat(pDict, "y", this->y);
    PythonUtils::setDictItemAsFloat(pDict, "yaw", this->yaw);
    PythonUtils::setDictItemAsFloat(pDict, "obstacle_width", this->width);
    PythonUtils::setDictItemAsFloat(pDict, "obstacle_height", this->height);
    PythonUtils::setDictItemAsFloat(pDict, "robot_radius", robot_radius);

    PyObject * aname = PyUnicode_DecodeFSDefault(this->model->GetName().c_str());
    PyDict_SetItemString(pDict, "name", aname);
    Py_DECREF(aname);

    // call onUpdate in the python module
    auto pRet = PyObject_Call(pFunc, pArgs, pDict);

    if (pRet != NULL && PyDict_Check(pRet)) {
      // Obstacles do not move. Following variables are not necessary.
      auto newX = PythonUtils::getDictItemAsDouble(pRet, "x", 0.0);
      auto newY = PythonUtils::getDictItemAsDouble(pRet, "y", 0.0);
      auto newZ = PythonUtils::getDictItemAsDouble(pRet, "z", 0.0);
      auto newRoll = PythonUtils::getDictItemAsDouble(pRet, "roll", 0.0);
      auto newPitch = PythonUtils::getDictItemAsDouble(pRet, "pitch", 0.0);
      auto newYaw = PythonUtils::getDictItemAsDouble(pRet, "yaw", 0.0);

      // variables only get from the module
      auto [[maybe_unused]] radius = PythonUtils::getDictItemAsDouble(pRet, "radius", 0.4);
      auto progress = PythonUtils::getDictItemAsDouble(pRet, "progress", 1);

      auto dx = newX - this->x;
      auto dy = newY - this->y;
      auto dz = newZ - this->z;
      auto dd = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (progress == 0.0) {
        dd = 0.0;
      }
      auto newDist = this->dist + dd;

      ignition::math::Pose3d pose;
      pose.Pos().X(newX);
      pose.Pos().Y(newY);
      pose.Pos().Z(newZ);
      pose.Rot() = ignition::math::Quaterniond(newRoll, newPitch, newYaw);
      this->model->SetWorldPose(pose);

      // code depends on being obstacle as Obstacle msg...
      pedestrian_plugin_msgs::msg::Obstacle obstacle;
      obstacle.name = this->name;
      obstacle.position.x = this->x;
      obstacle.position.y = this->y;
      obstacle.position.z = this->z;
      obstacle.reliability = 1.0;
      obstacle.heading = this->yaw;
      obstacle.size.x = this->width;
      obstacle.size.y = this->height;
      obstacle.size.z = this->depth;
      manager.updateObstacleMessage(this->model->GetName(), obstacle);

      // update obstacle agent
      pedestrian_plugin_msgs::msg::Agent obstacleAgent;
      obstacleAgent.type = pedestrian_plugin_msgs::msg::Agent::OBSTACLE;
      obstacleAgent.behavior_state = pedestrian_plugin_msgs::msg::Agent::ACTIVE;
      obstacleAgent.name = obstacle.name;
      obstacleAgent.position.position = obstacle.position;
      obstacleAgent.yaw = obstacle.heading;
      obstacleAgent.velocity.linear.x = 0;
      obstacleAgent.velocity.linear.y = 0;
      obstacleAgent.velocity.linear.z = 0;
      obstacleAgent.linear_vel = 0;
      manager.updateObstacleAgent(obstacleAgent.name, obstacleAgent);

      this->x = newX;
      this->y = newY;
      this->z = newZ;
      this->roll = newRoll;
      this->pitch = newPitch;
      this->yaw = newYaw;
      this->dist = newDist;


      Py_DECREF(pRet);
    } else {
      PyErr_Print();
    }
    Py_DECREF(pFunc);
    Py_DECREF(pArgs);
    Py_DECREF(pDict);
  }

  IGN_PROFILE_END();
}
