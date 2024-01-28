/*******************************************************************************
 * Copyright (c) 2024  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include <functional>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/common/Profiler.hh>
#include "gazebo/physics/physics.hh"
#include "pedestrian_plugin/pedestrian_plugin.hpp"
#include "pedestrian_plugin/python_module_loader.hpp"
#include "pedestrian_plugin/python_utils.hpp"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PedestrianPlugin)

#define WALKING_ANIMATION "walking"

// PedestrianPlugin implementation

PedestrianPlugin::PedestrianPlugin()
: manager(PedestrianPluginManager::getInstance())
{
  
}

PedestrianPlugin::~PedestrianPlugin()
{
}

void PedestrianPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
    std::bind(&PedestrianPlugin::OnUpdate, this, std::placeholders::_1)));

  actor_id = manager.addActor(_sdf);
  RCLCPP_INFO(manager.get_logger(), "Loading Pedestrign plugin...");

  this->Reset();
}


void PedestrianPlugin::Reset()
{
  std::lock_guard<std::mutex> guard(manager.mtx);
  RCLCPP_INFO(manager.get_logger(), "Reset");

  auto skelAnims = this->actor->SkeletonAnimations();
  auto it = skelAnims.find(WALKING_ANIMATION);
  if (it == skelAnims.end()) {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  } else {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }

  // reset python context
  this->plugin_params.clear();
  if (global_python_loader->canReset()) {
    RCLCPP_INFO(manager.get_logger(), "global_python_loader reset");
    global_python_loader->reset();
    PyImport_AppendInittab("ros", &PyInit_ros);
  }
  Py_Initialize();

  // load python modulde and parse plugin parameters as python object
  sdf::ElementPtr child = this->sdf->GetFirstElement();
  while (child) {
    std::string key = child->GetName();
    if (key == "module") {
      this->module_name = child->Get<std::string>();
      RCLCPP_INFO(manager.get_logger(), "module name is %s", this->module_name.c_str());
      global_python_loader->loadModule(this->module_name);
      child = child->GetNextElement();
      continue;
    }
    if (key == "robot") {
      auto robot_name = child->Get<std::string>();
      if (robot_name != "") {
        RCLCPP_INFO(manager.get_logger(), "robot name is %s", robot_name.c_str());
        robotModel = this->world->ModelByName(robot_name);
      }
      child = child->GetNextElement();
      continue;
    }

    sdf::ParamPtr typeAttr = child->GetAttribute("type");
    if (!typeAttr) {
      RCLCPP_ERROR(manager.get_logger(), "%s' type is not specified", key.c_str());
      child = child->GetNextElement();
      continue;
    }
    std::string type = typeAttr->GetAsString();
    std::string value = child->Get<std::string>();
    RCLCPP_INFO(manager.get_logger(), "plugin param %s (type=%s) value=%s",
      key.c_str(), type.c_str(), value.c_str());
    this->plugin_params[key] = child;
    child = child->GetNextElement();
  }

  auto pose = this->actor->WorldPose();
  auto rpy = pose.Rot().Euler();
  this->x = pose.Pos().X();
  this->y = pose.Pos().Y();
  this->z = pose.Pos().Z();
  this->roll = rpy.X();
  this->pitch = rpy.Y();
  this->yaw = rpy.Z();
  this->dist = 0;
}



void PedestrianPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  std::lock_guard<std::mutex> guard(manager.mtx);
  IGN_PROFILE("PedestrianPlugin::Update");
  IGN_PROFILE_BEGIN("Update");

  auto dt = (_info.simTime - this->lastUpdate).Double();
  if (dt > 1 || dt < 0) {  // reset, initialize
    this->lastUpdate = _info.simTime;
    return;
  }
  if (dt < 0.05) {  // 20hz
    return;
  }
  manager.publishPeopleIfReady();
  
  this->lastUpdate = _info.simTime;

  PyObject* pFunc = global_python_loader->getFunc(this->module_name, "onUpdate");

  if (pFunc != NULL) {
    PyObject* pArgs = PyTuple_New(0);
    PyObject* pDict = PyDict_New();

    if (robotModel) {
      PyObject* pRobotPose = PyDict_New();
      ignition::math::Vector3d rPos = robotModel->WorldPose().Pos();
      ignition::math::Vector3d rRpy = robotModel->WorldPose().Rot().Euler();
      PythonUtils::setDictItemAsFloat(pRobotPose, "x", rPos.X());
      PythonUtils::setDictItemAsFloat(pRobotPose, "y", rPos.Y());
      PythonUtils::setDictItemAsFloat(pRobotPose, "z", rPos.Z());
      PythonUtils::setDictItemAsFloat(pRobotPose, "roll", rRpy.X());
      PythonUtils::setDictItemAsFloat(pRobotPose, "pitch", rRpy.X());
      PythonUtils::setDictItemAsFloat(pRobotPose, "yaw", rRpy.X());
      PyDict_SetItemString(pDict, "robot", pRobotPose);
      Py_DECREF(pRobotPose);
    }
    PythonUtils::setDictItemAsFloat(pDict, "dt", dt);
    PythonUtils::setDictItemAsFloat(pDict, "x", this->x);
    PythonUtils::setDictItemAsFloat(pDict, "y", this->y);
    PythonUtils::setDictItemAsFloat(pDict, "z", this->z);
    PythonUtils::setDictItemAsFloat(pDict, "roll", this->roll);
    PythonUtils::setDictItemAsFloat(pDict, "pitch", this->pitch);
    PythonUtils::setDictItemAsFloat(pDict, "yaw", this->yaw);

    // add parameter to the arguments
    for (const auto& pair : this->plugin_params) {
      auto child = pair.second;
      sdf::ParamPtr typeAttr = child->GetAttribute("type");
      std::string type = typeAttr->GetAsString();
      PyObject* temp;
      if (type == "str") {
        temp = PyUnicode_FromString(child->Get<std::string>().c_str());
      } else if (type == "int") {
        temp = PyLong_FromLong(child->Get<int>());
      } else if (type == "float") {
        temp = PyFloat_FromDouble(child->Get<double>());
      } else if (type == "bool") {
        temp = child->Get<bool>() ? Py_True : Py_False;
      } else {
        RCLCPP_ERROR(manager.get_logger(), "Unsupported type: %s", type.c_str());
      }
      PyDict_SetItemString(pDict, pair.first.c_str(), temp);
      Py_DECREF(temp);
    }
    
    PyObject *aname = PyUnicode_DecodeFSDefault(this->actor->GetName().c_str());
    PyDict_SetItemString(pDict, "name", aname);
    Py_DECREF(aname);

    auto pRet = PyObject_Call(pFunc, pArgs, pDict);
    if (pRet != NULL && PyDict_Check(pRet)) {
      auto newX = PythonUtils::getDictItemAsDouble(pRet, "x", 0.0);
      auto newY = PythonUtils::getDictItemAsDouble(pRet, "y", 0.0);
      auto newZ = PythonUtils::getDictItemAsDouble(pRet, "z", 0.0);
      auto newRoll = PythonUtils::getDictItemAsDouble(pRet, "roll", 0.0);
      auto newPitch = PythonUtils::getDictItemAsDouble(pRet, "pitch", 0.0);
      auto newYaw = PythonUtils::getDictItemAsDouble(pRet, "yaw", 0.0);
      auto dx = newX - this->x;
      auto dy = newY - this->y;
      auto dz = newZ - this->z;
      auto dd = std::sqrt(dx * dx + dy * dy + dz * dz);
      auto newDist = this->dist + dd;
      double *wPose = get_walking_pose(newDist);

      ignition::math::Pose3d pose;
      pose.Pos().X(newX);
      pose.Pos().Y(newY+wPose[1]);
      pose.Pos().Z(newZ+wPose[2]);
      pose.Rot() = ignition::math::Quaterniond(newRoll+wPose[3], newPitch+wPose[4], newYaw+wPose[5]);
      this->actor->SetWorldPose(pose, false, false);

      double dst = (newDist - this->dist) / walking_dist_factor * walking_time_factor;
      this->actor->SetScriptTime(this->actor->ScriptTime() + dst);

      this->x = newX;
      this->y = newY;
      this->z = newZ;
      this->roll = newRoll;
      this->pitch = newPitch;
      this->yaw = newYaw;
      this->dist = newDist;

      people_msgs::msg::Person person;
      person.name = std::to_string(actor_id);
      person.position.x = newX;
      person.position.y = newY;
      person.position.z = newZ;
      person.velocity.x = std::cos(newYaw) * dd / dt;
      person.velocity.y = std::sin(newYaw) * dd / dt;
      person.velocity.z = 0.0;
      person.reliability = 1.0;
      manager.addPersonMessage(person);

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
