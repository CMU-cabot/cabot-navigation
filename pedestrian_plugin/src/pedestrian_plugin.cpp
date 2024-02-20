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
  manager.removePlugin(this->name);
}

void PedestrianPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->name = this->actor->GetName();
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
    std::bind(&PedestrianPlugin::OnUpdate, this, std::placeholders::_1)));

  actor_id = manager.addPlugin(this->name, this);
  RCLCPP_INFO(manager.get_logger(), "Loading Pedestrign plugin...");

  PedestrianPluginParams temp_params;
  sdf::ElementPtr child = this->sdf->GetFirstElement();
  while (child) {
    std::string key = child->GetName();
    std::string type = "str";
    sdf::ParamPtr typeAttr = child->GetAttribute("type");
    if (typeAttr) {
      type = typeAttr->GetAsString();
    }
    std::string value = child->Get<std::string>();
    RCLCPP_INFO(manager.get_logger(), "plugin param %s (type=%s) value=%s",
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

void PedestrianPlugin::apply_parameters() {
  RCLCPP_INFO(manager.get_logger(), "apply_parameters");
  needs_to_apply_params = false;

  for (const auto &it : plugin_params) {
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
      this->yaw = it.second.get<double>() /180.0 * M_PI;
    }
  }
}

void PedestrianPlugin::update_parameters(PedestrianPluginParams params) {
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  plugin_params = params;
  needs_to_apply_params = true;
}

void PedestrianPlugin::Reset()
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
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

  auto pose = this->actor->WorldPose();
  auto rpy = pose.Rot().Euler();
  this->x = pose.Pos().X();
  this->y = pose.Pos().Y();
  this->z = pose.Pos().Z();
  this->roll = rpy.X();
  this->pitch = rpy.Y();
  this->yaw = rpy.Z();
  this->dist = 0;

  apply_parameters();
}



void PedestrianPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  IGN_PROFILE("PedestrianPlugin::Update");
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
  manager.publishPeopleIfReady();
  
  this->lastUpdate = _info.simTime;

  PyObject* pFunc = global_python_loader->getFunc(this->module_name, "onUpdate");

  if (pFunc != NULL) {
    PyObject* pArgs = PyTuple_New(0);
    PyObject* pDict = PyDict_New();

    if (robotModel) {
      PyObject* pRobotPose = PyDict_New();
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
      PythonUtils::setDictItemAsFloat(pRobotPose, "pitch", rRpy.X());
      PythonUtils::setDictItemAsFloat(pRobotPose, "yaw", rRpy.X());
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
      robotAgent.name = std::string("robot");
      robotAgent.position = robot_pose;
      robotAgent.yaw = rRpy.X();
      robotAgent.velocity.linear.x = vel_x;
      robotAgent.velocity.linear.y = vel_y;
      robotAgent.velocity.angular.z = vel_theta;
      robotAgent.linear_vel = vel_linear;
      robotAgent.angular_vel = vel_theta;
      robotAgent.radius = 0.45; // default robot raduis
      manager.updateRobotAgent(robotAgent);

      PyDict_SetItemString(pDict, "robot", pRobotPose);
      Py_DECREF(pRobotPose);
    }
    PythonUtils::setDictItemAsFloat(pDict, "time", _info.simTime.Float());
    PythonUtils::setDictItemAsFloat(pDict, "dt", dt);
    PythonUtils::setDictItemAsFloat(pDict, "x", this->x);
    PythonUtils::setDictItemAsFloat(pDict, "y", this->y);
    PythonUtils::setDictItemAsFloat(pDict, "z", this->z);
    PythonUtils::setDictItemAsFloat(pDict, "roll", this->roll);
    PythonUtils::setDictItemAsFloat(pDict, "pitch", this->pitch);
    PythonUtils::setDictItemAsFloat(pDict, "yaw", this->yaw);

    // add parameter to the arguments
    for (const auto& pair : this->plugin_params) {
      auto value = pair.second;
      PyObject *temp = value.python_object();
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
      auto radius = PythonUtils::getDictItemAsDouble(pRet, "radius", 0.4);
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
      if (dd / dt < 0.1) {
        person.tags.push_back("stationary");
      }
      manager.updatePersonMessage(this->actor->GetName(), person);

      // update human agent
      double vel_linear = dd / dt;
      pedestrian_plugin_msgs::msg::Agent humanAgent;
      humanAgent.type = pedestrian_plugin_msgs::msg::Agent::PERSON;
      if (this->module_name == "pedestrian.pool"){
        humanAgent.behavior_state = pedestrian_plugin_msgs::msg::Agent::INACTIVE;
      }else{
        humanAgent.behavior_state = pedestrian_plugin_msgs::msg::Agent::ACTIVE;
      }
      humanAgent.name = person.name;
      humanAgent.position.position = person.position;
      humanAgent.yaw = newYaw;
      humanAgent.velocity.linear.x = person.velocity.x;
      humanAgent.velocity.linear.y = person.velocity.y;
      humanAgent.velocity.linear.z = person.velocity.z;
      humanAgent.linear_vel = vel_linear;
      // humanAgent.velocity.angular.z // undefined
      // humanAgent.angular_vel // undefined
      humanAgent.radius = radius;
      manager.updateHumanAgent(person.name, humanAgent);

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
