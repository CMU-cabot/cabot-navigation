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

using namespace gazebo;  // NOLINT

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
  this->model = _model;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->name = this->model->GetName();
  this->world = this->model->GetWorld();

  this->connections.push_back(
    event::Events::ConnectWorldUpdateBegin(
      std::bind(&PedestrianPlugin::OnUpdate, this, std::placeholders::_1)));

  // register only if the model is actor
  actor_id = manager.addPlugin(this->name, this, this->actor==nullptr);
  RCLCPP_INFO(manager.get_logger(), "Loading Pedestrian plugin for %s...", this->name.c_str());

  if (this->actor) {
    // TODO: move to a separated class/file
    // Map of collision scaling factors
    // equivalent mapping from a actor collision plugin example
    // https://github.com/gazebosim/gazebo-classic/blob/e4b4d0fb752c7e43e34ab97d0e01a2a3eaca1ed4/examples/plugins/actor_collisions/actor_collisions.world.erb#L128-L239
    std::map<std::string, ignition::math::Vector3d> scaling;
    std::map<std::string, ignition::math::Pose3d> offsets;
    scaling["LHipJoint_LeftUpLeg_collision"] = ignition::math::Vector3d(0.01, 0.001, 0.001);
    scaling["LeftUpLeg_LeftLeg_collision"] = ignition::math::Vector3d(8.0, 8.0, 1.0);
    scaling["LeftLeg_LeftFoot_collision"] = ignition::math::Vector3d(8.0, 8.0, 1.0);
    scaling["LeftFoot_LeftToeBase_collision"] = ignition::math::Vector3d(4.0, 4.0, 1.5);
    scaling["RHipJoint_RightUpLeg_collision"] = ignition::math::Vector3d(0.01, 0.001, 0.001);
    scaling["RightUpLeg_RightLeg_collision"] = ignition::math::Vector3d(8.0, 8.0, 1.0);
    scaling["RightLeg_RightFoot_collision"] = ignition::math::Vector3d(8.0, 8.0, 1.0);
    scaling["RightFoot_RightToeBase_collision"] = ignition::math::Vector3d(4.0, 4.0, 1.5);
    scaling["LowerBack_Spine_collision"] = ignition::math::Vector3d(12.0, 20.0, 5.0);
    offsets["LowerBack_Spine_collision"] = ignition::math::Pose3d(0.05, 0, 0, 0, -0.2, 0);
    scaling["Spine_Spine1_collision"] = ignition::math::Vector3d(0.01, 0.001, 0.001);
    scaling["Neck_Neck1_collision"] = ignition::math::Vector3d(0.01, 0.001, 0.001);
    scaling["Neck1_Head_collision"] = ignition::math::Vector3d(5.0, 5.0, 3.0);
    scaling["LeftShoulder_LeftArm_collision"] = ignition::math::Vector3d(0.01, 0.001, 0.001);
    scaling["LeftArm_LeftForeArm_collision"] = ignition::math::Vector3d(5.0, 5.0, 1.0);
    scaling["LeftForeArm_LeftHand_collision"] = ignition::math::Vector3d(5.0, 5.0, 1.0);
    scaling["LeftFingerBase_LeftHandIndex1_collision"] = ignition::math::Vector3d(4.0, 4.0, 3.0);
    scaling["RightShoulder_RightArm_collision"] = ignition::math::Vector3d(0.01, 0.001, 0.001);
    scaling["RightArm_RightForeArm_collision"] = ignition::math::Vector3d(5.0, 5.0, 1.0);
    scaling["RightForeArm_RightHand_collision"] = ignition::math::Vector3d(5.0, 5.0, 1.0);
    scaling["RightFingerBase_RightHandIndex1_collision"] = ignition::math::Vector3d(4.0, 4.0, 3.0);

    // enable links of actor collidable with RayShape
    for (const auto &link : this->actor->GetLinks())
    {
      if (scaling.empty())
        continue;

      // Init the links, which in turn enables collisions
      link->Init();

      // Process all the collisions in all the links
      for (const auto &collision : link->GetCollisions())
      {
        auto name = collision->GetName();

        // Set bitmask for collisions so actors won't collide with actors
        // TODO: maybe option to collide?
        collision->GetSurface()->collideBitmask = 0x00;

        if (scaling.find(name) != scaling.end())
        {
          auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
              collision->GetShape());

          // Make sure we have a box shape.
          if (boxShape) {
            boxShape->SetSize(boxShape->Size() * scaling[name]);
          }
        }

        if (offsets.find(name) != offsets.end())
        {
          collision->SetInitialRelativePose(
              offsets[name] + collision->InitialRelativePose());
        }
      }
    }
  }

  // parse other parameters for plugins
  PedestrianPluginParams temp_params;
  sdf::ElementPtr child = this->sdf->GetFirstElement();
  while (child) {
    std::string key = child->GetName();
    if (key == "scaling") {
       child = child->GetNextElement();
       continue;
    }
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

void PedestrianPlugin::apply_parameters()
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

void PedestrianPlugin::update_parameters(PedestrianPluginParams params)
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  plugin_params = params;
  needs_to_apply_params = true;
  needs_to_reset_robot = 20;
}

void PedestrianPlugin::Reset()
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  RCLCPP_INFO(manager.get_logger(), "Reset");

  if (this->actor) {
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
  }

  auto pose = this->model->WorldPose();
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


void PedestrianPlugin::OnUpdate(const common::UpdateInfo & _info)
{
  std::lock_guard<std::recursive_mutex> guard(manager.mtx);
  IGN_PROFILE("PedestrianPlugin::Update");
  IGN_PROFILE_BEGIN("Update");
  if (needs_to_reset_robot > 0) {
      needs_to_reset_robot--;
      manager.publish_cmd_vel(0, 0, 0, 0, 0, 0);
      // work around to fix base_joint
      auto base_joint = this->model->GetJoint("base_joint");
      if (base_joint) {
        base_joint->SetPosition(0, 0, false);
      }
      return;
  } else {
    if (needs_to_apply_params) {
      apply_parameters();
      if (!this->actor) {
        ignition::math::Pose3d pose;
        pose.Pos().X(this->x);
        pose.Pos().Y(this->y);
        pose.Pos().Z(this->z);
        pose.Rot() = ignition::math::Quaterniond(0, 0, this->yaw);
        this->model->SetWorldPose(pose, true, true);
      }
    }
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
    if (robotModel) {
      auto robot_radius = PythonUtils::getDictItemAsDouble(pDict, "robot_radius", 0.45);

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
    PythonUtils::setDictItemAsFloat(pDict, "time", _info.simTime.Float());
    PythonUtils::setDictItemAsFloat(pDict, "dt", dt);
    PythonUtils::setDictItemAsFloat(pDict, "x", this->x);
    PythonUtils::setDictItemAsFloat(pDict, "y", this->y);
    PythonUtils::setDictItemAsFloat(pDict, "z", this->z);
    PythonUtils::setDictItemAsFloat(pDict, "roll", this->roll);
    PythonUtils::setDictItemAsFloat(pDict, "pitch", this->pitch);
    PythonUtils::setDictItemAsFloat(pDict, "yaw", this->yaw);

    PyObject * aname = PyUnicode_DecodeFSDefault(this->model->GetName().c_str());
    PyDict_SetItemString(pDict, "name", aname);
    Py_DECREF(aname);

    // call onUpdate in the python module
    auto pRet = PyObject_Call(pFunc, pArgs, pDict);

    if (pRet != NULL && PyDict_Check(pRet)) {
      if (this->actor) {
        auto newX = PythonUtils::getDictItemAsDouble(pRet, "x", 0.0);
        auto newY = PythonUtils::getDictItemAsDouble(pRet, "y", 0.0);
        auto newZ = PythonUtils::getDictItemAsDouble(pRet, "z", 0.0);
        auto newRoll = PythonUtils::getDictItemAsDouble(pRet, "roll", 0.0);
        auto newPitch = PythonUtils::getDictItemAsDouble(pRet, "pitch", 0.0);
        auto newYaw = PythonUtils::getDictItemAsDouble(pRet, "yaw", 0.0);
        // variables only get from the module
        auto radius = PythonUtils::getDictItemAsDouble(pRet, "radius", 0.4);
        auto progress = PythonUtils::getDictItemAsDouble(pRet, "progress", 1);

        auto dx = newX - this->x;
        auto dy = newY - this->y;
        auto dz = newZ - this->z;
        auto dd = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (progress == 0.0) {
          dd = 0.0;
        }

        // little hack to adjust animation rotation
        auto newDist = this->dist + dd;
        auto skelAnims = this->actor->SkeletonAnimations();
        auto it = skelAnims.find(WALKING_ANIMATION);
        if (it != skelAnims.end()) {
          auto skelAnim = it->second;
          auto first_frame = skelAnim->PoseAt(0);
          auto last_frame = skelAnim->PoseAt(skelAnim->GetLength());
          auto first_trans = first_frame["Hips"].Translation();
          auto last_trans = last_frame["Hips"].Translation();

          double dst = (newDist - this->dist) / (last_trans.X() - first_trans.Y()) * (skelAnim->GetLength());
          this->actor->SetScriptTime(this->actor->ScriptTime() + dst);

          auto frame = skelAnim->PoseAt(this->actor->ScriptTime());

          ignition::math::Matrix4d rootTrans = ignition::math::Matrix4d::Identity;

          // "Hips" is hardcoded and needs to be fixed
          auto iter = frame.find("Hips");
          if (iter != frame.end())
          {
            rootTrans = frame["Hips"];
          }

          ignition::math::Vector3d rootPos = rootTrans.Translation();
          ignition::math::Quaterniond rootRot = rootTrans.Rotation();

          ignition::math::Pose3d actorPose;
          ignition::math::Pose3d pose;
          pose.Pos().X(newX);
          pose.Pos().Y(newY);
          pose.Pos().Z(newZ);
          pose.Rot() = ignition::math::Quaterniond(newRoll, newPitch, newYaw);

          rootPos.X(0);
          rootPos *= this->actor->Scale();
          pose.Pos() = pose.Pos() + pose.Rot().RotateVector(rootPos);
          pose.Rot() = pose.Rot() * rootRot;
          this->actor->SetWorldPose(pose, true, true);
        }

        /*
        auto newDist = this->dist + dd;
        double *wPose = get_walking_pose(newDist);

        ignition::math::Pose3d pose;
        pose.Pos().X(newX);
        pose.Pos().Y(newY + wPose[1]);
        pose.Pos().Z(newZ + wPose[2]);
        pose.Rot() = ignition::math::Quaterniond(newRoll + wPose[3], newPitch + wPose[4], newYaw + wPose[5]);
        this->actor->SetWorldPose(pose, true, true);

        double dst = (newDist - this->dist) / walking_dist_factor * walking_time_factor;
        this->actor->SetScriptTime(this->actor->ScriptTime() + dst);
        */

        this->x = newX;
        this->y = newY;
        this->z = newZ;
        this->roll = newRoll;
        this->pitch = newPitch;
        this->yaw = newYaw;
        this->dist = newDist;

        people_msgs::msg::Person person;
        person.name = this->name;
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
        if (this->module_name == "pedestrian.pool") {
          humanAgent.behavior_state = pedestrian_plugin_msgs::msg::Agent::INACTIVE;
        } else {
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
      } else {
        auto pCmdVel = PythonUtils::getDictItemAsDict(pRet, "cmd_vel");
        // if cmd_vel is specified, do not update the position in this plugin        
        if (pCmdVel != NULL) {
          auto pLinear = PythonUtils::getDictItemAsDict(pCmdVel, "linear");
          auto pAngular = PythonUtils::getDictItemAsDict(pCmdVel, "angular");
          if (pLinear && pAngular) {
            auto lx = PythonUtils::getDictItemAsDouble(pLinear, "x", 0.0);
            auto ly = PythonUtils::getDictItemAsDouble(pLinear, "y", 0.0);
            auto lz = PythonUtils::getDictItemAsDouble(pLinear, "z", 0.0);
            auto ax = PythonUtils::getDictItemAsDouble(pAngular, "x", 0.0);
            auto ay = PythonUtils::getDictItemAsDouble(pAngular, "y", 0.0);
            auto az = PythonUtils::getDictItemAsDouble(pAngular, "z", 0.0);
            manager.publish_cmd_vel(lx, ly, lz, ax, ay, az);
          }
          if (pLinear) {
            Py_DECREF(pLinear);
          }
          if (pAngular) {
            Py_DECREF(pAngular);
          }
          Py_DECREF(pCmdVel);

          auto pose = this->model->WorldPose();
          this->x = pose.Pos().X();
          this->y = pose.Pos().Y();
          this->z = pose.Pos().Z();
          this->roll = pose.Rot().X();
          this->pitch = pose.Rot().Y();
          this->yaw = pose.Rot().Z();
          
        } else {
          auto newX = PythonUtils::getDictItemAsDouble(pRet, "x", 0.0);
          auto newY = PythonUtils::getDictItemAsDouble(pRet, "y", 0.0);
          auto newZ = PythonUtils::getDictItemAsDouble(pRet, "z", 0.0);
          auto newRoll = PythonUtils::getDictItemAsDouble(pRet, "roll", 0.0);
          auto newPitch = PythonUtils::getDictItemAsDouble(pRet, "pitch", 0.0);
          auto newYaw = PythonUtils::getDictItemAsDouble(pRet, "yaw", 0.0);
          RCLCPP_INFO(manager.get_logger(), "x=%.2f, y=%.2f", newX, newY);

          ignition::math::Pose3d pose;
          pose.Pos().X(newX);
          pose.Pos().Y(newY);
          pose.Pos().Z(newZ);
          pose.Rot() = ignition::math::Quaterniond(newRoll, newPitch, newYaw);
          this->model->SetWorldPose(pose, true, true);

          this->x = newX;
          this->y = newY;
          this->z = newZ;
          this->roll = newRoll;
          this->pitch = newPitch;
          this->yaw = newYaw;
        }
      }
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
