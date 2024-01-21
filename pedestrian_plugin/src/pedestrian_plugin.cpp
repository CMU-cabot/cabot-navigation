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
#include "pedestrian_plugin.hpp"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedestrianPlugin)

#define WALKING_ANIMATION "walking"


PedestrianPlugin::PedestrianPlugin(): animationFactor(4.5)
{
}

void PedestrianPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&PedestrianPlugin::OnUpdate, this, std::placeholders::_1)));


  this->node = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(this->node->get_logger(), "Loading Pedestrign plugin...");
  if (_sdf->HasElement("module")) {
    std::string module_name = _sdf->Get<std::string>("module");
    RCLCPP_INFO(this->node->get_logger(), "module name is %s", module_name.c_str());

    Py_Initialize();
    auto pName = PyUnicode_DecodeFSDefault(module_name.c_str());
    this->pModule = PyImport_Import(pName);
    Py_DECREF(pName);
    
    if (this->pModule != NULL) {
      this->pOnUpdateFunc = PyObject_GetAttrString(this->pModule, "onUpdate");
      
      if (this->pOnUpdateFunc && PyCallable_Check(this->pOnUpdateFunc)) {
        RCLCPP_INFO(this->node->get_logger(), "found onUpdate func");
        auto pArgs = PyTuple_New(10);
        for (int i = 0; i < 10; ++i) {
          auto pValue = PyLong_FromLong(i);
          PyTuple_SetItem(pArgs, i, pValue);
        }
        RCLCPP_INFO(this->node->get_logger(), "calling onUpdate func");
        auto pRet = PyObject_CallObject(this->pOnUpdateFunc, pArgs);
        RCLCPP_INFO(this->node->get_logger(), "called onUpdate func");
        if (pRet != NULL) {
          RCLCPP_INFO(this->node->get_logger(), "print pRet");
          print_pyobject(pRet);
          Py_DECREF(pRet);
        }
        RCLCPP_INFO(this->node->get_logger(), "decref");
        Py_DECREF(pArgs);
        RCLCPP_INFO(this->node->get_logger(), "done");
      } else {
        RCLCPP_INFO(this->node->get_logger(), "cannot found onUpdate func");
      }
    } else {
      RCLCPP_INFO(this->node->get_logger(), "cannot found module %s", module_name.c_str());
    }
  }

  this->Reset();
}


void PedestrianPlugin::Reset()
{
  this->lastUpdate = 0;

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end()) {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  } else {
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

void PedestrianPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  IGN_PROFILE("PedestrianPlugin::Update");
  IGN_PROFILE_BEGIN("Update");

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d rpy = pose.Rot().Euler();
  pose.Pos().Z(1.0);
  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z());
  double dt = (_info.simTime - this->lastUpdate).Double();
  
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + dt);
  this->lastUpdate = _info.simTime;
  IGN_PROFILE_END();
}

void PedestrianPlugin::print_pyobject(PyObject* obj) {
  if (PyLong_Check(obj)) {
    // It's a long integer
    long value = PyLong_AsLong(obj);
    RCLCPP_INFO(this->node->get_logger(), "Integer: %ld\n", value);
  } else if (PyFloat_Check(obj)) {
    // It's a float
    double value = PyFloat_AsDouble(obj);
    RCLCPP_INFO(this->node->get_logger(), "Float: %f\n", value);
  } else if (PyUnicode_Check(obj)) {
    // It's a Unicode string
    PyObject *tempBytes = PyUnicode_AsEncodedString(obj, "utf-8", "strict");
    if (tempBytes != NULL) {
      char *str = PyBytes_AsString(tempBytes);
      if (str != NULL) {
        RCLCPP_INFO(this->node->get_logger(), "String: %s\n", str);
      }
      Py_DECREF(tempBytes);
    }
  } else if (PyList_Check(obj)) {
    // It's a list
    Py_ssize_t size = PyList_Size(obj);
    RCLCPP_INFO(this->node->get_logger(), "List of size %zd: [", size);
    for (Py_ssize_t i = 0; i < size; i++) {
      PyObject *item = PyList_GetItem(obj, i);
      print_pyobject(item);  // Recursive call to print each item
      if (i < size - 1) {
        RCLCPP_INFO(this->node->get_logger(), ", ");
      }
    }
    RCLCPP_INFO(this->node->get_logger(), "]\n");
  } else if (PyDict_Check(obj)) {
    // It's a dictionary
    RCLCPP_INFO(this->node->get_logger(), "Dictionary: {\n");
    PyObject *key, *value;
    Py_ssize_t pos = 0;

    while (PyDict_Next(obj, &pos, &key, &value)) {
      print_pyobject(key);
      RCLCPP_INFO(this->node->get_logger(), ": ");
      print_pyobject(value);
      RCLCPP_INFO(this->node->get_logger(), "\n");
    }
    RCLCPP_INFO(this->node->get_logger(), "}\n");
  } else if (PyTuple_Check(obj)) {
    // It's a tuple
    Py_ssize_t size = PyTuple_Size(obj);
    RCLCPP_INFO(this->node->get_logger(), "Tuple of size %zd: (", size);
    for (Py_ssize_t i = 0; i < size; i++) {
      PyObject *item = PyTuple_GetItem(obj, i);  // Get item from tuple
      print_pyobject(item);  // Recursive call to print each item
      if (i < size - 1) {
        RCLCPP_INFO(this->node->get_logger(), ", ");
      }
    }
    RCLCPP_INFO(this->node->get_logger(), ")\n");
  } else {
    // Object type is not handled in this example
    RCLCPP_INFO(this->node->get_logger(), "Object type not handled in this example.\n");
  }
}
