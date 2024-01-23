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

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PedestrianPlugin)

#define WALKING_ANIMATION "walking"

// exporting python module
static gazebo_ros::Node::SharedPtr global_node;
static std::shared_ptr<PythonModuleLoader> loader = std::make_shared<PythonModuleLoader>();

static PyObject* ros_info(PyObject *self, PyObject *args)
{
  const char* message;
  if (PyArg_ParseTuple(args, "s", &message)) {
    if (global_node) {
      RCLCPP_INFO(global_node->get_logger(), message);
    }
  }
  Py_RETURN_NONE;
}
static PyMethodDef RosMethods[] = {
  {"info", ros_info, METH_VARARGS,
   "call RCLCPP_INFO"},
  {NULL, NULL, 0, NULL}
};
static PyModuleDef RosModule = {
  PyModuleDef_HEAD_INIT, "ros", NULL, -1, RosMethods,
  NULL, NULL, NULL, NULL
};
static PyObject* PyInit_ros(void)
{
  return PyModule_Create(&RosModule);
}

PedestrianPlugin::PedestrianPlugin()
{
}

PedestrianPlugin::~PedestrianPlugin()
{
  loader = nullptr;
}

void PedestrianPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
    std::bind(&PedestrianPlugin::OnUpdate, this, std::placeholders::_1)));


  global_node = this->node = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(this->node->get_logger(), "Loading Pedestrign plugin...");
  if (_sdf->HasElement("module")) {
    this->module_name = _sdf->Get<std::string>("module");
    RCLCPP_INFO(this->node->get_logger(), "module name is %s", this->module_name.c_str());
  }
  this->Reset();
}


void PedestrianPlugin::Reset()
{
  RCLCPP_INFO(this->node->get_logger(), "Reset");
  this->lastUpdate = 0;
  this->lastDist = 0;

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

  if (loader->canReset()) {
    RCLCPP_INFO(this->node->get_logger(), "loader reset");
    loader->reset();
  }
  PyImport_AppendInittab("ros", &PyInit_ros);
  loader->loadModule(this->module_name);
}

void PedestrianPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  IGN_PROFILE("PedestrianPlugin::Update");
  IGN_PROFILE_BEGIN("Update");

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  PyObject* func = loader->getFunc(this->module_name, "onUpdate");

  if (func != NULL) {
    auto pArgs = PyTuple_New(6);
    PyTuple_SetItem(pArgs, 0, PyFloat_FromDouble(pose.X()));
    PyTuple_SetItem(pArgs, 1, PyFloat_FromDouble(pose.Y()));
    PyTuple_SetItem(pArgs, 2, PyFloat_FromDouble(pose.Z()));
    PyTuple_SetItem(pArgs, 3, PyFloat_FromDouble(rpy.X()));
    PyTuple_SetItem(pArgs, 4, PyFloat_FromDouble(rpy.Y()));
    PyTuple_SetItem(pArgs, 5, PyFloat_FromDouble(rpy.Z()));

    auto pRet = PyObject_CallObject(func, pArgs);
    if (pRet != NULL && PyTuple_Check(pRet) && PyTuple_Size(pRet) == 7) {
      double x = PyFloat_AsDouble(PyTuple_GetItem(pRet, 0));
      double y = PyFloat_AsDouble(PyTuple_GetItem(pRet, 1));
      double z = PyFloat_AsDouble(PyTuple_GetItem(pRet, 2));
      double roll = PyFloat_AsDouble(PyTuple_GetItem(pRet, 3));
      double pitch = PyFloat_AsDouble(PyTuple_GetItem(pRet, 4));
      double yaw = PyFloat_AsDouble(PyTuple_GetItem(pRet, 5));
      double dist = PyFloat_AsDouble(PyTuple_GetItem(pRet, 6));

      double *wPose = get_walking_pose(dist);
      pose.Pos().X(x);
      pose.Pos().Y(y);
      pose.Pos().Z(z+wPose[2]);
      pose.Rot() = ignition::math::Quaterniond(roll+wPose[3], pitch+wPose[4], yaw+wPose[5]);

      this->actor->SetWorldPose(pose, false, false);

      double dt = (dist - this->lastDist) / walking_dist_factor * walking_time_factor;
      this->actor->SetScriptTime(this->actor->ScriptTime() + dt);
      this->lastDist = dist;
      Py_DECREF(pRet);
    }
    Py_DECREF(pArgs);
  }

  this->lastUpdate = _info.simTime;

  IGN_PROFILE_END();
}

void PedestrianPlugin::print_pyobject(PyObject* obj) {
  if (PyLong_Check(obj)) {
    // It's a long integer
    long value = PyLong_AsLong(obj);
    RCLCPP_INFO(this->node->get_logger(), "Integer: %ld", value);
  } else if (PyFloat_Check(obj)) {
    // It's a float
    double value = PyFloat_AsDouble(obj);
    RCLCPP_INFO(this->node->get_logger(), "Float: %f", value);
  } else if (PyUnicode_Check(obj)) {
    // It's a Unicode string
    PyObject *tempBytes = PyUnicode_AsEncodedString(obj, "utf-8", "strict");
    if (tempBytes != NULL) {
      char *str = PyBytes_AsString(tempBytes);
      if (str != NULL) {
        RCLCPP_INFO(this->node->get_logger(), "String: %s", str);
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
    RCLCPP_INFO(this->node->get_logger(), "]");
  } else if (PyDict_Check(obj)) {
    // It's a dictionary
    RCLCPP_INFO(this->node->get_logger(), "Dictionary: {");
    PyObject *key, *value;
    Py_ssize_t pos = 0;

    while (PyDict_Next(obj, &pos, &key, &value)) {
      print_pyobject(key);
      RCLCPP_INFO(this->node->get_logger(), ": ");
      print_pyobject(value);
    }
    RCLCPP_INFO(this->node->get_logger(), "}");
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
    RCLCPP_INFO(this->node->get_logger(), ")");
  } else {
    // Object type is not handled in this example
    RCLCPP_INFO(this->node->get_logger(), "Object type not handled in this example.");
  }
}
