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

  this->Reset();
}


void PedestrianPlugin::Reset()
{
  RCLCPP_INFO(this->node->get_logger(), "Reset");

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
  Py_Initialize();

  this->plugin_params.empty();

  sdf::ElementPtr child = this->sdf->GetFirstElement();
  while (child) {
    std::string key = child->GetName();
    if (key == "module") {
      this->module_name = child->Get<std::string>();
      RCLCPP_INFO(this->node->get_logger(), "module name is %s", this->module_name.c_str());
      loader->loadModule(this->module_name);
      child = child->GetNextElement();
      continue;
    }

    sdf::ParamPtr typeAttr = child->GetAttribute("type");
    if (!typeAttr) {
      RCLCPP_ERROR(this->node->get_logger(), "%s' type is not specified", key.c_str());
      child = child->GetNextElement();
      continue;
    }
    std::string type = typeAttr->GetAsString();
    RCLCPP_INFO(this->node->get_logger(), "plugin param %s type=%s", key.c_str(), type.c_str());
    if (type == "str") {
      std::string value = child->Get<std::string>();
      RCLCPP_INFO(this->node->get_logger(), "value is %s", value.c_str());
      this->plugin_params[key] = PyUnicode_FromString(value.c_str());
    } else if (type == "int") {
      int value = child->Get<int>();
      RCLCPP_INFO(this->node->get_logger(), "value is %d", value);
      this->plugin_params[key] = PyLong_FromLong(value);
    } else if (type == "float") {
      double value = child->Get<double>();
      RCLCPP_INFO(this->node->get_logger(), "value is %.2f", value);
      this->plugin_params[key] = PyFloat_FromDouble(value);
    } else if (type == "bool") {
      bool value = child->Get<bool>();
      RCLCPP_INFO(this->node->get_logger(), "value is %d, %s", value, value  ? "true" : "false");
      this->plugin_params[key] = value ? Py_True : Py_False;
    } else {
      RCLCPP_ERROR(this->node->get_logger(), "Unsupported type: %s", type.c_str());
    }
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

double getDictItemAsDouble(PyObject *dict, char *key, double default_value = 0.0) {
  auto obj = PyDict_GetItem(dict, PyUnicode_FromString(key));
  if (obj == NULL) {
    return default_value;
  }
  return PyFloat_AsDouble(obj);
}

void PedestrianPlugin::OnUpdate(const common::UpdateInfo &_info)
{
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
  this->lastUpdate = _info.simTime;

  PyObject* func = loader->getFunc(this->module_name, "onUpdate");

  if (func != NULL) {
    PyObject* pArgs = PyTuple_New(0);
    PyObject* pDict = PyDict_New();

    double *wPose = get_walking_pose(this->dist);
    PyDict_SetItemString(pDict, "dt", PyFloat_FromDouble(dt));
    PyDict_SetItemString(pDict, "x", PyFloat_FromDouble(x));
    PyDict_SetItemString(pDict, "y", PyFloat_FromDouble(y));
    PyDict_SetItemString(pDict, "z", PyFloat_FromDouble(z));
    PyDict_SetItemString(pDict, "roll", PyFloat_FromDouble(roll));
    PyDict_SetItemString(pDict, "pitch", PyFloat_FromDouble(pitch));
    PyDict_SetItemString(pDict, "yaw", PyFloat_FromDouble(yaw));

    // add parameter to the arguments
    for (const auto& pair : this->plugin_params) {
      PyDict_SetItemString(pDict, pair.first.c_str(), pair.second);
    }

    auto pRet = PyObject_Call(func, pArgs, pDict);
    if (pRet != NULL && PyDict_Check(pRet)) {
      auto newX = getDictItemAsDouble(pRet, "x", 0.0);
      auto newY = getDictItemAsDouble(pRet, "y", 0.0);
      auto newZ = getDictItemAsDouble(pRet, "z", 0.0);
      auto newRoll = getDictItemAsDouble(pRet, "roll", 0.0);
      auto newPitch = getDictItemAsDouble(pRet, "pitch", 0.0);
      auto newYaw = getDictItemAsDouble(pRet, "yaw", 0.0);
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
  
      Py_XDECREF(pRet);
    }
    Py_XDECREF(pArgs);
  }

  IGN_PROFILE_END();
}

// debug function
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
