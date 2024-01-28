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

#include "pedestrian_plugin/pedestrian_plugin_manager.hpp"

using namespace gazebo;

// exporting ros python module
PyObject* ros_info(PyObject *self, PyObject *args)
{
  const char* message;
  if (PyArg_ParseTuple(args, "s", &message)) {
    RCLCPP_INFO(PedestrianPluginManager::getInstance().get_logger(), message);
  }
  Py_RETURN_NONE;
}

PyMethodDef RosMethods[] = {
  {"info", ros_info, METH_VARARGS,
   "call RCLCPP_INFO"},
  {NULL, NULL, 0, NULL}
};

PyModuleDef RosModule = {
  PyModuleDef_HEAD_INIT, "ros", NULL, -1, RosMethods,
  NULL, NULL, NULL, NULL
};

PyObject* PyInit_ros(void)
{
  return PyModule_Create(&RosModule);
}


PedestrianPluginManager::PedestrianPluginManager()
    : node_(gazebo_ros::Node::Get())
{
  people_pub_ = node_->create_publisher<people_msgs::msg::People>("/people", 10);
}

PedestrianPluginManager::~PedestrianPluginManager() {
}

size_t PedestrianPluginManager::addActor(sdf::ElementPtr sdf) {
  actors_.push_back(sdf);
  return actors_.size();
}

void PedestrianPluginManager::publishPeopleIfReady() {
  if (peopleMap_.size() == actors_.size()) {
    people_msgs::msg::People msg;
    for (auto it : peopleMap_) {
      msg.people.push_back(it.second);
    }
    msg.header.frame_id="map_global";
    people_pub_->publish(msg);
    peopleMap_.clear();
  }
}

void PedestrianPluginManager::addPersonMessage(people_msgs::msg::Person person) {
  peopleMap_.insert({person.name, person});
}


rclcpp::Logger PedestrianPluginManager::get_logger() {
  return node_->get_logger();
}
