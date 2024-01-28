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

#ifndef PEDESTRIAN_PLUGIN_PEDESTRIAN_PLUGIN_MANAGER_HPP_
#define PEDESTRIAN_PLUGIN_PEDESTRIAN_PLUGIN_MANAGER_HPP_
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <people_msgs/msg/people.hpp>
#include <gazebo_ros/node.hpp>

PyObject* PyInit_ros(void);

namespace gazebo {

class PedestrianPluginManager {
 public:
  static PedestrianPluginManager& getInstance() {
      static PedestrianPluginManager instance; // Guaranteed to be destroyed and instantiated on first use
      return instance;
  }  
  PedestrianPluginManager();
  ~PedestrianPluginManager();
  size_t addActor(sdf::ElementPtr sdf);
  void publishPeopleIfReady();
  void addPersonMessage(people_msgs::msg::Person person);
  rclcpp::Logger get_logger();

  std::mutex mtx;

 private:
  gazebo_ros::Node::SharedPtr node_;
  std::vector<sdf::ElementPtr> actors_;
  std::map<std::string, people_msgs::msg::Person> peopleMap_;
  rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pub_;
  
  
};

}

#endif  // PEDESTRIAN_PLUGIN_PEDESTRIAN_PLUGIN_MANAGER_HPP_
