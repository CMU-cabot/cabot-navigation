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

#ifndef CABOT_GAZEBO_PEDESTRIAN_PLUGIN_HPP_
#define CABOT_GAZEBO_PEDESTRIAN_PLUGIN_HPP_

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <gazebo_ros/node.hpp>

namespace gazebo
{
class GZ_PLUGIN_VISIBLE PedestrianPlugin : public ModelPlugin
{
public:
  PedestrianPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();

private:
  void OnUpdate(const common::UpdateInfo &_info);
  void print_pyobject(PyObject* obj);
  gazebo_ros::Node::SharedPtr node;
  sdf::ElementPtr sdf;
  physics::ActorPtr actor;
  physics::WorldPtr world;
  std::string module_name;

  PyObject *pModule;
  PyObject *pOnUpdateFunc;

  std::vector<event::ConnectionPtr> connections;
  common::Time lastUpdate;
  double lastDist;
  physics::TrajectoryInfoPtr trajectoryInfo;

  double* get_walking_pose(double distance);
  static double walking_time_factor;
  static double walking_dist_factor;
  static double walking_pose[145][6];
};
}

#endif  // CABOT_GAZEBO_PEDESTRIAN_PLUGIN_HPP_
