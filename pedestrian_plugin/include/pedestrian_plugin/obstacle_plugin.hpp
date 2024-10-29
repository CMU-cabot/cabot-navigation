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

#ifndef PEDESTRIAN_PLUGIN__OBSTACLE_PLUGIN_HPP_
#define PEDESTRIAN_PLUGIN__OBSTACLE_PLUGIN_HPP_

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <mutex>
#include <string>
#include <vector>

#include <gazebo_ros/node.hpp>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "pedestrian_plugin/obstacle_plugin_manager.hpp"

namespace gazebo
{

class GZ_PLUGIN_VISIBLE ObstaclePlugin : public ModelPlugin
{
public:
  ObstaclePlugin();
  ~ObstaclePlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  void apply_parameters();
  void update_parameters(ObstaclePluginParams params);

private:
  void OnUpdate(const common::UpdateInfo & _info);
  void print_pyobject(PyObject * obj);
  sdf::ElementPtr sdf;
  // physics::ActorPtr actor;
  physics::ModelPtr model;
  physics::WorldPtr world;
  std::string name;
  std::string module_name;
  gazebo::physics::ModelPtr robotModel;

  std::vector<event::ConnectionPtr> connections;
  common::Time lastUpdate;
  physics::TrajectoryInfoPtr trajectoryInfo;

  ObstaclePluginManager & manager;

  // manage the obstacle location by the plugin
  // because animation can change its position
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  double dist;
  double width; //  x axis length
  double height; //  y axis length
  double depth; //  z axis length
  int obstacle_id;

  ObstaclePluginParams plugin_params;
  bool needs_to_apply_params;

  // Need to be separated
  static double walking_time_factor;
  static double walking_dist_factor;
  static double walking_pose[145][6];
};
}  // namespace gazebo

#endif  // PEDESTRIAN_PLUGIN__OBSTACLE_PLUGIN_HPP_
