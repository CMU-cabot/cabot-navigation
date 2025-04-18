// Copyright (c) 2022  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "cabot_navigation2/cabot_planner_param.hpp"

#include <mutex>

namespace cabot_navigation2
{

CaBotPlan::CaBotPlan(CaBotPlannerParam & param_, DetourMode detour_mode_)
: param(param_),
  detour_mode(detour_mode_)
{
  nodes_backup = param.getNodes(detour_mode);
  RCLCPP_INFO(logger_, "nodes_backup.size = %ld, detour_mode=%d", nodes_backup.size(), detour_mode);
  resetNodes();
  findIndex();
}

void CaBotPlan::resetNodes()
{
  nodes.clear();
  for (uint64_t i = 0; i < nodes_backup.size(); i++) {
    nodes_backup[i].reset();
    nodes.push_back(nodes_backup[i]);
  }
}

nav_msgs::msg::Path CaBotPlan::getPlan(bool normalized, float normalize_length)
{
  nav_msgs::msg::Path ret;
  ret.header.frame_id = "map";

  if (nodes.size() == 0) {
    return ret;
  }

  if (normalized == false) {
    auto mp0 = nodes[0];
    for (uint64_t i = 1; i < nodes.size(); i++) {
      auto mp1 = nodes[i];
      float dy = mp1.y - mp0.y;
      float dx = mp1.x - mp0.x;
      if (dx == 0 && dy == 0) {continue;}
      float yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);

      float mx, my;
      param.mapToWorld(mp0.x + 0.5, mp0.y + 0.5, mx, my);

      geometry_msgs::msg::PoseStamped wp;
      wp.pose.position.x = mx;
      wp.pose.position.y = my;
      wp.pose.orientation.x = q.x();
      wp.pose.orientation.y = q.y();
      wp.pose.orientation.z = q.z();
      wp.pose.orientation.w = q.w();
      ret.poses.push_back(wp);

      mp0 = mp1;
    }
  } else {
    auto mp0 = nodes[0];
    for (uint64_t i = 1; i < nodes.size(); i++) {
      auto mp1 = nodes[i];
      float dy = mp1.y - mp0.y;
      float dx = mp1.x - mp0.x;
      if (dx == 0 && dy == 0) {continue;}
      float yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);

      float mx, my;
      param.mapToWorld(mp0.x + 0.5, mp0.y + 0.5, mx, my);

      geometry_msgs::msg::PoseStamped wp;
      wp.pose.position.x = mx;
      wp.pose.position.y = my;
      wp.pose.orientation.x = q.x();
      wp.pose.orientation.y = q.y();
      wp.pose.orientation.z = q.z();
      wp.pose.orientation.w = q.w();
      ret.poses.push_back(wp);

      float dist = std::hypot(dx, dy);
      if (dist > normalize_length / param.resolution) {
        mp0 = Node(mp0.x + std::cos(yaw) * normalize_length / param.resolution, mp0.y + std::sin(yaw) * normalize_length / param.resolution);
        i--;
        continue;
      }

      mp0 = mp1;
    }
  }
  return ret;
}

void CaBotPlan::findIndex()
{
  // find start/end index around the start
  float mx, my;
  param.worldToMap(param.start.pose.position.x, param.start.pose.position.y, mx, my);
  Node start_node(mx, my);
  start_index = 0;
  end_index = ULLONG_MAX;
  float min_dist = 1000;
  float optimize_distance_from_start = param.options.optimize_distance_from_start / param.resolution;
  float distance_from_start = 0;
  for (uint64_t i = 0; i < nodes.size() - 1; i++) {
    Node * n0 = &nodes[i];
    if (start_node.distance(*n0) < min_dist) {
      start_index = i;
      min_dist = start_node.distance(*n0);
    }
  }

  double end_distance = optimize_distance_from_start;
  for (uint64_t i = start_index; i < nodes.size() - 1; i++) {
    Node * n0 = &nodes[i];
    Node * n1 = &nodes[i + 1];
    distance_from_start += n0->distance(*n1);
    end_index = i + 2;
    // check optimized sitance and also the last node is okay to go through
    RCLCPP_DEBUG(logger_, "end_distance=%.2f, distance_from_start=%.2f", end_distance, distance_from_start);
    if (end_distance < distance_from_start) {
      if (param.checkPointIsOkay(*n1, detour_mode)) {
        break;
      } else {
        end_distance = distance_from_start + (2.0 / param.resolution);  // margin
      }
    }
  }
  if (nodes.size() < end_index) {
    end_index = nodes.size();
  }
  /*
  if (start_index < (uint64_t)(5.0/options_.initial_node_interval)) {
    start_index = 0;
  } else {
    start_index = start_index - (int)(5.0/options_.initial_node_interval);
  }
  */
  RCLCPP_INFO(param.logger, "start_index=%ld, end_index=%ld, size=%ld", start_index, end_index, nodes.size());
}

void CaBotPlan::adjustNodeInterval()
{
  bool changed = false;
  int devide_link_cell_interval_threshold = param.options.initial_node_interval_scale * 2.0;
  int shrink_link_cell_interval_threshold = param.options.initial_node_interval_scale / 2.0;
  for (uint64_t i = 0; i < nodes.size() - 2; i++) {
    if (nodes_backup.size() * 2 < nodes.size()) {
      break;
    }
    Node * n0 = &nodes[i];
    Node * n1 = &nodes[i + 1];

    auto distance = n0->distance(*n1);
    if (distance > devide_link_cell_interval_threshold) {
      Node newNode;
      newNode.x = (n0->x + n1->x) / 2.0;
      newNode.y = (n0->y + n1->y) / 2.0;
      newNode.anchor.x = (n0->anchor.x + n1->anchor.y) / 2.0;
      newNode.anchor.y = (n0->anchor.y + n1->anchor.y) / 2.0;
      newNode.angle = n0->angle;
      // RCLCPP_INFO(param.logger, "newNode2 [%ld] x = %.2f, y = %.2f", i + 1, newNode.x, newNode.y);
      if (i < start_index) {
        start_index++;
        changed = true;
      }
      if (i < end_index) {
        end_index++;
        changed = true;
      }
      i++;
      nodes.insert(nodes.begin() + i, newNode);
    }
    if (distance < shrink_link_cell_interval_threshold) {
      nodes.erase(nodes.begin() + i);
      if (i < start_index) {
        start_index--;
        changed = true;
      }
      if (i < end_index) {
        end_index--;
        changed = true;
      }
      i--;
    }
  }
  // if (changed) {
  // RCLCPP_INFO(param.logger, "start_index=%ld, end_index=%ld", start_index, end_index);
  // }
}

float CaBotPlan::length()
{
  float total = 0;
  for (uint64_t i = 0; i < nodes.size() - 1; i++) {
    Node * n0 = &nodes[i];
    Node * n1 = &nodes[i + 1];
    total += n0->distance(*n1);
  }
  return total;
}

std::vector<Node> CaBotPlan::getTargetNodes()
{
  std::vector<Node> temp;
  for (uint64_t i = start_index; i < end_index; i++) {
    temp.push_back(nodes[i]);
  }
  return temp;
}


bool CaBotPlan::checkGoAround()
{
  float go_around_detect_threshold = param.options.go_around_detect_threshold;

  // check if the path makes a round
  float total_yaw_diff = 0;
  Node * n0 = &nodes[0];
  Node * n1 = &nodes[1];
  float prev_yaw = (*n1 - *n0).yaw();
  n0 = n1;
  for (uint64_t i = 2; i < nodes.size(); i++) {
    n1 = &nodes[i];
    float current_yaw = (*n1 - *n0).yaw();
    auto diff = normalized_diff(current_yaw, prev_yaw);
    total_yaw_diff += diff;
    // RCLCPP_INFO(logger_, "total yaw %.2f, diff %.2f, (%.2f, %.2f)",
    //            total_yaw_diff, diff, current_yaw, prev_yaw);
    n0 = n1;
    prev_yaw = current_yaw;
    if (std::abs(total_yaw_diff) > go_around_detect_threshold) {
      return true;
    }
  }
  return false;
}


bool CaBotPlan::checkPathIsOkay()
{
  for (uint64_t i = start_index; (i < nodes.size() - 1) && (i < end_index - 1); i++) {
    auto n0 = nodes[i];
    auto n1 = nodes[i + 1];

    int N = ceil(n0.distance(n1) / (param.options.initial_node_interval_scale * param.resolution));
    for (int j = 0; j < 1; j++) {
      Point temp((n0.x * j + n1.x * (N - j)) / N, (n0.y * j + n1.y * (N - j)) / N);

      if (!param.checkPointIsOkay(temp, detour_mode)) {
        RCLCPP_ERROR(logger_, "something wrong (%.2f, %.2f), (%.2f, %.2f), %d", n0.x, n0.y, n1.x, n1.y, N);
        return false;
      }
    }
  }
  return true;
}


///////////////////////////

CaBotPlannerParam::CaBotPlannerParam(
  CaBotPlannerOptions & options_,
  PathEstimateOptions & pe_options_,
  const geometry_msgs::msg::PoseStamped & start_,
  const geometry_msgs::msg::PoseStamped & goal_,
  nav_msgs::msg::Path navcog_path_,
  people_msgs::msg::People::SharedPtr people_msg_ptr_,
  people_msgs::msg::People::SharedPtr obstacles_msg_ptr_,
  queue_msgs::msg::Queue::SharedPtr queue_msg_ptr_,
  nav2_costmap_2d::Costmap2D * costmap_,
  nav2_costmap_2d::Costmap2D * static_costmap_
)
: options(options_),
  pe_options(pe_options_),
  start(start_),
  goal(goal_),
  navcog_path(navcog_path_),
  costmap(costmap_),
  static_costmap(static_costmap_),
  cost(nullptr),
  static_cost(nullptr),
  mark(nullptr),
  data_(nullptr),
  data_non_collision_(nullptr),
  idx_(nullptr),
  idx_non_collision_(nullptr)
{
  if (people_msg_ptr_ != nullptr) {people_msg = *people_msg_ptr_;}
  if (obstacles_msg_ptr_ != nullptr) {obstacles_msg = *obstacles_msg_ptr_;}
  if (queue_msg_ptr_ != nullptr) {queue_msg = *queue_msg_ptr_;}

  assert(costmap->getSizeInCellsX() == static_costmap->getSizeInCellsX());
  assert(costmap->getSizeInCellsY() == static_costmap->getSizeInCellsY());

  width = costmap->getSizeInCellsX();
  height = costmap->getSizeInCellsY();
  resolution = costmap->getResolution();
  origin_x = costmap->getOriginX();
  origin_y = costmap->getOriginY();

  allocate();
  // get cost from costmap
  //  1. static map only
  //  2. layered cost (all layers)
  // then remove cost around moving people/obstacle
  setCost();
}

CaBotPlannerParam::~CaBotPlannerParam()
{
  deallocate();
}


void CaBotPlannerParam::allocate()
{
  auto size = width * height;
  cost = new unsigned char[size];
  static_cost = new unsigned char[size];
  mark = new uint16_t[size];
}

void CaBotPlannerParam::deallocate()
{
  if (cost != nullptr) {delete cost;}
  if (static_cost != nullptr) {delete static_cost;}
  if (mark != nullptr) {delete mark;}
  if (data_) {delete data_;}
  if (data_non_collision_) {delete data_non_collision_;}
  if (idx_) {delete idx_;}
  if (idx_non_collision_) {delete idx_non_collision_;}
}


void CaBotPlannerParam::setCost()
{
  unsigned char * cost_ = costmap->getCharMap();
  unsigned char * static_cost_ = static_costmap->getCharMap();
  unsigned char * p1 = cost;
  unsigned char * p2 = static_cost;
  uint16_t * p3 = mark;

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      *p1++ = *cost_++;
      *p2++ = *static_cost_++;
      *p3++ = 0;
    }
  }

  // ignore moving people/obstacles/queue people
  for (auto it = people_msg.people.begin(); it != people_msg.people.end(); it++) {
    bool stationary = (std::find(it->tags.begin(), it->tags.end(), "stationary") != it->tags.end());
    if (!stationary || options.ignore_people) {
      clearCostAround(*it);
      RCLCPP_INFO(logger, "setCost: ignore a person %s in frame %s", it->name.c_str(), people_msg.header.frame_id.c_str());
    } else {
      RCLCPP_INFO(logger, "setCost: consider a person %s in frame %s", it->name.c_str(), people_msg.header.frame_id.c_str());
    }
  }
  for (auto it = obstacles_msg.people.begin(); it != obstacles_msg.people.end(); it++) {
    bool stationary = (std::find(it->tags.begin(), it->tags.end(), "stationary") != it->tags.end());
    if (!stationary) {
      clearCostAround(*it);
    }
  }
  for (auto it = queue_msg.people.begin(); it != queue_msg.people.end(); it++) {
    clearCostAround(*it);
  }

  int count = 0;
  int scount = 0;
  for (int i = 0; i < width * height; i++) {
    if (cost[i] > 0) {count++;}
    if (static_cost[i] > 0) {scount++;}
  }
  RCLCPP_INFO(logger, "cost non zero count = %d, static cost non zero count = %d", count, scount);
}

bool CaBotPlannerParam::adjustPath()
{
  path = normalizedPath(navcog_path);
  if (path.poses.empty()) {return false;}
  RCLCPP_INFO(logger, "adjustPath, path.poses.size() = %ld", path.poses.size());

  estimatePathWidthAndAdjust(path, costmap, pe_options);
  if (options.adjust_start) {
    RCLCPP_INFO(logger, "adjust_start, path.poses.size() = %ld, start = (%.2f, %.2f)", path.poses.size(), start.pose.position.x, start.pose.position.y);
    path = adjustedPathByStart(path, start);
    RCLCPP_INFO(logger, "adjust_start, path.poses.size() = %ld", path.poses.size());
  }

  // path.poses.push_back(goal);
  return true;
}

void CaBotPlannerParam::clearCostAround(people_msgs::msg::Person & person)
{
  int max_obstacle_scan_distance_cell = options.max_obstacle_scan_distance / resolution;

  float mx, my;
  if (!worldToMap(person.position.x, person.position.y, mx, my)) {
    RCLCPP_INFO(logger, "person position is out of map (%.2f, %.2f)", person.position.x, person.position.y);
    return;
  }
  RCLCPP_INFO(logger, "clearCostAround %.2f %.2f \n %s", mx, my, rosidl_generator_traits::to_yaml(person).c_str());

  ObstacleGroup group;
  scanObstacleAt(group, mx, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, max_obstacle_scan_distance_cell);

  RCLCPP_INFO(logger, "found obstacles %ld", group.obstacles_.size());
  for (auto obstacle = group.obstacles_.begin(); obstacle != group.obstacles_.end(); obstacle++) {
    cost[obstacle->index] = 0;
  }
}


void CaBotPlannerParam::scanObstacleAt(ObstacleGroup & group, float mx, float my, unsigned int min_obstacle_cost, float max_dist)
{
  std::queue<std::pair<Obstacle, float>> queue;
  int index = getIndex(mx, my);
  if (index < 0) {return;}
  queue.push(std::pair<Obstacle, int>(Obstacle(mx, my, index), 0));

  while (queue.size() > 0) {
    auto entry = queue.front();
    auto temp = entry.first;
    queue.pop();
    // RCLCPP_INFO(logger, "queue.size=%ld, cost=%d, static_cost=%d, mark=%d, (%.2f %.2f) [%d], %d",
    //             queue.size(), cost[temp.index], static_cost[temp.index], mark[temp.index], temp.x, temp.y, temp.index, entry.second);
    if (entry.second > max_dist) {
      continue;
    }
    if (mark[temp.index] == 65535) {
      continue;
    }
    mark[temp.index] = 65535;
    if (cost[temp.index] < min_obstacle_cost || static_cost[temp.index] >= min_obstacle_cost) {
      continue;
    }
    auto obstacle = Obstacle(temp.x, temp.y, temp.index, false, cost[temp.index], temp.size, true);
    group.add(obstacle);

    index = getIndex(temp.x + 1, temp.y);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x + 1, temp.y, index), entry.second + 1));
    }
    index = getIndex(temp.x + 1, temp.y + 1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x + 1, temp.y + 1, index), entry.second + sqrt(2)));
    }
    index = getIndex(temp.x, temp.y + 1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x, temp.y + 1, index), entry.second + 1));
    }
    index = getIndex(temp.x - 1, temp.y + 1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x - 1, temp.y + 1, index), entry.second + sqrt(2)));
    }
    index = getIndex(temp.x - 1, temp.y);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x - 1, temp.y, index), entry.second + 1));
    }
    index = getIndex(temp.x - 1, temp.y - 1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x - 1, temp.y - 1, index), entry.second + sqrt(2)));
    }
    index = getIndex(temp.x, temp.y - 1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x, temp.y - 1, index), entry.second + 1));
    }
    index = getIndex(temp.x + 1, temp.y - 1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x + 1, temp.y - 1, index), entry.second + sqrt(2)));
    }
  }
}

bool CaBotPlannerParam::worldToMap(float wx, float wy, float & mx, float & my) const
{
  auto origin_x = costmap->getOriginX();
  auto origin_y = costmap->getOriginY();
  auto resolution = costmap->getResolution();
  auto width = costmap->getSizeInCellsX();
  auto height = costmap->getSizeInCellsY();
  mx = (wx - origin_x) / resolution;
  my = (wy - origin_y) / resolution;

  if (0 <= mx && mx < width && 0 <= my && my < height) {
    return true;
  }
  RCLCPP_ERROR(logger, "worldToMap out of bound origin (%.2f, %.2f) resolution=%.2f (%d, %d) - (%.2f, %.2f)", origin_x, origin_y, resolution, width, height, mx, my);
  return false;
}

void CaBotPlannerParam::mapToWorld(float mx, float my, float & wx, float & wy) const
{
  auto origin_x = costmap->getOriginX();
  auto origin_y = costmap->getOriginY();
  auto resolution = costmap->getResolution();
  wx = origin_x + mx * resolution;
  wy = origin_y + my * resolution;
}
int CaBotPlannerParam::getIndex(float x, float y) const
{
  int width = costmap->getSizeInCellsX();
  int height = costmap->getSizeInCellsY();
  int ix = static_cast<int>(x);
  int iy = static_cast<int>(y);
  if (ix < 0 || iy < 0 || width <= ix || height <= iy) {
    return -(ix + iy * width);
  }
  return std::max(0, std::min(ix + iy * width, width * height - 1));
}

int CaBotPlannerParam::getIndexByPoint(Point & p) const
{
  return getIndex(p.x + 0.5, p.y + 0.5);
}

std::vector<Node> CaBotPlannerParam::getNodes(DetourMode detour_mode) const
{
  std::vector<Node> nodes;
  auto initial_node_interval = options.initial_node_interval_scale * resolution;

  // push initial pose
  auto p0 = path.poses[0].pose.position;
  auto p1 = path.poses[0].pose.position;
  float mx = 0, my = 0;
  for (uint64_t i = 1; i < path.poses.size(); i++) {
    worldToMap(p0.x, p0.y, mx, my);
    nodes.push_back(Node(mx, my));
    p1 = path.poses[i].pose.position;

    auto dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
    int N = std::round(dist / initial_node_interval);
    if (N == 0) {continue;}

    for (int j = 1; j <= N; j++) {
      // deal with the last pose
      if (j == N && i < path.poses.size() - 1) {
        continue;
      }
      worldToMap((p0.x * (N - j) + p1.x * j) / N, (p0.y * (N - j) + p1.y * j) / N, mx, my);
      Node temp = Node(mx, my);
      temp.fixed = (j == 0) || (j == N);
      nodes.push_back(temp);
    }
    p0 = p1;
  }
  worldToMap(p1.x, p1.y, mx, my);
  Node temp = Node(mx, my);
  temp.fixed = true;
  nodes.push_back(temp);

  // check if the goal (last node) is on lethal area and remove it until it can be reached
  Node * prev = nullptr;
  double dist = 0;
  do {
    if (!checkPointIsOkay(nodes.back(), detour_mode) && nodes.size() > 2) {
      if (prev) {
        dist += prev->distance(nodes.back()) * resolution;
      }
      prev = &nodes.back();
      nodes.pop_back();
      RCLCPP_WARN(logger, "remove last node, dist=%.2f, size=%ld", dist, nodes.size());
    } else {
      break;
    }
  } while (true);

  return nodes;
}


bool isEdge(unsigned char * cost, int width, int height, int index, int target_cost)
{
  int max_size = width * height;
  if (index < 0 || max_size <= index) {return false;}
  unsigned char cost_center = cost[index];
  unsigned char cost_left = target_cost;
  unsigned char cost_right = target_cost;
  unsigned char cost_up = target_cost;
  unsigned char cost_down = target_cost;
  if (0 <= index - 1) {cost_left = cost[index - 1];}
  if (index + 1 < max_size) {cost_right = cost[index + 1];}
  if (0 <= index - width) {cost_up = cost[index - width];}
  if (index + width < max_size) {cost_down = cost[index + width];}
  return cost_center == target_cost && (cost_left < target_cost || cost_up < target_cost || cost_right < target_cost || cost_down < target_cost);
}

/*
findObstacles scans the costmap to determine obstacles
  Obstacle class: point where the cost is higher than threthold
  ObstacleGroup class: points that all in a single adjucented region

1. find static obstacles near the path
2. find dynamic obstacles near the path
3. create flann index for obstacles (not group obstacles) and point cloud for debug
*/
void CaBotPlannerParam::findObstacles(std::vector<Node> nodes)
{
  // check obstacles only from current position to N meters forward
  int max_obstacle_scan_distance_cell = options.max_obstacle_scan_distance / resolution;
  float optimize_distance_from_start = options.optimize_distance_from_start / resolution;
  RCLCPP_INFO(logger, "nodes.size = %ld", nodes.size());

  groups.clear();
  obstacles.clear();

  // find max index of nodes which does not exceed optimize_distance_from_start
  uint64_t end_index = nodes.size();
  float distance_from_start = 0;
  for (uint64_t i = 0; i < nodes.size() - 1; i++) {
    Node * n0 = &nodes[i];
    Node * n1 = &nodes[i + 1];
    distance_from_start += n0->distance(*n1);
    if (distance_from_start > optimize_distance_from_start) {
      end_index = i + 1;
      break;
    }
  }

  // 1. find static obstacles near the path
  {
    memset(mark, 0, width * height * sizeof(uint16_t));
    std::vector<int> marks;
    for (uint64_t i = 0; i < end_index; i++) {
      int index = getIndexByPoint(nodes[i]);
      if (index < 0) {
        continue;
      }
      mark[index] = 1;
      marks.push_back(index);
    }
    uint64_t i = 0;
    int64_t max_size = width * height;
    std::set<Obstacle> staticObstacleSet;
    while (i < marks.size()) {
      int64_t index = marks[i++];
      float x = index % width;
      float y = index / width;

      unsigned char static_cost_value = static_cost[index];
      uint16_t current = mark[index];

      if (current > max_obstacle_scan_distance_cell) {continue;}

      if (static_cost_value >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        if (isEdge(static_cost, width, height, index, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
          staticObstacleSet.insert(Obstacle(x, y, index, true, static_cost_value));
        }
        if (isEdge(static_cost, width, height, index, nav2_costmap_2d::LETHAL_OBSTACLE)) {
          staticObstacleSet.insert(Obstacle(x, y, index, true, static_cost_value));
        }
      }

      if (0 <= index - 1 && mark[index - 1] == 0) {
        mark[index - 1] = current + 1;
        marks.push_back(index - 1);
      }
      if (index + 1 < max_size && mark[index + 1] == 0) {
        mark[index + 1] = current + 1;
        marks.push_back(index + 1);
      }
      if (0 <= index - width && mark[index - width] == 0) {
        mark[index - width] = current + 1;
        marks.push_back(index - width);
      }
      if (index + width < max_size && mark[index + width] == 0) {
        mark[index + width] = current + 1;
        marks.push_back(index + width);
      }
    }

    RCLCPP_INFO(logger, "staticObstacleSet size = %ld", staticObstacleSet.size());
    obstacles.insert(obstacles.end(), staticObstacleSet.begin(), staticObstacleSet.end());
  }

  // 2. find dynamic obstacles near the path
  {
    memset(mark, 0, width * height * sizeof(uint16_t));
    std::vector<int> marks;
    uint64_t start_node_index = 0;

    for (uint64_t i = 0; i < end_index; i++) {
      int index = getIndexByPoint(nodes[i]);
      if (index < 0) {
        continue;
      }
      if (mark[index] == 65535) {
        continue;
      }
      auto cost_value = cost[index];
      mark[index] = 1;
      marks.push_back(index);
      if (cost_value >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        ObstacleGroup group;
        scanObstacleAt(group, nodes[i].x, nodes[i].y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, max_obstacle_scan_distance_cell);
        if (group.complete()) {
          group.index = getIndexByPoint(group);
          group.collision = true;
          RCLCPP_DEBUG(logger, "Group Obstacle %.2f %.2f %.2f %ld", group.x, group.y, group.size, group.obstacles_.size());
          groups.push_back(group);
        }
      }
    }

    uint64_t i = 0;
    int64_t max_size = width * height;
    std::set<Obstacle> obstacleSet;
    while (i < marks.size()) {
      int64_t index = marks[i++];
      float x = index % width;
      float y = index / width;

      unsigned char cost_value = cost[index];
      unsigned char static_cost_value = static_cost[index];
      uint16_t current = mark[index];

      if (current > max_obstacle_scan_distance_cell) {continue;}

      if (static_cost_value < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        if (cost_value >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          if (isEdge(cost, width, height, index, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
            obstacleSet.insert(Obstacle(x, y, index, true, cost_value));
          }
          if (isEdge(cost, width, height, index, nav2_costmap_2d::LETHAL_OBSTACLE)) {
            obstacleSet.insert(Obstacle(x, y, index, true, cost_value));
          }
        }
      }

      if (0 <= index - 1 && mark[index - 1] == 0) {
        mark[index - 1] = current + 1;
        marks.push_back(index - 1);
      }
      if (index + 1 < max_size && mark[index + 1] == 0) {
        mark[index + 1] = current + 1;
        marks.push_back(index + 1);
      }
      if (0 <= index - width && mark[index - width] == 0) {
        mark[index - width] = current + 1;
        marks.push_back(index - width);
      }
      if (index + width < max_size && mark[index + width] == 0) {
        mark[index + width] = current + 1;
        marks.push_back(index + width);
      }
    }

    RCLCPP_INFO(logger, "obstacleSet size = %ld", obstacleSet.size());
    obstacles.insert(obstacles.end(), obstacleSet.begin(), obstacleSet.end());

    // find corresponding lethal obstacle
    for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
      if (it->cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {continue;}
      double min = 1000;
      Obstacle * nearest;
      for (auto it2 = obstacles.begin(); it2 != obstacles.end(); ++it2) {
        if (it2->cost != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
        if (it->is_static != it2->is_static) {continue;}
        double dist = it2->distance(*it);
        if (dist < min) {
          min = dist;
          nearest = &(*it2);
        }
      }
      it->lethal = nearest;
    }

    // merging nearby obstacle groups
    bool flag = false;
    do {
      flag = false;
      if (groups.size() > 1) {
        for (uint64_t i = 0; i < groups.size() - 1; i++) {
          if (groups.back().distance(groups.at(i)) < 5) {
            groups.at(i).combine(groups.back());
            groups.pop_back();
            flag = true;
          }
        }
      }
    } while(flag);
  }

  // 4. create flann index for obstacles (not group obstacles) and point cloud for debug
  uint64_t n = 0;
  uint64_t n_collision = 0;
  for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++) {
    n++;
    if (obstacle->in_group) {
      n_collision++;
    }
  }
  if (idx_) {
    delete idx_;
    idx_ = nullptr;
  }
  if (idx_non_collision_) {
    delete idx_non_collision_;
    idx_non_collision_ = nullptr;
  }
  if (data_) {
    delete data_;
    data_ = nullptr;
  }
  if (data_non_collision_) {
    delete data_non_collision_;
    data_non_collision_ = nullptr;
  }
  RCLCPP_INFO(logger, "data length=%ld, data non collision length=%ld", n, n - n_collision);
  if (n == 0 || n - n_collision == 0) {
    return;
  }
  data_ = new cv::Mat(n, 2, CV_32FC1);
  data_non_collision_ = new cv::Mat(n - n_collision, 2, CV_32FC1);
  olist_.clear();
  olist_non_collision_.clear();
  uint64_t i = 0;
  uint64_t j = 0;
  // add index

  for (auto oit = obstacles.begin(); oit != obstacles.end(); ++oit) {
    olist_.push_back(*oit);
    data_->at<float>(i, 0) = oit->x;
    data_->at<float>(i, 1) = oit->y;
    i++;
    if (!oit->in_group) {
      olist_non_collision_.push_back(*oit);
      data_non_collision_->at<float>(j, 0) = oit->x;
      data_non_collision_->at<float>(j, 1) = oit->y;
      j++;
    }
  }
  RCLCPP_DEBUG(logger, "making index %ld/%ld", i, n);
  idx_ = new cv::flann::Index(*data_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  RCLCPP_DEBUG(logger, "making index %ld/%ld", j, n - n_collision);
  idx_non_collision_ = new cv::flann::Index(*data_non_collision_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  RCLCPP_DEBUG(logger, "obstacles = %ld", obstacles.size());
}

std::vector<Obstacle> CaBotPlannerParam::getObstaclesNearPoint(const Point & node, bool collision) const
{
  int kdtree_search_radius_cell = options.kdtree_search_radius / resolution;
  int kdtree_max_results = options.kdtree_max_results;

  std::vector<Obstacle> list;
  cv::flann::Index * idx = collision ? idx_non_collision_ : idx_;
  if (!idx || !data_) {
    return list;
  }

  cv::Mat query = cv::Mat::zeros(1, 2, CV_32FC1);
  query.at<float>(0) = node.x;
  query.at<float>(1) = node.y;
  std::vector<int> indices;
  std::vector<float> dists;

  int m = idx->radiusSearch(query, indices, dists, kdtree_search_radius_cell, kdtree_max_results);
  for (int i = 0; i < m; i++) {
    if (collision) {
      list.push_back(olist_non_collision_[indices[i]]);
    } else {
      list.push_back(olist_[indices[i]]);
    }
  }
  return list;
}

bool CaBotPlannerParam::checkPointIsOkay(Point & point, DetourMode detour_mode) const
{
  int index = getIndexByPoint(point);

  if (width * height <= index) {
    RCLCPP_WARN(
      logger, "index is out of bound %ld, width=%ld, height=%ld, size=%ld, point=(%.2f, %.2f)",
      index, width, height, width * height, point.x, point.y);
    return false;
  }


  if (detour_mode == DetourMode::IGNORE) {
    if (index >= 0 && static_cost[index] >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      float mx, my;
      mapToWorld(point.x, point.y, mx, my);
      RCLCPP_WARN(
        logger, "ignore mode: path above threshold at (%.2f, %.2f)", mx, my);
      return false;
    }
  } else {
    if (index >= 0 && cost[index] >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      float mx, my;
      mapToWorld(point.x, point.y, mx, my);
      RCLCPP_WARN(
        logger, "path above threshold at (%.2f, %.2f)", mx, my);
      return false;
    }
  }
  return true;
}
}  // namespace cabot_navigation2
