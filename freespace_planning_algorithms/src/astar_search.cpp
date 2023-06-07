// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "freespace_planning_algorithms/astar_search.hpp"
#include <vector>

namespace freespace_planning_algorithms
{
double calcReedsSheppDistance(
  const Pose & p1, const Pose & p2, double radius)
{
  auto rs_space = ReedsSheppStateSpace(radius);
  ReedsSheppStateSpace::StateXYT pose0{p1.position.x, p1.position.y, p1.yaw};
  ReedsSheppStateSpace::StateXYT pose1{p2.position.x, p2.position.y, p2.yaw};
  return rs_space.distance(pose0, pose1);
}


Pose calcRelativePose(
  const Pose & base_pose, const Pose & pose)
{
  Pose relative_pose;
  double deita_x = pose.position.x - base_pose.position.x;
  double deita_y = pose.position.y - base_pose.position.y;
  relative_pose.position.x = deita_x * cos(base_pose.yaw) + deita_y * sin(base_pose.yaw);
  relative_pose.position.y = -deita_x * sin(base_pose.yaw) + deita_y * cos(base_pose.yaw);
  relative_pose.yaw = pose.yaw - base_pose.yaw;
  return relative_pose;
}

Pose node2pose(const AstarNode & node)
{
  Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.yaw = node.theta;

  return pose_local;
}

AstarSearch::TransitionTable createTransitionTable(
  const double minimum_turning_radius, const double maximum_turning_radius,
  const int turning_radius_size, const double theta_size, const bool use_back)
{
  // Vehicle moving for each angle
  AstarSearch::TransitionTable transition_table;
  transition_table.resize(theta_size);

  const double dtheta = 2.0 * M_PI / theta_size;

  // Minimum moving distance with one state update
  // arc  = r * theta
  const auto & R_min = minimum_turning_radius;
  const auto & R_max = maximum_turning_radius;
  const double step_min = R_min * dtheta;
  const double dR = (R_max - R_min) / turning_radius_size;

  // NodeUpdate actions
  std::vector<NodeUpdate> forward_node_candidates;
  const NodeUpdate forward_straight{step_min, 0.0, 0.0, step_min, false, false};
  forward_node_candidates.push_back(forward_straight);
  for (int i = 0; i < turning_radius_size + 1; ++i) {
    double R = R_min + i * dR;
    double step = R * dtheta;
    NodeUpdate forward_left{R * sin(dtheta), R * (1 - cos(dtheta)), dtheta, step, true, false};
    NodeUpdate forward_right = forward_left.flipped();
    forward_node_candidates.push_back(forward_left);
    forward_node_candidates.push_back(forward_right);
  }

  for (int i = 0; i < theta_size; i++) {
    const double theta = dtheta * i;

    for (const auto & nu : forward_node_candidates) {
      transition_table[i].push_back(nu.rotated(theta));
    }

    if (use_back) {
      for (const auto & nu : forward_node_candidates) {
        transition_table[i].push_back(nu.reversed().rotated(theta));
      }
    }
  }

  return transition_table;
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const AstarParam & astar_param)
: AbstractPlanningAlgorithm(planner_common_param),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  transition_table_ = createTransitionTable(
    planner_common_param_.minimum_turning_radius, planner_common_param_.maximum_turning_radius,
    planner_common_param_.turning_radius_size, planner_common_param_.theta_size,
    astar_param_.use_back);
}

void AstarSearch::setMap(const Costmap & costmap)
{
  AbstractPlanningAlgorithm::setMap(costmap);
}

bool AstarSearch::makePlan(
  const Pose & start_pose, const Pose & goal_pose)
{
  std::cout<<"makeplan"<<std::endl;
  const auto height = costmap_.m_height;
  const auto width = costmap_.m_width;
  // Initialize nodes
  nodes_.clear();
  nodes_.resize(height);
  for (uint32_t i = 0; i < height; i++) {
    nodes_[i].resize(width);
    for (uint32_t j = 0; j < width; j++) {
      nodes_[i][j].resize(planner_common_param_.theta_size);
    }
  }
  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);
  if (!setStartNode()) {
    std::cout<<"setStartNode"<<std::endl;
    return false;
  }

  if (!setGoalNode()) {
    std::cout<<"setGoalNode"<<std::endl;
    return false;
  }
  std::cout<<"search"<<std::endl;
  return search();
}

bool AstarSearch::setStartNode()
{
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);
  std::cout<<"SX"<<index.x<<"SY"<<index.y<<"Stheta"<<index.theta<<std::endl;
  if (detectCollision(index)) {
    return false;
  }

  // Set start node
  AstarNode * start_node = getNodeRef(index);
  start_node->x = start_pose_.position.x;
  start_node->y = start_pose_.position.y;
  start_node->theta = 2.0 * M_PI / planner_common_param_.theta_size * index.theta;
  start_node->gc = 0;
  start_node->hc = estimateCost(start_pose_);
  start_node->is_back = false;
  start_node->status = NodeStatus::Open;
  start_node->parent = nullptr;

  // Push start node to openlist
  openlist_.push(start_node);

  return true;
}

bool AstarSearch::setGoalNode()
{
  const auto index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);
  std::cout<<"GX"<<index.x<<"GY"<<index.y<<"Gtheta"<<index.theta<<std::endl;
  if (detectCollision(index)) {
    return false;
  }

  return true;
}

double AstarSearch::estimateCost(const Pose & pose)
{
  double total_cost = 0.0;
  // Temporarily, until reeds_shepp gets stable.

  double radius = (planner_common_param_.minimum_turning_radius +
                    planner_common_param_.maximum_turning_radius) *
                  0.5;
  double rs_cost = calcReedsSheppDistance(pose, goal_pose_, radius);
  total_cost += rs_cost * astar_param_.distance_heuristic_weight;
  return total_cost;
}

bool AstarSearch::search()
{
  nowtime begin = Clock::now();

  min_x = 9999;
  min_y = 9999;
  min_dis = 999999;
  min_yaw = 9999;;
  // Start A* search
  int i = 0;
  while (!openlist_.empty()) {
    i++;
    // std::cout<<i<<std::endl;
    // Check time and terminate if the search reaches the time limit
    nowtime now = Clock::now();//计时结束
    const double msec = std::chrono::duration_cast<std::chrono::seconds>(now - begin).count() * 1000;
    if (msec > planner_common_param_.time_limit) {
      return false;
    }

    // Expand minimum cost node
    AstarNode * current_node = openlist_.top();
    openlist_.pop();
    current_node->status = NodeStatus::Closed;

    if (isGoal(*current_node)) {
      goal_node_ = current_node;
      setPath(*current_node);
      return true;
    }

    // Transit
    const auto index_theta = discretizeAngle(current_node->theta, planner_common_param_.theta_size);
    for (const auto & transition : transition_table_[index_theta]) {
      const bool is_turning_point = transition.is_back != current_node->is_back;
      

      double reverse_weight_ = planner_common_param_.reverse_weight;;
      const double move_cost = is_turning_point
                                 ? reverse_weight_ * transition.distance
                                 : transition.distance;

      // Calculate index of the next state
      Pose next_pose;
      next_pose.position.x = current_node->x + transition.shift_x;
      next_pose.position.y = current_node->y + transition.shift_y;
      next_pose.yaw = current_node->theta + transition.shift_theta;
      const auto next_index = pose2index(costmap_, next_pose, planner_common_param_.theta_size);
      if (detectCollision(next_index)) {
        continue;
      }

      // Compare cost
      AstarNode * next_node = getNodeRef(next_index);
      const double next_gc = current_node->gc + move_cost;
      if (next_node->status == NodeStatus::None || next_gc < next_node->gc) {
        next_node->status = NodeStatus::Open;
        next_node->x = next_pose.position.x;
        next_node->y = next_pose.position.y;
        next_node->theta = next_pose.yaw;
        next_node->gc = next_gc;
        next_node->hc = estimateCost(next_pose);
        next_node->is_back = transition.is_back;
        next_node->parent = current_node;
        openlist_.push(next_node);
        continue;
      }
    }
  }

  // Failed to find path
  return false;
}

void AstarSearch::setPath(const AstarNode & goal_node)
{
  waypoints_.stamp = Clock::now();
  waypoints_.waypoints.clear();

  // From the goal node to the start node
  const AstarNode * node = &goal_node;

  while (node != nullptr) {
    Pose pose;
    pose = local2global(costmap_, node2pose(*node));

    // PlannerWaypoint
    PlannerWaypoint pw;
    pw.pose = pose;
    pw.is_back = node->is_back;
    waypoints_.waypoints.push_back(pw);

    // To the next node
    node = node->parent;
  }
  // Reverse the vector to be start to goal order
  std::reverse(waypoints_.waypoints.begin(), waypoints_.waypoints.end());

  // Update first point direction
  if (waypoints_.waypoints.size() > 1) {
    waypoints_.waypoints.at(0).is_back = waypoints_.waypoints.at(1).is_back;
  }
}

bool AstarSearch::hasFeasibleSolution()
{
  if (goal_node_ == nullptr) {
    return false;
  }
  const AstarNode * node = goal_node_;
  while (node != nullptr) {
    auto index = pose2index(costmap_, node2pose(*node), planner_common_param_.theta_size);
    if (isOutOfRange(index) || detectCollision(index)) {
      return false;
    }
    node = node->parent;
  }
  return true;
}

bool AstarSearch::isGoal(const AstarNode & node)
{
  const double lateral_goal_range = planner_common_param_.lateral_goal_range / 2.0;
  const double longitudinal_goal_range = planner_common_param_.longitudinal_goal_range / 2.0;
  const double goal_angle = planner_common_param_.angle_goal_range * M_PI / 2.0 / 180;

  const auto relative_pose = calcRelativePose(goal_pose_, node2pose(node));

  if (astar_param_.only_behind_solutions && relative_pose.position.x > 0) {
    return false;
  }
  if(min_dis > (std::pow(std::fabs(relative_pose.position.x),2) + std::pow(std::fabs(relative_pose.position.y),2))){
    min_x = std::fabs(relative_pose.position.x);
    min_y = std::fabs(relative_pose.position.y);
    min_dis = (std::pow(std::fabs(relative_pose.position.x),2) + std::pow(std::fabs(relative_pose.position.y),2));
  }


  // std::cout<<"min_x"<<min_x<<std::endl;
  // std::cout<<"min_y"<<min_x<<std::endl;
  // std::cout<<"min_dis"<<min_x<<std::endl;
  // std::cout<<"min_yaw"<<min_yaw * 180 / 3.1415926<<std::endl;
  if (
    std::fabs(relative_pose.position.x) > longitudinal_goal_range ||
    std::fabs(relative_pose.position.y) > lateral_goal_range) {
    return false;
  }
  const auto angle_diff = relative_pose.yaw;

  if(min_yaw > std::abs(angle_diff)){
    min_yaw = std::abs(angle_diff);
  }  
  if (std::abs(angle_diff) > goal_angle) {
    return false;
  }
  // std::cout<<"min_x   "<<min_x<<std::endl;
  // std::cout<<"min_y   "<<min_x<<std::endl;
  // std::cout<<"min_dis   "<<min_x<<std::endl;
  // std::cout<<"min_yaw   "<<min_yaw * 180 / 3.1415926<<std::endl;
  return true;
}

}  // namespace freespace_planning_algorithms
