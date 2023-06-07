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


#include "freespace_planning_algorithms/abstract_algorithm.hpp"
#include "freespace_planning_algorithms/reeds_shepp.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

namespace freespace_planning_algorithms
{
enum class NodeStatus : uint8_t { None, Open, Closed };

struct AstarParam
{
  // base configs
  bool only_behind_solutions = false;  // solutions should be behind the goal
  bool use_back = true;               // backward search
  bool mid_pose = false; 
  double reverse_limit = 15.0; 
  double distance_heuristic_weight = 1.0;  // obstacle threshold on grid [0,255]

};
struct AstarNode
{
  NodeStatus status = NodeStatus::None;  // node status
  double x;                              // x
  double y;                              // y
  double theta;                          // theta
  double gc = 0;                         // actual cost
  double hc = 0;                         // heuristic cost
  bool is_back;                          // true if the current direction of the vehicle is back
  AstarNode * parent = nullptr;          // parent node

  double cost() const { return gc + hc; }
};

struct NodeComparison
{
  bool operator()(const AstarNode * lhs, const AstarNode * rhs)
  {
    return lhs->cost() > rhs->cost();
  }
};

struct NodeUpdate
{
  double shift_x;
  double shift_y;
  double shift_theta;
  double distance;
  bool is_curve;
  bool is_back;

  NodeUpdate rotated(const double theta) const
  {
    NodeUpdate result = *this;
    result.shift_x = std::cos(theta) * this->shift_x - std::sin(theta) * this->shift_y;
    result.shift_y = std::sin(theta) * this->shift_x + std::cos(theta) * this->shift_y;
    return result;
  }

  NodeUpdate flipped() const
  {
    NodeUpdate result = *this;
    result.shift_y = -result.shift_y;
    result.shift_theta = -result.shift_theta;
    return result;
  }

  NodeUpdate reversed() const
  {
    NodeUpdate result = *this;
    result.shift_x = -result.shift_x;
    result.shift_theta = -result.shift_theta;
    result.is_back = !result.is_back;
    return result;
  }
};

class AstarSearch : public AbstractPlanningAlgorithm
{
public:
  using TransitionTable = std::vector<std::vector<NodeUpdate>>;

  AstarSearch(const PlannerCommonParam & planner_common_param, const AstarParam & astar_param);

  void setMap(const Costmap & costmap) override;
  bool makePlan(
    const Pose & start_pose,
    const Pose & goal_pose) override;
  bool hasFeasibleSolution() override;  // currently used only in testing

  const PlannerWaypoints & getWaypoints() const { return waypoints_; }

private:
  bool search();
  void setPath(const AstarNode & goal);
  bool setStartNode();
  bool setGoalNode();
  double estimateCost(const Pose & pose);
  bool isGoal(const AstarNode & node);

  AstarNode * getNodeRef(const IndexXYT & index) { return &nodes_[index.y][index.x][index.theta]; }

  // Algorithm specific param
  AstarParam astar_param_;

  // hybrid astar variables
  TransitionTable transition_table_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison> openlist_;

  // goal node, which may helpful in testing and debugging
  AstarNode * goal_node_;

  // distance metric option (removed when the reeds_shepp gets stable)
  bool use_reeds_shepp_;
  bool node_init = false;
  double min_x, min_y, min_dis, min_yaw;
};
}  // namespace freespace_planning_algorithms

