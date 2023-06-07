// Copyright 2021 Tier IV, Inc. All rights reserved.
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



#include <vector>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <math.h>
#include <algorithm>
#include "freespace_planning_algorithms/costmap.hpp"
using namespace std;
using namespace chrono;
typedef high_resolution_clock Clock;
typedef time_point<std::chrono::high_resolution_clock> nowtime;
namespace freespace_planning_algorithms
{
int discretizeAngle(const double theta, const int theta_size);
struct Position
{
  double x;
  double y;
};
struct Pose
{
  Position position;
  double yaw;
  double vel;
  double acc;
};
struct PoseArray
{
  nowtime stamp;
  vector<Pose> poses;
};
struct IndexXYT
{
  int x;
  int y;
  int theta;
};

struct IndexXY
{
  int x;
  int y;
};
IndexXYT pose2index(
  const Costmap & costmap, const Pose & pose_local,
  const int theta_size);

Pose index2pose(
  const Costmap & costmap, const IndexXYT & index, const int theta_size);

Pose global2local(
  const Costmap & costmap, const Pose & pose_global);

Pose local2global(
  const Costmap & costmap, const Pose & pose_local);

struct VehicleShape
{
  double length = 5.0;     // X [m]
  double width = 2.0;      // Y [m]
  double base2back = 1.0;  // base_link to rear [m]
};

struct PlannerCommonParam
{
  // base configs
  double time_limit = 15000.0;  // planning time limit [msec]

  // robot configs
  VehicleShape vehicle_shape;
  double minimum_turning_radius = 3.6;  // [m]
  double maximum_turning_radius = 3.6;  // [m]
  int turning_radius_size = 1;        // discretized turning radius table size [-]

  // search configs
  int theta_size = 90;                  // discretized angle table size [-]
  double curve_weight = 1.2;             // curve moving cost [-]
  double reverse_weight = 2.0;           // backward moving cost [-]
  double lateral_goal_range = 4;       // reaching threshold, lateral error [m]
  double longitudinal_goal_range = 4;  // reaching threshold, longitudinal error [m]
  double angle_goal_range = 10.0;         // reaching threshold, angle error [deg]
  // costmap configs
};

struct PlannerWaypoint
{
  Pose pose;
  bool is_back = false;
};

struct PlannerWaypoints
{
  nowtime stamp;
  std::vector<PlannerWaypoint> waypoints;
};

class AbstractPlanningAlgorithm
{
public:
  AbstractPlanningAlgorithm(const PlannerCommonParam & planner_common_param)
  {
    planner_common_param_ = planner_common_param;
  }
  AbstractPlanningAlgorithm()
  {
  }
  virtual void setMap(const Costmap & costmap);
  virtual bool makePlan(
    const Pose & start_pose, const Pose & goal_pose) = 0;
  virtual bool hasFeasibleSolution() = 0;  // currently used only in testing
  void setVehicleShape(const VehicleShape & vehicle_shape)
  {
    std::cout<<"veh"<<std::endl;
    planner_common_param_.vehicle_shape = vehicle_shape;
    std::cout<<"veh2"<<std::endl;
  }
  bool hasObstacleOnTrajectory(const PoseArray & trajectory);
  const PlannerWaypoints & getWaypoints() const { return waypoints_; }
  virtual ~AbstractPlanningAlgorithm() {}

protected:
  void computeCollisionIndexes(int theta_index, std::vector<IndexXY> & indexes);
  bool detectCollision(const IndexXYT & base_index);
  inline bool isOutOfRange(const IndexXYT & index)
  {
    if (index.y < 0 || static_cast<int>(costmap_.m_width) <= index.y) {
      std::cout<<"isOutOfRange1"<<std::endl;
      return true;
    }
    if (index.x < 0 || static_cast<int>(costmap_.m_height) <= index.x) {
      std::cout<<"isOutOfRange2"<<std::endl;
      return true;
    }
    return false;
  }
  inline bool isObs(const IndexXYT & index)
  {
    // NOTE: Accessing by .at() instead makes 1.2 times slower here.
    // Also, boundary check is already done in isOutOfRange before calling this function.
    // So, basically .at() is not necessary.
    return is_obstacle_table_[costmap_.m_height - index.x][costmap_.m_width - index.y];
  }

  PlannerCommonParam planner_common_param_;

  // costmap as occupancy grid
  Costmap costmap_;

  // collision indexes cache
  std::vector<std::vector<IndexXY>> coll_indexes_table_;

  // is_obstacle's table
  std::vector<std::vector<bool>> is_obstacle_table_;

  // pose in costmap frame
  Pose start_pose_;
  Pose goal_pose_;

  // result path
  PlannerWaypoints waypoints_;
};
}  // namespace freespace_planning_algorithms
