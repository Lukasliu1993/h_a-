// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */



#include <freespace_planning_algorithms/astar_search.hpp>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
using namespace cv;

using namespace std;
using namespace chrono;
typedef high_resolution_clock Clock;
typedef time_point<std::chrono::high_resolution_clock> nowtime;
using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::AstarParam;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::Pose;
using freespace_planning_algorithms::PoseArray;
using freespace_planning_algorithms::Position;
using freespace_planning_algorithms::IndexXYT;
using freespace_planning_algorithms::IndexXY;
using freespace_planning_algorithms::PlannerWaypoint;
using freespace_planning_algorithms::PlannerWaypoints;
struct ParkDetect
{
  int    park_mode;
  int    park_right;
  double  park_yaw;
  double  pos_x;
  double  pos_y;
  double  pos_yaw;
  double  car_length;
  double  car_back;
  double  car_width;
  double  side_front;
  double  side_back;
  double  side_left;
  double  side_right;
};
// struct Trajectory
// {
//   nowtime stamp;
//   PoseArray points;
// };
struct Trajectory
{
  nowtime stamp;
  vector<Pose> poses;
};
struct TrajectoryPoint
{
  nowtime stamp;
  Pose pose;
};
struct Twist
{
  nowtime stamp;
  double v_x;
  double v_y;
  double v_z;
  double a_x;
  double a_y;
  double a_z;
  double g_x;
  double g_y;
  double g_z;
};

namespace freespace_planner
{

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double brief_velocity;
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_max;
  double th_arrived_distance_min;
  double th_arrived_distance_;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
};
struct AreaParam
{
  double angle;
  int park_right; 
  bool search_area;
  double wc; 
  double inner_width; 
  double outer_width; 
  double lc; 
  double bc; 
  double d; 
  double len_r; 
  double len_l; 
  double infla; 
};
class FreespacePlannerNode 
{
public:
  FreespacePlannerNode();


  // params
  NodeParam node_param_;
  AreaParam area_param_;
  PlannerCommonParam planner_common_param_;
  AstarParam astar_param_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  Pose current_pose_;
  Pose goal_pose_;
  Pose mid_pose_;
  PoseArray trajectory_;
  PoseArray partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_, target_index_;
  bool engage_, way_valid;
  bool is_completed_ = false;
  bool stop_state_ = false;
  bool is_lateral = false;
  int forward_num_, back_num_;
  double goal_x_, goal_y_, goal_yaw_, street_len_r_, street_len_l_, street_width_, front_len_, wc_, bc_, infla_;
  // PoseStamped::ConstSharedPtr route_;
  Costmap occupancy_grid_, occupancy_grid_inflation;
  Twist twist_;
  nowtime stop_time_;
  std::deque<Twist> twist_buffer_;

  // functions used in the constructor
  void getPlanningCommonParam();
  void getAstarParam();

  // functions, callback
  void onRoute(const Pose msg);
  void onPark(const ParkDetect msg);
  void onOccupancyGrid(const Costmap msg);
  void onOccupancyGrid_inflation(const Costmap msg);
  void onEngage(const bool msg);
  void onTwist(const Twist msg);
  bool isStopped(const std::deque<Twist> & twist_buffer,
  const double th_stopped_velocity_mps);

  // void onTimer();
  void reset();
  void extend_fix_trajectory();
  bool isPlanRequired();
  void planTrajectory();
  void planReverseTrajectory();
  void planCurveTrajectory();
  void updateTargetIndex();
  void initializePlanningAlgorithm();
  void writelanelat_node(std::ofstream &ofs, std::string filepath, int id, double x, double y);
  void loadlanelat_v(struct AreaParam p, std::string name);
  void loadlanelat_l(struct AreaParam p, std::string name);
  // void loadlanelat_v(double angle, double wc, double lc, double bc, double d, double len_r, double len_l, double infla, double street_infla_, std::string name);
  // void loadlanelat_l(double wc, double lc, double bc, double d, double len_r, double len_l, double infla, double street_infla_, std::string name);
};
}  // namespace freespace_planner


