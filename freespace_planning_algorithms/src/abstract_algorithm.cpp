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

#include "freespace_planning_algorithms/abstract_algorithm.hpp"


#include <vector>
double normalizeRadian(const double rad, const double min_rad = -M_PI)
{
  const auto max_rad = min_rad + 2 * M_PI;

  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * M_PI, value);
}
namespace freespace_planning_algorithms
{



int discretizeAngle(const double theta, const int theta_size)
{
  const double one_angle_range = 2.0 * M_PI / theta_size;
  return static_cast<int>(std::rint(normalizeRadian(theta, 0.0) / one_angle_range)) % theta_size;
}

IndexXYT pose2index(
  const Costmap & costmap, const Pose & pose_local,
  const int theta_size)
{
  const int index_x = pose_local.position.x / costmap.resolution;
  const int index_y = pose_local.position.y / costmap.resolution;
  const int index_theta = discretizeAngle(pose_local.yaw, theta_size);
  return {index_x, index_y, index_theta};
}

Pose index2pose(
  const Costmap & costmap, const IndexXYT & index, const int theta_size)
{
  Pose pose_local;

  pose_local.position.x = index.x * costmap.resolution;
  pose_local.position.y = index.y * costmap.resolution;

  const double one_angle_range = 2.0 * M_PI / theta_size;
  const double yaw = index.theta * one_angle_range;
  pose_local.yaw = yaw;

  return pose_local;
}
Pose global2local(
  const Costmap & costmap, const Pose & pose_global)
{
  Pose pose_local;
  pose_local.position.x = pose_global.position.x - costmap.pos_x;
  pose_local.position.y = pose_global.position.y - costmap.pos_y;
  pose_local.yaw = pose_global.yaw;

  return pose_local;
}
Pose local2global(
  const Costmap & costmap, const Pose & pose_local)
{
  Pose pose_globel;
  pose_globel.position.x = pose_local.position.x + costmap.pos_x;
  pose_globel.position.y = pose_local.position.y + costmap.pos_y;
  pose_globel.yaw = pose_local.yaw;

  return pose_globel;
}
void AbstractPlanningAlgorithm::setMap(const Costmap & costmap)
{
  costmap_ = costmap;
  const auto height = costmap_.m_height;
  const auto width = costmap_.m_width;
  std::cout<<"MAP1"<<std::endl;
  // Initialize status
  std::vector<std::vector<bool>> is_obstacle_table;
  is_obstacle_table.resize(height);
  for (uint32_t i = 0; i < height; i++) {
    is_obstacle_table.at(i).resize(width);
    for (uint32_t j = 0; j < width; j++) {
      const int cost = costmap_.data[i * width + j];

      if (cost < 200) {
        // std::cout<<i<<"and"<<j<<std::endl;
        is_obstacle_table[i][j] = false;
      }
      else{
        is_obstacle_table[i][j] = false;
      }
    }
  }
  std::cout<<"MAP2"<<std::endl;
  is_obstacle_table_ = is_obstacle_table;

  // construct collision indexes table
  for (int i = 0; i < planner_common_param_.theta_size; i++) {
    std::vector<IndexXY> indexes_2d;
    computeCollisionIndexes(i, indexes_2d);
    coll_indexes_table_.push_back(indexes_2d);
  }
  std::cout<<"MAP3"<<std::endl;
}

void AbstractPlanningAlgorithm::computeCollisionIndexes(
  int theta_index, std::vector<IndexXY> & indexes_2d)
{
  IndexXYT base_index{0, 0, theta_index};
  const VehicleShape & vehicle_shape = planner_common_param_.vehicle_shape;

  // Define the robot as rectangle
  const double back = -1.0 * vehicle_shape.base2back;
  const double front = vehicle_shape.length - vehicle_shape.base2back;
  const double right = -1.0 * vehicle_shape.width / 2.0;
  const double left = vehicle_shape.width / 2.0;

  const auto base_pose = index2pose(costmap_, base_index, planner_common_param_.theta_size);
  const auto base_theta = base_pose.yaw;
  // std::cout<<"back"<<back<<std::endl;
  // std::cout<<"front"<<front<<std::endl;
  // std::cout<<"right"<<right<<std::endl;
  // std::cout<<"left"<<left<<std::endl;
  // Convert each point to index and check if the node is Obstacle
  for (double x = back; x <= front; x += costmap_.resolution) {
    for (double y = right; y <= left; y += costmap_.resolution) {
  // for (double x = back; x <= front; x += front - back) {
  //   for (double y = right; y <= left; y += left - right) {
      // Calculate offset in rotated frame
      // std::cout<<"i"<<std::endl;
      const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
      const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

      Pose pose_local;
      pose_local.position.x = base_pose.position.x + offset_x;
      pose_local.position.y = base_pose.position.y + offset_y;

      const auto index = pose2index(costmap_, pose_local, planner_common_param_.theta_size);
      const auto index_2d = IndexXY{index.x, index.y};
      indexes_2d.push_back(index_2d);
    }
  }
}

bool AbstractPlanningAlgorithm::detectCollision(const IndexXYT & base_index)
{
  const auto & coll_indexes_2d = coll_indexes_table_[base_index.theta];
  // std::cout<<"coll_indexes_2d.size"<<coll_indexes_2d.size()<<std::endl;
  for (const auto & coll_index_2d : coll_indexes_2d) {
    int idx_theta = 0;  // whatever. Yaw is nothing to do with collision detection between grids.
    IndexXYT coll_index{coll_index_2d.x, coll_index_2d.y, idx_theta};
    // must slide to current base position
    coll_index.x += base_index.x;
    coll_index.y += base_index.y;
    // std::cout<<"coll_index.x"<<coll_index.x<<"coll_index.y"<<coll_index.y<<std::endl;
    if (isOutOfRange(coll_index) || isObs(coll_index)) {
      return true;
    }
  }
  return false;
}

bool AbstractPlanningAlgorithm::hasObstacleOnTrajectory(
  const PoseArray & trajectory)
{
  for (int i = 0; i< trajectory.poses.size();i++) {
    Pose pose = trajectory.poses[i];
    const auto pose_local = global2local(costmap_, pose);
    const auto index = pose2index(costmap_, pose_local, planner_common_param_.theta_size);
    if (detectCollision(index)) {
      return true;
    }
  }

  return false;
}

}  // namespace freespace_planning_algorithms
