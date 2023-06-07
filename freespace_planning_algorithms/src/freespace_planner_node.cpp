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
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "freespace_planning_algorithms/freespace_planner_node.hpp"
#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
namespace
{

// using freespace_planning_algorithms::AbstractPlanningAlgorithm;
// using freespace_planning_algorithms::AstarSearch;
// using freespace_planning_algorithms::AstarParam;
// using freespace_planning_algorithms::PlannerCommonParam;
// using freespace_planning_algorithms::Pose;
// using freespace_planning_algorithms::PoseArray;
// using freespace_planning_algorithms::Position;
// using freespace_planning_algorithms::IndexXYT;
// using freespace_planning_algorithms::IndexXY;
// using freespace_planning_algorithms::PlannerWaypoint;
// using freespace_planning_algorithms::PlannerWaypoints;

// PoseArray trajectory2PoseArray(const PoseArray & PoseArray)
// {
//   PoseArray pose_array;
//   pose_array.stamp = PoseArray.stamp;

//   for (const auto & pose : PoseArray.poses) {
//     pose_array.poses.push_back(pose);
//   }

//   return pose_array;
// }

std::vector<size_t> getReversingIndices(const PoseArray & PoseArray)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < PoseArray.poses.size() - 1; ++i) {
    if (PoseArray.poses.at(i).vel * PoseArray.poses.at(i + 1).vel < 0) {
      indices.push_back(i);
      // RCLCPP_INFO(rclcpp::get_logger("parking"), "indices:%ld", i);
    }
  }

  return indices;
}

size_t getNextTargetIndex(
  const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
  const size_t current_target_index)
{
  if (!reversing_indices.empty()) {
    for (const auto reversing_index : reversing_indices) {
      if (reversing_index > current_target_index) {
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;
}

PoseArray getPartialTrajectory(
  const PoseArray & posearray, const size_t start_index, const size_t end_index)
{
  PoseArray partial_trajectory;
  partial_trajectory.stamp = posearray.stamp;
  partial_trajectory.stamp = Clock::now();;

  partial_trajectory.poses.reserve(posearray.poses.size());
  for (size_t i = start_index; i <= end_index; ++i) {
    partial_trajectory.poses.push_back(posearray.poses.at(i));
  }

  // Modify velocity at start/end point
  if (partial_trajectory.poses.size() >= 2) {
    partial_trajectory.poses.front().vel =
      partial_trajectory.poses.at(1).vel;
  }
  if (!partial_trajectory.poses.empty()) {
    partial_trajectory.poses.back().vel = 0;
  }

  return partial_trajectory;
}
double calcSquaredDistance2d(const Position & point1, const Position & point2)
{
  const auto dx = point1.x - point2.x;
  const auto dy = point1.y - point2.y;
  return dx * dx + dy * dy;
}
double calcHypotDistance2d(const Position & point1, const Position & point2)
{
  const auto dx = point1.x - point2.x;
  const auto dy = point1.y - point2.y;
  return std::hypot(dx, dy);
}
size_t findNearestIndex(const PoseArray & posearray, const Position & point)
{

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < posearray.poses.size(); ++i) {
    const auto dist = calcSquaredDistance2d(posearray.poses.at(i).position, point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

double calcDistance2d(const PoseArray & posearray, const Pose & pose)
{
  const auto idx = findNearestIndex(posearray, pose.position);
  return calcHypotDistance2d(posearray.poses.at(idx).position, pose.position);
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
double normalizeRadian(const double rad, const double min_rad = -M_PI)
{
  const auto max_rad = min_rad + 2 * M_PI;

  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * M_PI, value);
}
PoseArray createTrajectory(const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  PoseArray trajectory;
  PoseArray trajectory_ex;
  trajectory_ex.stamp = planner_waypoints.stamp;
  trajectory.stamp = planner_waypoints.stamp;

  for (const auto & awp : planner_waypoints.waypoints) {
    Pose pose;

    pose.position = awp.pose.position;
    pose.yaw = awp.pose.yaw;
    pose.acc = 0;
    pose.vel = velocity;


    // switch sign by forward/backward
    pose.vel = (awp.is_back ? -1 : 1) * pose.vel;

    trajectory.poses.push_back(pose);
  }
  // for(int i = 0; i < static_cast<int>(trajectory.points.size()); i++){
  //   RCLCPP_INFO(rclcpp::get_logger("!!"), "PoseArray point %d park goal is (%f,%f), v is %f, yaw is %f", i, trajectory.points[i].pose.position.x, 
  //   trajectory.points[i].pose.position.y,trajectory.points[i].twist.linear.x, tf2::getYaw(trajectory.points[i].pose.orientation));
  // }

  for (size_t i = 0; i < trajectory.poses.size() - 1; i++) {

    trajectory_ex.poses.push_back(trajectory.poses[i]);
    double deita_yaw = std::atan2(trajectory.poses[i+1].position.y - trajectory.poses[i].position.y,
    trajectory.poses[i+1].position.x - trajectory.poses[i].position.x);
    deita_yaw = (trajectory.poses[i+1].vel < 0 ? M_PI : 0) + deita_yaw;
    deita_yaw = normalizeRadian(deita_yaw);

    Pose point_ex;
    point_ex.yaw = deita_yaw;
    point_ex.position.x = (2 * trajectory.poses[i].position.x + trajectory.poses[i+1].position.x) / 3;
    point_ex.position.y = (2 * trajectory.poses[i].position.y + trajectory.poses[i+1].position.y) / 3;

    point_ex.acc = trajectory.poses[i+1].acc;
    point_ex.vel = trajectory.poses[i+1].vel;
    trajectory_ex.poses.push_back(point_ex);
    Pose point_ex2;
    point_ex2.yaw = deita_yaw;
    point_ex2.position.x = (trajectory.poses[i].position.x + 2 * trajectory.poses[i+1].position.x) / 3;
    point_ex2.position.y = (trajectory.poses[i].position.y + 2 * trajectory.poses[i+1].position.y) / 3;
 
    point_ex2.acc = trajectory.poses[i+1].acc;
    point_ex2.vel = trajectory.poses[i+1].vel;
    trajectory_ex.poses.push_back(point_ex2);    
  }
  trajectory_ex.poses.push_back(trajectory.poses[trajectory.poses.size() - 1]);  
  return trajectory_ex;
}
PoseArray createReverseTrajectory(const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  PoseArray trajectory;
  PoseArray trajectory_ex;
  trajectory_ex.stamp = planner_waypoints.stamp;
  trajectory.stamp = planner_waypoints.stamp;
  for (int i = planner_waypoints.waypoints.size() - 1; i >= 0; i--) {
    auto awp = planner_waypoints.waypoints[i];
    Pose pose;

    pose.position = awp.pose.position;
    pose.yaw = awp.pose.yaw;
    pose.acc = 0;
    pose.vel = velocity;


    // switch sign by forward/backward
    pose.vel = (awp.is_back ? -1 : 1) * pose.vel;

    trajectory.poses.push_back(pose);
  }

  for(int i = 1; i < static_cast<int>(trajectory.poses.size()); i++){
    const auto relative_pose = calcRelativePose(trajectory.poses[i-1], trajectory.poses[i]);
    if(relative_pose.position.x > 0){
      trajectory.poses[i].vel = velocity;
    }else if(relative_pose.position.x < 0){
      trajectory.poses[i].vel = -1 * velocity;
    }else{
      trajectory.poses[i].vel = trajectory.poses[i-1].vel;
    }
  }
  // for(int i = 0; i < static_cast<int>(PoseArray.points.size()); i++){
  //   RCLCPP_INFO(rclcpp::get_logger("!!"), "PoseArray point %d park goal is (%f,%f), v is %f, yaw is %f", i, PoseArray.points[i].pose.position.x, 
  //   PoseArray.points[i].pose.position.y,PoseArray.points[i].twist.linear.x, tf2::getYaw(PoseArray.points[i].pose.orientation));
  // }
  for (size_t i = 0; i < trajectory.poses.size() - 1; i++) {

    trajectory_ex.poses.push_back(trajectory.poses[i]);
    double deita_yaw = std::atan2(trajectory.poses[i+1].position.y - trajectory.poses[i].position.y,
    trajectory.poses[i+1].position.x - trajectory.poses[i].position.x);
    deita_yaw = (trajectory.poses[i+1].vel < 0 ? M_PI : 0) + deita_yaw;
    deita_yaw = normalizeRadian(deita_yaw);

    Pose point_ex;
    point_ex.yaw = deita_yaw;
    point_ex.position.x = (2 * trajectory.poses[i].position.x + trajectory.poses[i+1].position.x) / 3;
    point_ex.position.y = (2 * trajectory.poses[i].position.y + trajectory.poses[i+1].position.y) / 3;

    point_ex.acc = trajectory.poses[i+1].acc;
    point_ex.vel = trajectory.poses[i+1].vel;
    trajectory_ex.poses.push_back(point_ex);
    Pose point_ex2;
    point_ex2.yaw = deita_yaw;
    point_ex2.position.x = (trajectory.poses[i].position.x + 2 * trajectory.poses[i+1].position.x) / 3;
    point_ex2.position.y = (trajectory.poses[i].position.y + 2 * trajectory.poses[i+1].position.y) / 3;
 
    point_ex2.acc = trajectory.poses[i+1].acc;
    point_ex2.vel = trajectory.poses[i+1].vel;
    trajectory_ex.poses.push_back(point_ex2);    
  }
  trajectory_ex.poses.push_back(trajectory.poses[trajectory.poses.size() - 1]);  
  return trajectory_ex;
}
PoseArray createStopTrajectory(const Pose & current_pose)
{
  PlannerWaypoints waypoints;
  PlannerWaypoint waypoint;

  waypoints.stamp = Clock::now();
  waypoint.pose = current_pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);

  return createTrajectory(waypoints, 0.0);
}



}  // namespace

namespace freespace_planner
{
FreespacePlannerNode::FreespacePlannerNode()
{
  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = "astar";
    p.waypoints_velocity = 1.0;
    p.brief_velocity = 0.1;
    p.update_rate = 1.0;
    p.th_arrived_distance_min = 0.1;
    p.th_arrived_distance_max = 0.4;
    p.th_arrived_distance_ = p.th_arrived_distance_min;
    p.th_stopped_time_sec = 1.0;
    p.th_stopped_velocity_mps = 0.01;
    p.th_course_out_distance_m = 3.0;
    p.replan_when_obstacle_found = true;
    p.replan_when_course_out = true;
    is_completed_ = false;
    goal_x_ = 0.0;
    goal_y_ = 0.0;
    goal_yaw_ = 0.0;
    forward_num_ = 0;
    back_num_ = 0;


    area_param_.len_r = 30.0;
    area_param_.len_l = 30.0;
    area_param_.d = 12.0;
    area_param_.lc = 0.0;
    area_param_.wc = 0.0;
    area_param_.inner_width = 0.0;
    area_param_.outer_width = 0.0;
    area_param_.bc = 0.0;
    area_param_.infla = 0.0;
    bool park_right = true;
    area_param_.park_right = park_right ? 1 : -1;
  }

  // Planning
  getPlanningCommonParam();
  getAstarParam();

  current_pose_.position.x = 0;
  current_pose_.position.y = 0;
  current_pose_.yaw = 0;
  goal_pose_.position.x = 0;
  goal_pose_.position.y = 0;;
  goal_pose_.yaw = 0;
}

void FreespacePlannerNode::getPlanningCommonParam()
{
  auto & p = planner_common_param_;

  // base configs
  p.time_limit = 5000.0;

  // robot configs
  p.vehicle_shape.length = 5.0;
  p.vehicle_shape.width = 2.0;
  p.vehicle_shape.base2back = 1.0;
  // TODO(Kenji Miyake): obtain from vehicle_info
  p.minimum_turning_radius = 3.5;
  p.maximum_turning_radius = 3.5;
  p.turning_radius_size = 1;
  p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  // search configs
  p.theta_size = 90;
  p.angle_goal_range = 10.0;
  p.curve_weight = 1.2;
  p.reverse_weight = 2.0;
  p.lateral_goal_range = 2.0;
  p.longitudinal_goal_range = 2.0;

}

void FreespacePlannerNode::getAstarParam()
{
  auto & p = astar_param_;
  p.only_behind_solutions = false;
  p.use_back = true;
  p.mid_pose = false;
  p.distance_heuristic_weight = 1.0;
  p.reverse_limit = 0.2;

}



// void FreespacePlannerNode::onScenario(const Scenario::ConstSharedPtr msg) { scenario_ = msg; }

void FreespacePlannerNode::onTwist(const Twist msg)
{
  twist_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = msg.stamp - twist_buffer_.front().stamp;
    if (std::chrono::duration_cast<std::chrono::seconds>(time_diff).count() < node_param_.th_stopped_time_sec) {
      break;
    }
    twist_buffer_.pop_front();
  }
}

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.poses.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found) {
    // RCLCPP_INFO(rclcpp::get_logger("parking"), "[check rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());
    algo_->setMap(occupancy_grid_);
    // RCLCPP_INFO(rclcpp::get_logger("parking"), "[check rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());
    const size_t nearest_index_partial = findNearestIndex(partial_trajectory_, current_pose_.position);
    const size_t end_index_partial = partial_trajectory_.poses.size() - 1;
    const auto forward_trajectory =
      getPartialTrajectory(partial_trajectory_, nearest_index_partial, end_index_partial);
    const bool is_obstacle_found =
      algo_->hasObstacleOnTrajectory(forward_trajectory);
    if (is_obstacle_found) {
      // RCLCPP_INFO(get_logger(), "Found obstacle");
      return true;
    }
  }
  if (node_param_.replan_when_course_out) {
    const bool is_course_out =
      calcDistance2d(trajectory_, current_pose_) > node_param_.th_course_out_distance_m;
    if (is_course_out) {
      // RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }
  return false;
}

bool FreespacePlannerNode::isStopped(const std::deque<Twist> & twist_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & twist : twist_buffer) {
   if (std::abs(twist.v_x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}



void FreespacePlannerNode::updateTargetIndex()
{
  const auto is_near_target =
    calcHypotDistance2d(trajectory_.poses.at(target_index_).position, current_pose_.position) <
    node_param_.th_arrived_distance_;
  if (is_near_target) {
    // RCLCPP_INFO(rclcpp::get_logger("parking"), "near target");
  }

  const auto is_stopped = isStopped(twist_buffer_, node_param_.th_stopped_velocity_mps);
  if (is_stopped && !stop_state_) {
    stop_time_ = Clock::now();
    stop_state_ = true;
  }
  else if(!is_stopped){
    stop_state_ = false;
    node_param_.th_arrived_distance_ = node_param_.th_arrived_distance_min;
  }
  else{
  }

  if (!is_near_target && is_stopped) {
    nowtime now = Clock::now();
    double delta_time = std::chrono::duration_cast<std::chrono::seconds>(now - stop_time_).count();
    if(delta_time > 2.5){
      node_param_.th_arrived_distance_ = node_param_.th_arrived_distance_max;
    }
  }

  if (is_near_target && is_stopped) {
    // RCLCPP_INFO(rclcpp::get_logger("parking"), "near target and stopped");
    const auto new_target_index =
      getNextTargetIndex(trajectory_.poses.size(), reversing_indices_, target_index_);

    if (new_target_index == target_index_ && is_stopped) {
      // Finished publishing all partial trajectories
      is_completed_ = true;
      // this->set_parameter(rclcpp::Parameter("is_completed", true));
      // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
      // std_msgs::msg::Bool data_pause;
      // data_pause.data = true;
      // completed_pub_->publish(data_pause); 
    } else {
      // Switch to next partial PoseArray
      prev_target_index_ = target_index_;
      target_index_ =
        getNextTargetIndex(trajectory_.poses.size(), reversing_indices_, target_index_);
    }
  }
}

// void FreespacePlannerNode::onTimer()
// {

//   if (!engage_ ) {
//     return;
//   }

//   if (is_completed_) {
//     return;
//   }
//   // Get current pose
//   current_pose_.position.x = 0;
//   current_pose_.position.y = 0;

//   initializePlanningAlgorithm();
//   if (isPlanRequired()) {
//     // RCLCPP_INFO(rclcpp::get_logger("parking"), "plan required");
//     reset();

//     // Stop before planning new PoseArray
//     const auto stop_trajectory = createStopTrajectory(current_pose_);
//     // RCLCPP_INFO(rclcpp::get_logger("parking"), "stop pub!!!!!!!!! ");
//     // trajectory_pub_->publish(stop_trajectory);
//     // debug_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
//     // debug_partial_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));

//     // Plan new PoseArray
//     current_pose_ =
//     autoware_utils::transform2pose(getTransform(occupancy_grid_->header.frame_id, vehicle_frame));
//     const auto relative_pose = calcRelativePose(goal_pose_.pose, current_pose_.pose);
//     // RCLCPP_INFO(rclcpp::get_logger("parking"), "rl x:%f y:%f yaw %f", relative_pose.position.x, relative_pose.position.y, tf2::getYaw(relative_pose.orientation));
//     // if(is_lateral && relative_pose.position.x < planner_common_param_.vehicle_shape.length){
//     //   // astar_param_.end_limit = 0.0;
//     //   // planner_common_param_.reverse_weight = 2.0;
//     //   // initializePlanningAlgorithm();
//     //   planCurveTrajectory();
//     // }else{
//     //   planCurveTrajectory();
//     //   // planTrajectory();
//     // }
//     planReverseTrajectory();
//   }
//   // StopTrajectory
//   if (trajectory_.points.size() <= 1) {
//     // RCLCPP_INFO(rclcpp::get_logger("parking"), "stop PoseArray ");
//     return;
//   }
//   double distance = 0;
//   for(int i = 1; i < static_cast<int>(trajectory_.points.size()); i++){
//     distance += std::sqrt(std::pow(trajectory_.points[i].pose.position.x - trajectory_.points[i-1].pose.position.x,2) + std::pow(trajectory_.points[i].pose.position.y - trajectory_.points[i-1].pose.position.y,2));
//   }
//   // RCLCPP_INFO(rclcpp::get_logger("parking"), "distance %f", distance);
//   // Update partial PoseArray
//   // RCLCPP_INFO(rclcpp::get_logger("parking"), "begin update Target Index, time %f", node_param_.th_stopped_time_sec);
//   current_pose_ =
//   autoware_utils::transform2pose(getTransform(occupancy_grid_->header.frame_id, vehicle_frame));
//   updateTargetIndex();
//   // RCLCPP_INFO(rclcpp::get_logger("parking"), "prev_target_index_ %ld |||| target_index_ %ld", prev_target_index_, target_index_);
//   partial_trajectory_ = getPartialTrajectory(trajectory_, prev_target_index_, target_index_);
//   partial_trajectory_.points[partial_trajectory_.points.size()-1].twist.linear.x = 0;
//   partial_trajectory_.points[0].twist.linear.x = partial_trajectory_.points[0].twist.linear.x / std::abs(partial_trajectory_.points[0].twist.linear.x) * node_param_.brief_velocity;
//   // if(partial_trajectory_.points.size() >= 4){
//   //   partial_trajectory_.points[partial_trajectory_.points.size()-3].twist.linear.x = (2 * partial_trajectory_.points[partial_trajectory_.points.size()-4].twist.linear.x + 
//   //   1 * partial_trajectory_.points[partial_trajectory_.points.size()-1].twist.linear.x) / 3;
//   //   partial_trajectory_.points[partial_trajectory_.points.size()-2].twist.linear.x = (1 * partial_trajectory_.points[partial_trajectory_.points.size()-4].twist.linear.x + 
//   //   2 * partial_trajectory_.points[partial_trajectory_.points.size()-1].twist.linear.x) / 3;;
//   // }
//   double dx = partial_trajectory_.points[partial_trajectory_.points.size() -1].pose.position.x - partial_trajectory_.points[0].pose.position.x;
//   double dy = partial_trajectory_.points[partial_trajectory_.points.size() -1].pose.position.y - partial_trajectory_.points[0].pose.position.y;
//   double dis = dx * dx + dy * dy;
//   if(dis < 1){
//     for(int i = 0; i < static_cast<int>(partial_trajectory_.points.size()); i++){
//       if (partial_trajectory_.points[i].twist.linear.x == 0)
//         continue;
//       partial_trajectory_.points[i].twist.linear.x = 
//         partial_trajectory_.points[i].twist.linear.x / std::abs(partial_trajectory_.points[i].twist.linear.x) * node_param_.brief_velocity;
//     }    
//   }
//   size_t k = std::min((size_t) 5, (size_t)partial_trajectory_.points.size() / 2);
//   double deita_v = partial_trajectory_.points[k].twist.linear.x - partial_trajectory_.points[0].twist.linear.x;
//   for (size_t i = 0; i < k; i++){
//     partial_trajectory_.points[i].twist.linear.x = partial_trajectory_.points[0].twist.linear.x + deita_v * i / (k - 1);
//   } 

//   // k = std::min((size_t) 5, (size_t)partial_trajectory_.points.size() / 2);
//   // deita_v = partial_trajectory_.points[partial_trajectory_.points.size() - k].twist.linear.x - partial_trajectory_.points[partial_trajectory_.points.size()-1].twist.linear.x;
//   // for (size_t i = partial_trajectory_.points.size() - k; i < partial_trajectory_.points.size(); i++){
//   //   partial_trajectory_.points[i].twist.linear.x = deita_v * (partial_trajectory_.points.size() - i -1) / (k - 1);
//   // } 
  
//   // for(int i = 0; i < static_cast<int>(partial_trajectory_.points.size()); i++){
//   //   RCLCPP_INFO(rclcpp::get_logger("bb"), "partial point %d park goal is (%f,%f), v is %f, yaw is %f", i, partial_trajectory_.points[i].pose.position.x, 
//   //   partial_trajectory_.points[i].pose.position.y,partial_trajectory_.points[i].twist.linear.x, tf2::getYaw(partial_trajectory_.points[i].pose.orientation));
//   // }
//   // Publish messages
//   trajectory_all_pub_->publish(trajectory_);
//   trajectory_pub_->publish(partial_trajectory_);
//   debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory_));
//   debug_partial_pose_array_pub_->publish(trajectory2PoseArray(partial_trajectory_));
// }
void FreespacePlannerNode::planReverseTrajectory()
{
  // Extend robot shape
  freespace_planning_algorithms::VehicleShape extended_vehicle_shape =
    planner_common_param_.vehicle_shape;
  constexpr double margin = 0.0;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;
  // RCLCPP_INFO(rclcpp::get_logger("parking"), "extended_vehicle_shape.length %f", extended_vehicle_shape.length);
  // RCLCPP_INFO(rclcpp::get_logger("parking"), "extended_vehicle_shape.width %f", extended_vehicle_shape.width);
  // RCLCPP_INFO(rclcpp::get_logger("parking"), "extended_vehicle_shape.base2back %f", extended_vehicle_shape.base2back);
  // Provide robot shape and map for the planner
  algo_->setVehicleShape(extended_vehicle_shape);
  // auto t = rclcpp::Clock().now(); 
  // RCLCPP_INFO(rclcpp::get_logger("parking"), "[rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());
  algo_->setMap(occupancy_grid_inflation);
  // t = rclcpp::Clock().now(); 
  // RCLCPP_INFO(rclcpp::get_logger("parking"), "[rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());
  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = current_pose_;

  const auto goal_pose_in_costmap_frame = goal_pose_;

  // execute planning

  // RCLCPP_INFO(rclcpp::get_logger("parking"), "begin plan!");
  const bool result = algo_->makePlan(goal_pose_in_costmap_frame, current_pose_in_costmap_frame);

  if (result) {
    // RCLCPP_INFO(get_logger(), "Found goal!");
    trajectory_ =
      createReverseTrajectory(algo_->getWaypoints(), node_param_.waypoints_velocity);
    // RCLCPP_INFO(rclcpp::get_logger("parking"), "PoseArray size is %d", static_cast<int>(trajectory_.points.size()));
    for(int i = 0; i < static_cast<int>(trajectory_.poses.size()); i++){
      // RCLCPP_INFO(rclcpp::get_logger("bb"), "point %d park goal is (%f,%f), v is %f, yaw is %f", i, trajectory_.points[i].pose.position.x, 
      // trajectory_.points[i].pose.position.y,trajectory_.points[i].twist.linear.x, tf2::getYaw(trajectory_.points[i].pose.orientation));
    }
    reversing_indices_ = getReversingIndices(trajectory_);
    prev_target_index_ = 0;
    target_index_ =
      getNextTargetIndex(trajectory_.poses.size(), reversing_indices_, prev_target_index_);

  } else {
    // RCLCPP_INFO(get_logger(), "Can't find goal...");
    reset();
  }
}

void FreespacePlannerNode::reset()
{
  trajectory_ = PoseArray();
  partial_trajectory_ = PoseArray();
  is_completed_ = false;
  // this->set_parameter(rclcpp::Parameter("is_completed", false));
  // std_msgs::msg::Bool data_pause;
  // data_pause.data = false;
  // completed_pub_->publish(data_pause); 
}

// TransformStamped FreespacePlannerNode::getTransform(
//   const std::string & from, const std::string & to)
// {
//   TransformStamped tf;
//   try {
//     tf =
//       tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
//   } catch (const tf2::TransformException & ex) {
//     RCLCPP_ERROR(get_logger(), "%s", ex.what());
//   }
//   return tf;
// }

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  if (node_param_.planning_algorithm == "astar") {
    algo_.reset(new AstarSearch(planner_common_param_, astar_param_));
  } else {
    throw std::runtime_error(
      "No such algorithm named " + node_param_.planning_algorithm + " exists.");
  }
}
}  // namespace freespace_planner
int main()
{
  freespace_planner::FreespacePlannerNode planner_;
  planner_.reset();
  planner_.initializePlanningAlgorithm();
  cv::Mat map;
  map = cv::imread("./map.png",cv::IMREAD_GRAYSCALE);
  cv::Size sz = map.size();
  Costmap costmap_(map.data, map.cols, map.rows);
  costmap_.setpose(0, 0);

  freespace_planning_algorithms::VehicleShape extended_vehicle_shape;
  extended_vehicle_shape.length = 5.0;
  extended_vehicle_shape.width = 2.0;
  extended_vehicle_shape.base2back = 1.0;
  std::cout<<"here1"<<std::endl;
  planner_.algo_->setVehicleShape(extended_vehicle_shape);
  std::cout<<"here2"<<std::endl;
  planner_.algo_->setMap(costmap_);
  std::cout<<"here3"<<std::endl;
  freespace_planning_algorithms::Pose  start_pose, goal_pose;
  start_pose.position.x = 400;
  start_pose.position.y = 200;
  start_pose.yaw = 3.1415926*1.5;
  goal_pose.position.x = 400;
  goal_pose.position.y = 300;
  goal_pose.yaw = 3.1415926*1.5;
  const bool result = planner_.algo_->makePlan(goal_pose, start_pose);
  std::cout<<"result"<<result<<std::endl;
  std::cout<<"here4"<<std::endl;

  if (result) {
    std::cout<<"Found goal!"<<std::endl;
    PoseArray trajectory_ =
      createReverseTrajectory(planner_.algo_->getWaypoints(), planner_.node_param_.waypoints_velocity);

    for(int i = 0; i < static_cast<int>(trajectory_.poses.size()); i++){
      std::cout<<"i"<<i<<"pos_x"<<trajectory_.poses[i].position.x<<"pos_y<<"<<
      trajectory_.poses[i].position.y<<"velx"<<trajectory_.poses[i].vel<<"pos_yaw"<<trajectory_.poses[i].yaw<<std::endl;
    }
    planner_.reversing_indices_ = getReversingIndices(trajectory_);
    planner_.prev_target_index_ = 0;
    planner_.target_index_ =
      getNextTargetIndex(trajectory_.poses.size(), planner_.reversing_indices_, planner_.prev_target_index_);

  } else {
    std::cout<<"Can't find goal..."<<std::endl;
    planner_.reset();
  }

  // for(int i = 395; i < 405;i++){
  //   for(int j = 195;j < 205;j++){
  //     map.at<uchar>(i,j)=0;
  //   }
  // }
  std::cout<<"map.cols"<<map.cols<<std::endl;
  std::cout<<"map.rows"<<map.rows<<std::endl;
  imshow("01",map);
  // imshow("02",outImg);
  waitKey(100000);
  return 0;
}
