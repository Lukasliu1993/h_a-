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

#include "freespace_planning_algorithms/test.hpp"
using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::AstarSearch;
int main()
{
  cv::Mat map;
  map = cv::imread("./map.png",cv::IMREAD_GRAYSCALE);
  cv::Size sz = map.size();
  Costmap costmap_(map.data, map.cols, map.rows);
  costmap_.setpose(0, 0);
  freespace_planning_algorithms::PlannerCommonParam P_param;
  freespace_planning_algorithms::AstarParam A_param;
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  algo_.reset(new AstarSearch(P_param, A_param));
  freespace_planning_algorithms::VehicleShape extended_vehicle_shape;
  extended_vehicle_shape.length = 5.0;
  extended_vehicle_shape.width = 2.0;
  extended_vehicle_shape.base2back = 1.0;
  std::cout<<"here1"<<std::endl;
  algo_->setVehicleShape(extended_vehicle_shape);
  std::cout<<"here2"<<std::endl;
  algo_->setMap(costmap_);
  std::cout<<"here3"<<std::endl;
  freespace_planning_algorithms::Pose  start_pose, goal_pose;
  start_pose.position.x = 200;
  start_pose.position.y = 400;
  start_pose.yaw = 3.1415926*1.5;
  goal_pose.position.x = 300;
  goal_pose.position.y = 400;
  goal_pose.yaw = 3.1415926*1.5;
  const bool result = algo_->makePlan(goal_pose, start_pose);
  std::cout<<"result"<<result<<std::endl;
  std::cout<<"here4"<<std::endl;
  // Costmap *costmap_ = new Costmap(map.data, map.cols, map.rows);
	// cv::Mat outImg = Mat::zeros(map.rows, map.cols, CV_8UC1);
	// memcpy(outImg.data, costmap_->data, map.rows * map.cols);
  for(int i = 395; i < 405;i++){
    for(int j = 195;j < 205;j++){
      map.at<uchar>(i,j)=0;
    }
  }
  std::cout<<"map.cols"<<map.cols<<std::endl;
  std::cout<<"map.rows"<<map.rows<<std::endl;
  imshow("01",map);
  // imshow("02",outImg);
  waitKey(100000);
  return 0;
}
