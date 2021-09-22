// This file is part of REVE - Radar Ego Velocity Estimator
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <ros/ros.h>

#include <radar_ego_velocity_estimator/radar_body_velocity_estimator_ros.h>

using namespace reve;

const std::string kNodeName = "radar_body_velocity_estimator";
const std::string kPrefix   = "[" + kNodeName + "]: ";

int main(int argc, char** argv)
{
  ros::init(argc, argv, kNodeName);
  ros::NodeHandle nh("~");

  RadarBodyVelocityEstimatorRos estimator_ros(nh);

  ros::spin();

  return 0;
}
