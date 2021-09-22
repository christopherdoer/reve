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

#pragma once

#include <mutex>
#include <angles/angles.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <radar_ego_velocity_estimator/data_types.h>
#include <radar_ego_velocity_estimator/ros_helper.h>
#include <radar_ego_velocity_estimator/simple_profiler.h>

#include <radar_ego_velocity_estimator/RadarEgoVelocityEstimatorConfig.h>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator.h>

namespace reve
{
/**
 * @brief The RadarEgoVelocityEstimatorRos class provides a ROS interface for the RadarBodyVelocityEstimator
 */
class RadarEgoVelocityEstimatorRos
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   * @brief Constructor
   * @param nh    ros node handle
   */
  RadarEgoVelocityEstimatorRos(ros::NodeHandle nh);

  /**
   * @brief Pocesses a whole rosbag
   * @note Use sleep_ms to limit the processing speed
   * @param rosbag_path    full path to the rosbag
   * @param bag_start      start time for processing (skip the first bag_start seconds)
   * @param bag_duration   duration of processing (stop bag_duration after bag_start)
   * @param sleep_ms       sleep for these mulliseconds after each radar scan was processed
   */
  void runFromRosbag(const std::string& rosbag_path,
                     const double bag_start,
                     const double bag_duration,
                     const double sleep_ms);
  /**
   * @brief Recofigure callback
   */
  void reconfigureCallback(radar_ego_velocity_estimation::RadarEgoVelocityEstimatorConfig& config, uint32_t level)
  {
    estimator_.configure(config);
  }

  /**
   * @brief Does the acutal processing using the RadarBodeyVelocityEstimator class
   * @param radar_scan     radar scan message
   * @param trigger_stamp  trigger time stamp of radar scan used to stamp the estimated velocity
   */
  void processRadarData(const sensor_msgs::PointCloud2& radar_scan, const ros::Time& trigger_stamp);

  /**
   * @brief Imu message callback, called by ros::spin (ros mode) or runFromRosbag
   */
  void callbackImu(const sensor_msgs::ImuConstPtr& imu_msg);

  /**
   * @brief Radar scan message callback, alled by ros::spin (ros mode) or runFromRosbag
   */
  void callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_scan_msg);

  /**
   * @brief Radar trigger header message callback, called by ros::spin (ros mode) or runFromRosbag
   */
  void callbackRadarTrigger(const std_msgs::HeaderConstPtr& trigger_msg);

private:
  const std::string kPrefix = "[RadarEgoVelocityEstimatorRos]: ";

  dynamic_reconfigure::Server<radar_ego_velocity_estimation::RadarEgoVelocityEstimatorConfig> reconfigure_server_;

  RadarEgoVelocityEstimator estimator_;

  SimpleProfiler profiler;

  ros::Subscriber sub_radar_scan_;
  ros::Subscriber sub_radar_trigger_;

  ros::Publisher pub_twist_;
  ros::Publisher pub_twist_ground_truth_;

  bool run_without_trigger = false;

  std::mutex mutex_;
  ros::Time trigger_stamp = ros::TIME_MIN;
};

}  // namespace reve
