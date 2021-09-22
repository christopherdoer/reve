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

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <radar_ego_velocity_estimator/ros_helper.h>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_ros.h>

using namespace reve;

RadarEgoVelocityEstimatorRos::RadarEgoVelocityEstimatorRos(ros::NodeHandle nh)
{
  reconfigure_server_.setCallback(boost::bind(&RadarEgoVelocityEstimatorRos::reconfigureCallback, this, _1, _2));

  run_without_trigger = false;
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "run_without_trigger", run_without_trigger);

  if (run_without_trigger)
    ROS_WARN_STREAM(kPrefix << "Running without radar trigger");

  std::string topic_twist = "twist";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_twist", topic_twist);

  std::string topic_radar_scan = "/sensor_platform/radar/scan";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_scan", topic_radar_scan);

  std::string topic_radar_trigger = "/sensor_platform/radar/trigger";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_trigger", topic_radar_trigger);

  std::string topic_twist_ego_ground_truth = "/ground_truth/twist_radar";
  getRosParameter(
      nh, kPrefix, RosParameterType::Recommended, "topic_twist_radar_ground_truth", topic_twist_ego_ground_truth);

  sub_radar_scan_ = nh.subscribe<sensor_msgs::PointCloud2>(
      topic_radar_scan, 50, &RadarEgoVelocityEstimatorRos::callbackRadarScan, this);
  sub_radar_trigger_ = nh.subscribe<std_msgs::Header>(
      topic_radar_trigger, 50, &RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this);
  pub_twist_              = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(topic_twist, 5);
  pub_twist_ground_truth_ = nh.advertise<geometry_msgs::TwistStamped>(topic_twist_ego_ground_truth, 5);
}

void RadarEgoVelocityEstimatorRos::runFromRosbag(const std::string& rosbag_path,
                                                 const double bag_start,
                                                 const double bag_duration,
                                                 const double sleep_ms)
{
  rosbag::Bag source_bag;
  source_bag.open(rosbag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(sub_radar_scan_.getTopic());
  topics.push_back(sub_radar_trigger_.getTopic());
  topics.push_back(pub_twist_ground_truth_.getTopic());

  rosbag::View view(source_bag, rosbag::TopicQuery(topics));

  auto first_timestamp = ros::TIME_MIN;

  for (const rosbag::MessageInstance& m : view)
  {
    if (!ros::ok())
      break;

    if (first_timestamp == ros::TIME_MIN)
      first_timestamp = m.getTime();

    if ((m.getTime() - first_timestamp).toSec() < bag_start)
      continue;

    if ((m.getTime() - first_timestamp).toSec() > bag_duration)
      break;

    const auto topic = m.getTopic();
    if (topic == sub_radar_scan_.getTopic())
    {
      const auto radar_scan = m.instantiate<sensor_msgs::PointCloud2>();
      if (radar_scan != NULL)
      {
        callbackRadarScan(radar_scan);
        if (sleep_ms > 0)
          ros::Duration(sleep_ms / 1.0e3).sleep();
      }
    }
    else if (topic == sub_radar_trigger_.getTopic())
    {
      const auto radar_trigger_msg = m.instantiate<std_msgs::Header>();
      if (radar_trigger_msg != NULL)
        callbackRadarTrigger(radar_trigger_msg);
    }
    else if (topic == pub_twist_ground_truth_.getTopic())
    {
      const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
      if (msg)
        pub_twist_ground_truth_.publish(msg);
    }

    ros::spinOnce();
  }

  ROS_INFO("%s Final Runtime statistics: %s",
           kPrefix.c_str(),
           profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::processRadarData(const sensor_msgs::PointCloud2& radar_scan,
                                                    const ros::Time& trigger_stamp)
{
  Vector3 v_b_r;
  Matrix3 P_v_b_r;
  profiler.start("ego_velocity_estimation");
  if (estimator_.estimate(radar_scan, v_b_r, P_v_b_r))
  {
    profiler.stop("ego_velocity_estimation");

    geometry_msgs::TwistWithCovarianceStamped msg;
    msg.header.stamp         = trigger_stamp;
    msg.header.frame_id      = (radar_scan.header.frame_id.empty())? "radar" : radar_scan.header.frame_id;
    msg.twist.twist.linear.x = v_b_r.x();
    msg.twist.twist.linear.y = v_b_r.y();
    msg.twist.twist.linear.z = v_b_r.z();

    for (uint l = 0; l < 3; ++l)
      for (uint k = 0; k < 3; ++k) msg.twist.covariance.at(l * 6 + k) = P_v_b_r(l, k);
    pub_twist_.publish(msg);
  }
  else
  {
    profiler.stop("ego_velocity_estimation");
    ROS_ERROR_STREAM(kPrefix << "Radar ego velocity estimation failed");
  }

  ROS_INFO_THROTTLE(5,
                    "%s Runtime statistics: %s",
                    kPrefix.c_str(),
                    profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_scan_msg)
{
  mutex_.lock();

  if (run_without_trigger)
  {
    // no trigger available --> use most recent omege measurement
    // catch bug of ti_mmwave driver --> time stamp is 0 :(
    if (radar_scan_msg->header.stamp.sec == 0)
    {
      ROS_WARN_THROTTLE(1.0, "Time stamp of radar scan pcl is 0 using current ros time!");
      processRadarData(*radar_scan_msg, ros::Time::now());
    }
    else
    {
      processRadarData(*radar_scan_msg, radar_scan_msg->header.stamp);
    }
  }
  else
  {
    if (trigger_stamp > ros::TIME_MIN)
      processRadarData(*radar_scan_msg, trigger_stamp);
    else
      ROS_ERROR_STREAM(kPrefix << "Unable to process radar scan, no trigger message received!");
    trigger_stamp = ros::TIME_MIN;
  }

  mutex_.unlock();
}

void RadarEgoVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::HeaderConstPtr& trigger_msg)
{
  mutex_.lock();
  trigger_stamp = trigger_msg->stamp;
  mutex_.unlock();
}
