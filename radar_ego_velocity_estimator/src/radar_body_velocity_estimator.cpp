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

#include <random>
#include <algorithm>
#include <angles/angles.h>

#include <radar_ego_velocity_estimator/ros_helper.h>
#include <radar_ego_velocity_estimator/math_helper.h>
#include <radar_ego_velocity_estimator/radar_body_velocity_estimator.h>

using namespace reve;

RadarBodyVelocityEstimator::RadarBodyVelocityEstimator(ros::NodeHandle nh, const bool load_param_without_reconfigure)
{
  bool success = true;

  Vector3 l_b_r;
  success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "l_b_r_x", l_b_r.x());
  success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "l_b_r_y", l_b_r.y());
  success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "l_b_r_z", l_b_r.z());

  Quaternion q_b_r;
  success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_w", q_b_r.w());
  success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_x", q_b_r.x());
  success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_y", q_b_r.y());
  success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_z", q_b_r.z());

  if (load_param_without_reconfigure)
  {
    // clang-format off
    radar_ego_velocity_estimation::RadarEgoVelocityEstimatorConfig config;
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "min_dist", config.min_dist);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "max_dist", config.max_dist);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "min_db", config.min_db);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "elevation_thresh_deg", config.elevation_thresh_deg);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "azimuth_thresh_deg", config.azimuth_thresh_deg);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "filter_min_z", config.filter_min_z);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "filter_max_z", config.filter_max_z);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "radar_velocity_correction_factor", config.doppler_velocity_correction_factor);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "thresh_zero_velocity", config.thresh_zero_velocity);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "allowed_outlier_percentage", config.allowed_outlier_percentage);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "sigma_zero_velocity_x", config.sigma_zero_velocity_x);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "sigma_zero_velocity_y", config.sigma_zero_velocity_y);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "sigma_zero_velocity_z", config.sigma_zero_velocity_z);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "max_sigma_x", config.max_sigma_x);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "max_sigma_y", config.max_sigma_y);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "max_sigma_z", config.max_sigma_z);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "max_r_cond", config.max_r_cond);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "use_cholesky_instead_of_bdcsvd", config.use_cholesky_instead_of_bdcsvd);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "use_ransac", config.use_ransac);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "outlier_prob", config.outlier_prob);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "success_prob", config.success_prob);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "N_ransac_points", config.N_ransac_points);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "inlier_thresh", config.inlier_thresh);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "sigma_offset_radar_x", config.sigma_offset_radar_x);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "sigma_offset_radar_y", config.sigma_offset_radar_y);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "sigma_offset_radar_z", config.sigma_offset_radar_z);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "use_odr", config.use_odr);
    success &= getRosParameter(nh, kPrefix, RosParameterType::Required, "sigma_v_d", config.sigma_v_d);
    // clang-format on
    configure(config);
  }

  assert(success && "Failed to load all rosparameters --> check error message above");

  T_b_r_.translation() = l_b_r;
  T_b_r_.linear()      = Matrix3(q_b_r);
}

bool RadarBodyVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                          const Vector3& w_b,
                                          Vector3& v_b_r,
                                          Matrix3& P_v_b)
{
  Vector3 v_r;
  Matrix3 P_v_r;

  if (radar_ego_velocity_estimator_.estimate(radar_scan_msg, v_r, P_v_r))
  {
    // v_b & sigma_v_b
    const Vector3 v_b_w = math_helper::skewVec(w_b) * T_b_r_.translation();
    v_b_r               = T_b_r_.linear() * v_r - v_b_w;
    P_v_b               = T_b_r_.linear() * P_v_r * T_b_r_.linear().transpose();

    return true;
  }

  return false;
}
