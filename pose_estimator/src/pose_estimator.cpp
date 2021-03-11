/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Original version: Christopher Scianna <Christopher.Scianna@us.QinetiQ.com>

#include "pose_estimator/pose_estimator.h"

#include <cmath>
#include <limits>

#include <auv_interfaces/CartesianPose.h>
#include <auv_interfaces/helpers.h>
#include <health_monitor/ReportFault.h>
#include <sensor_msgs/NavSatStatus.h>


namespace qna
{
namespace robot
{

PoseEstimator::PoseEstimator()
    : pnh_("~"),
      diagnostics_updater_(nh_)
{
  diagnostics_updater_.setHardwareID("pose_estimator");

  // Get runtime parameters
  double min_rate, max_rate;
  pnh_.param("min_rate", min_rate, 1.);  // Hz
  pnh_.param("max_rate", max_rate, 50.);  // Hz

  double absolute_steady_band;
  double relative_steady_band;
  pnh_.param("absolute_steady_band", absolute_steady_band, 0.);
  pnh_.param("relative_steady_band", relative_steady_band, 0.);

  constexpr double inf = std::numeric_limits<double>::infinity();
  double max_depth, max_roll_angle, max_pitch_angle, max_yaw_angle;
  pnh_.param("max_depth", max_depth, inf);
  pnh_.param("max_roll_angle", max_roll_angle, inf);
  pnh_.param("max_pitch_angle", max_pitch_angle, inf);
  pnh_.param("max_yaw_angle", max_yaw_angle, inf);

  pnh_.param("in_saltwater", in_saltwater_, false);
  pnh_.param("rpm_per_knot", rpm_per_kn_, 303.0);

  // Subscribe to all topics
  bool use_ins;
  pnh_.param("use_ins", use_ins, false);
  if (!use_ins)
  {
    ahrs_sub_ = nh_.subscribe("imu/data", 1, &PoseEstimator::ahrsCallback, this);
    pressure_sub_ = nh_.subscribe("pressure", 1, &PoseEstimator::pressureCallback, this);
    rpms_sub_ = nh_.subscribe("rpms", 1, &PoseEstimator::rpmsCallback, this);
    fix_sub_ = nh_.subscribe("fix", 1, &PoseEstimator::fixCallback, this);
  }
  else
  {
    ins_sub_ = nh_.subscribe("ins/state", 1, &PoseEstimator::insCallback, this);
  }

  // Advertise all topics and services
  state_pub_ =
      diagnostic_tools::create_publisher<auv_interfaces::StateStamped>(nh_, "state", 1);

  // Setup diagnostics
  diagnostic_tools::PeriodicMessageStatusParams state_rate_check_params;
  state_rate_check_params.min_acceptable_period(1.0 / max_rate);
  state_rate_check_params.max_acceptable_period(1.0 / min_rate);
  state_rate_check_params.abnormal_diagnostic({  // NOLINT(whitespace/braces)
      diagnostic_tools::Diagnostic::ERROR,
      health_monitor::ReportFault::POSE_DATA_STALE
  });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(
      state_pub_.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check", state_rate_check_params));

  diagnostic_tools::MessageStagnationCheckParams state_stagnation_check_params;
  state_stagnation_check_params.stagnation_diagnostic({  // NOLINT(whitespace/braces)
      diagnostic_tools::Diagnostic::ERROR,
      health_monitor::ReportFault::POSE_DATA_STAGNATED
  });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(state_pub_.add_check<diagnostic_tools::MessageStagnationCheck>(
      "stagnation check",
      [absolute_steady_band, relative_steady_band](const auv_interfaces::StateStamped &a,
                                                   const auv_interfaces::StateStamped &b)
      {
        return auv_interfaces::almost_equal(
            a.state, b.state, absolute_steady_band, relative_steady_band);
      }, state_stagnation_check_params));  // NOLINT

  // Report if message data is out of range
  depth_check_ = diagnostic_tools::create_health_check<double>(
      "Vehicle position - Depth check",
      [max_depth](double depth)
      {
        using diagnostic_tools::Diagnostic;
        using health_monitor::ReportFault;
        if (depth < 0. || depth > max_depth)
        {
          return Diagnostic(Diagnostic::ERROR, ReportFault::POSE_DEPTH_THRESHOLD_REACHED)
              .description("NOT OK - %f m outside [%f m, 0 m] range", depth, max_depth);
        }
        return Diagnostic(Diagnostic::OK).description(
            "OK - %f m within [%f m, 0 m] range", depth, max_depth);
      });  // NOLINT(whitespace/braces)

  diagnostics_updater_.add(depth_check_);

  orientation_roll_check_ =
    diagnostic_tools::create_health_check<double>(
      "Vehicle orientation - Roll angle check",
      [max_roll_angle](double roll) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        using health_monitor::ReportFault;
        Diagnostic diagnostic{Diagnostic::OK};
        if (std::abs(roll) > max_roll_angle)
        {
          diagnostic.status(Diagnostic::ERROR)
              .description("Roll angle - NOT OK -> |%f rad| > %f rad",
                           roll, max_roll_angle)
              .code(ReportFault::POSE_ROLL_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Roll angle - OK (%f)", roll);
        }
        return diagnostic;
      });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(orientation_roll_check_);

  orientation_pitch_check_ =
    diagnostic_tools::create_health_check<double>(
      "Vehicle orientation - Pitch angle check",
      [max_pitch_angle](double pitch) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        using health_monitor::ReportFault;
        Diagnostic diagnostic{Diagnostic::OK};
        if (std::abs(pitch) > max_pitch_angle)
        {
          diagnostic.status(Diagnostic::ERROR)
              .description("Pitch angle - NOT OK -> |%f rad| > %f rad",
                           pitch, max_pitch_angle)
              .code(ReportFault::POSE_PITCH_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Pitch angle - OK (%f)", pitch);
        }
        return diagnostic;
      });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(orientation_pitch_check_);

  orientation_yaw_check_ =
    diagnostic_tools::create_health_check<double>(
      "Vehicle orientation - Yaw angle check",
      [max_yaw_angle](double yaw) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        using health_monitor::ReportFault;
        Diagnostic diagnostic{Diagnostic::OK};
        if (std::abs(yaw) > max_yaw_angle)
        {
          diagnostic.status(Diagnostic::ERROR)
              .description("Yaw angle - NOT OK -> |%f rad| > %f rad",
                           yaw, max_yaw_angle)
              .code(ReportFault::POSE_HEADING_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Yaw angle - OK (%f)", yaw);
        }
        return diagnostic;
      });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(orientation_yaw_check_);

  // Start diagnostics publishing timer
  diagnostics_timer_ = nh_.createTimer(
      ros::Duration(diagnostics_updater_.getPeriod()),
      boost::bind(&diagnostic_updater::Updater::update, &diagnostics_updater_));
}

bool PoseEstimator::spin()
{
  while (ros::ok())
  {
    ros::spin();
  }
}

void PoseEstimator::insCallback(const auv_interfaces::StateStamped::ConstPtr& ins_msg)
{
  state_pub_.publish(ins_msg);  // forward as-is for now
}

namespace
{

// assumes that the pressure is in absolute pascal and a true flag indicates saltwater
double calculateDepth(double pressure, bool saltwater)
{
  constexpr double absolute_pascal_to_gauge_pascal = 101325;
  constexpr double gravity_acceleration = 9.80665;  // m/s^2
  constexpr double freshwater_density = 997.0474;  // kg/m^3
  constexpr double saltwater_density = 1023.6;  // kg/m^3

  const double density = saltwater ? saltwater_density : freshwater_density;
  const double gauge_pressure = pressure - absolute_pascal_to_gauge_pascal;
  return gauge_pressure / gravity_acceleration / density;
}

}  // namespace

void PoseEstimator::ahrsCallback(const sensor_msgs::Imu::ConstPtr& ahrs_msg)
{
  // TODO(hidmic): set frames
  auv_interfaces::StateStamped msg;
  msg.header.stamp = ros::Time::now();

  if (last_fix_msg_)
  {
    // Use GPS for geolocation (should consider validity time?)
    msg.state.geolocation.position.latitude = last_fix_msg_->latitude;
    msg.state.geolocation.position.longitude = last_fix_msg_->longitude;
    msg.state.geolocation.position.altitude = last_fix_msg_->altitude;
    msg.state.geolocation.covariance = last_fix_msg_->position_covariance;
  }

  // Use orientation from AHRS
  tf::Quaternion orientation(
      ahrs_msg->orientation.x, ahrs_msg->orientation.y,
      ahrs_msg->orientation.z, ahrs_msg->orientation.w);

  tf::Matrix3x3 m(orientation);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  msg.state.manoeuvring.pose.mean.orientation.x = roll;
  msg.state.manoeuvring.pose.mean.orientation.y = pitch;
  msg.state.manoeuvring.pose.mean.orientation.z = yaw;
  msg.state.manoeuvring.pose.covariance[21] = ahrs_msg->orientation_covariance[0];
  msg.state.manoeuvring.pose.covariance[22] = ahrs_msg->orientation_covariance[1];
  msg.state.manoeuvring.pose.covariance[23] = ahrs_msg->orientation_covariance[2];
  msg.state.manoeuvring.pose.covariance[27] = ahrs_msg->orientation_covariance[3];
  msg.state.manoeuvring.pose.covariance[28] = ahrs_msg->orientation_covariance[4];
  msg.state.manoeuvring.pose.covariance[29] = ahrs_msg->orientation_covariance[5];
  msg.state.manoeuvring.pose.covariance[33] = ahrs_msg->orientation_covariance[6];
  msg.state.manoeuvring.pose.covariance[34] = ahrs_msg->orientation_covariance[7];
  msg.state.manoeuvring.pose.covariance[35] = ahrs_msg->orientation_covariance[8];

  orientation_roll_check_.test(roll);
  orientation_pitch_check_.test(pitch);
  orientation_yaw_check_.test(yaw);

  if (last_pressure_msg_)
  {
    // Estimate depth from pressure sensor
    double depth = calculateDepth(last_pressure_msg_->fluid_pressure, in_saltwater_);
    msg.state.manoeuvring.pose.mean.position.z = depth;
    msg.state.manoeuvring.pose.covariance[14] = last_pressure_msg_->variance;

    depth_check_.test(depth);
  }

  if (last_rpms_msg_)
  {
    // Estimate velocity from AHRS and thruster RPMs
    const double speed_kn = static_cast<double>(last_rpms_msg_->rpms) / rpm_per_kn_;
    constexpr double kn_per_ms = 1852. / 3600.;  // exact conversion
    const tf::Quaternion nominal_velocity(speed_kn * kn_per_ms, 0., 0., 0.);
    const tf::Quaternion velocity =
      orientation * nominal_velocity * orientation.inverse();
    msg.state.manoeuvring.velocity.mean.linear.x = velocity.x();
    msg.state.manoeuvring.velocity.mean.linear.y = velocity.y();
    msg.state.manoeuvring.velocity.mean.linear.z = velocity.z();
  }

  state_pub_.publish(msg);
}

void PoseEstimator::fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
  {
    ROS_DEBUG("GPS unable to fix position! Ignoring...");
    last_fix_msg_.reset();
  }
  last_fix_msg_ = msg;
}

void PoseEstimator::pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
  last_pressure_msg_ = msg;
}

void PoseEstimator::rpmsCallback(const thruster_control::ReportRPM::ConstPtr& msg)
{
  last_rpms_msg_ = msg;
}

}  // namespace robot
}  // namespace qna
