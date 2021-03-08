/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com

#ifndef POSE_ESTIMATOR_POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_POSE_ESTIMATOR_H

#include <auv_interfaces/State.h>
#include <auv_interfaces/StateStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <thruster_control/ReportRPM.h>

#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_tools/health_check.h>
#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>
#include <diagnostic_updater/diagnostic_updater.h>


namespace qna
{
namespace robot
{

#define NODE_VERSION "3.0x"
// Version log
// 2.0 Initial MK-IV version
// 2.01 Updating the roll positive direction
// 3.0 Refactor to publish auv state

class PoseEstimator
{
public:
  PoseEstimator();

  bool spin();

private:
  void dataCallback(
      const sensor_msgs::Imu::ConstPtr& ahrs_msg,
      const sensor_msgs::FluidPressure::ConstPtr& pressure_msg,
      const thruster_control::ReportRPM::ConstPtr& rpms_msg);

  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Enviornmental Flags
  bool in_saltwater_;
  double rpm_per_kn_;

  message_filters::Subscriber<sensor_msgs::Imu> ahrs_sub_;
  message_filters::Subscriber<sensor_msgs::FluidPressure> pressure_sub_;
  message_filters::Subscriber<thruster_control::ReportRPM> rpms_sub_;

  using DataPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Imu, sensor_msgs::FluidPressure, thruster_control::ReportRPM>;
  message_filters::Synchronizer<DataPolicy> synchronizer_;

  ros::Subscriber fix_sub_;
  sensor_msgs::NavSatFix::ConstPtr last_fix_msg_;

  diagnostic_tools::DiagnosedPublisher<
    auv_interfaces::StateStamped> state_pub_;

  diagnostic_tools::HealthCheck<double> depth_check_;
  diagnostic_tools::HealthCheck<double> orientation_roll_check_;
  diagnostic_tools::HealthCheck<double> orientation_pitch_check_;
  diagnostic_tools::HealthCheck<double> orientation_yaw_check_;
  diagnostic_updater::Updater diagnostics_updater_;
  ros::Timer diagnostics_timer_;
};

}  // namespace robot
}  // namespace qna

#endif  //  POSE_ESTIMATOR_POSE_ESTIMATOR_H
