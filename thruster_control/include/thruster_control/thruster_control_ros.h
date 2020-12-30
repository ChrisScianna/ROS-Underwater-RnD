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

/*
 * thruster_control.h
 */

#ifndef THRUSTER_CONTROL_THRUSTER_CONTROL_ROS_H
#define THRUSTER_CONTROL_THRUSTER_CONTROL_ROS_H

#include <cmath>
#include <memory>

#include <ros/ros.h>
#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_tools/health_check.h>
#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/BatteryState.h>
#include <health_monitor/ReportFault.h>
#include <thruster_control/ReportMotorTemperature.h>
#include <thruster_control/ReportRPM.h>
#include <thruster_control/SetRPM.h>
#include "thruster_control/CANIntf.h"
#include "thruster_control/thruster_control.h"

namespace qna
{
namespace robot
{
std::unique_ptr<CANIntf> ConfigureCANInterfaceUsingROSParams(ros::NodeHandle nh);

class ThrusterControlROS
{
public:
  ThrusterControlROS();
  ThrusterControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ThrusterControlROS(std::unique_ptr<ThrusterControlInterface> thruster_control);
  ThrusterControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                     std::unique_ptr<ThrusterControlInterface> thruster_control);

  void MonitorMotorRPMUsing(diagnostic_updater::Updater* updater);
  void MonitorMotorTemperatureUsing(diagnostic_updater::Updater* updater);
  void MonitorBatteryHealthUsing(diagnostic_updater::Updater* updater);
  void MonitorUsing(diagnostic_updater::Updater* updater);

private:
  void OnReportMotorRPMTimeout(const ros::TimerEvent& event);
  void OnReportMotorTemperatureTimeout(const ros::TimerEvent& event);
  void OnReportBatteryHealthTimeout(const ros::TimerEvent& event);
  void OnMotorRPMSet(const thruster_control::SetRPM::ConstPtr& message);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<ThrusterControlInterface> thruster_control_;

  ros::Subscriber motor_rpm_subscriber_;
  diagnostic_tools::DiagnosedPublisher<
    thruster_control::ReportRPM> motor_rpm_publisher_;
  diagnostic_tools::DiagnosedPublisher<
    thruster_control::ReportMotorTemperature> motor_temperature_publisher_;
  diagnostic_tools::DiagnosedPublisher<
    sensor_msgs::BatteryState> battery_health_publisher_;

  double report_motor_rpm_period_;
  ros::Timer report_motor_rpm_timer_;

  double report_battery_health_period_;
  ros::Timer report_motor_temperature_timer_;

  double report_motor_temperature_period_;
  ros::Timer report_battery_health_timer_;

  diagnostic_tools::TopicDiagnosticTask<
    thruster_control::ReportRPM>* report_motor_rpm_rate_check_{nullptr};
  diagnostic_tools::HealthCheck<double> motor_rpm_check_;

  diagnostic_tools::TopicDiagnosticTask<
    thruster_control::ReportMotorTemperature>* motor_temperature_rate_check_{nullptr};
  diagnostic_tools::TopicDiagnosticTask<
    thruster_control::ReportMotorTemperature>* motor_temperature_stagnation_check_{nullptr};
  diagnostic_tools::HealthCheck<double> motor_temperature_check_;

  diagnostic_tools::TopicDiagnosticTask<
    sensor_msgs::BatteryState>* battery_health_rate_check_{nullptr};
  diagnostic_tools::TopicDiagnosticTask<
    sensor_msgs::BatteryState>* battery_health_stagnation_check_{nullptr};
  diagnostic_tools::HealthCheck<double> battery_total_current_check_;
  diagnostic_tools::HealthCheck<double> battery_cell_voltage_check_;
};

}  // namespace robot
}  // namespace qna

#endif  // THRUSTER_CONTROL_THRUSTER_CONTROL_ROS_H
