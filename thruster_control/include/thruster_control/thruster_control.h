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

#ifndef THRUSTER_CONTROL_THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_THRUSTER_CONTROL_H

#include <ros/ros.h>
#include <cmath>
#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_tools/health_check.h>
#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <health_monitor/ReportFault.h>
#include <thruster_control/ReportMotorTemperature.h>
#include <thruster_control/ReportRPM.h>
#include <thruster_control/SetRPM.h>
#include "CANIntf.h"


namespace qna
{
namespace robot
{
class ThrusterControl
{
 public:
  explicit ThrusterControl(ros::NodeHandle& nodeHandle);
  virtual ~ThrusterControl();

  ros::Timer reportRPMTimer;
  void reportRPMSendTimeout(const ros::TimerEvent& timer);

  ros::Timer reportMotorTempTimer;
  void reportMotorTempSendTimeout(const ros::TimerEvent& timer);

  CANIntf canIntf;

 private:
  double setRPMTimeout;
  ros::Time lastSetRPMCommandTime;

  double reportRPMRate;
  double minReportRPMRate;
  double maxReportRPMRate;
  double reportMotorTemperatureRate;
  double minReportMotorTemperatureRate;
  double maxReportMotorTemperatureRate;
  bool currentLoggingEnabled;
  double motorTemperatureThreshold;
  double maxAllowedMotorRPM;

  void handle_SetRPM(const thruster_control::SetRPM::ConstPtr& msg);

  ros::NodeHandle& nodeHandle;

  ros::Subscriber subscriber_setRPM;

  diagnostic_tools::DiagnosedPublisher<thruster_control::ReportRPM> publisher_reportRPM;
  diagnostic_tools::DiagnosedPublisher<thruster_control::ReportMotorTemperature>
      publisher_reportMotorTemp;

  diagnostic_tools::HealthCheck<double> motorRPMCheck;
  diagnostic_tools::HealthCheck<double> motorTemperatureCheck;
  diagnostic_updater::Updater diagnosticsUpdater;

  double convertRadSecToRPM(double radsec);
  double convertRPMToRadSec(double rpms);
};
}  // namespace robot
}  // namespace qna

#endif  // THRUSTER_CONTROL_THRUSTER_CONTROL_H
