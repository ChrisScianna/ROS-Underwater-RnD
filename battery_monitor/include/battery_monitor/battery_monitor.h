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

// Original version: Christopher Scianna <Christopher.Scianna@us.QinetiQ.com>

/*
 * battery_monitor.h
 */

#ifndef BATTERY_MONITOR_BATTERY_MONITOR_H
#define BATTERY_MONITOR_BATTERY_MONITOR_H

#include <battery_monitor/ReportBatteryInfo.h>
#include <battery_monitor/ReportLeakDetected.h>
#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_tools/health_check.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>
#include <health_monitor/ReportFault.h>
#include <ros/ros.h>

#include <cmath>

#include "CANIntf.h"

namespace qna
{
namespace robot
{
class BatteryMonitor
{
 public:
  explicit BatteryMonitor(ros::NodeHandle& nodeHandle);
  virtual ~BatteryMonitor();

 private:
  void batteryCheckTimeout(const ros::TimerEvent &timer);

  void checkBatteryInfo();
  void checkBatteryLeaks();

  CANIntf canIntf;

  ros::NodeHandle &nodeHandle;

  ros::Timer batteryCheckTimer;

  diagnostic_tools::DiagnosedPublisher<battery_monitor::ReportBatteryInfo>
      publisher_reportBatteryInfo;
  diagnostic_tools::DiagnosedPublisher<battery_monitor::ReportLeakDetected>
      publisher_reportLeakDetected;

  diagnostic_tools::HealthCheck<BatteryInfo> batteryCurrentCheck;
  diagnostic_tools::HealthCheck<BatteryInfo> batteryVoltageCheck;
  diagnostic_tools::HealthCheck<BatteryInfo> batteryTemperatureCheck;
  diagnostic_tools::HealthCheck<bool> batteryLeakCheck;
  diagnostic_updater::Updater diagnosticsUpdater;
};

}  // namespace robot
}  // namespace qna

#endif  // BATTERY_MONITOR_BATTERY_MONITOR_H
