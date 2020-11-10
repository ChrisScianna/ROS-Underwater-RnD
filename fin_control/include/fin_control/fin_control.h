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
 * fin_control.h
 */

#ifndef FIN_CONTROL_H
#define FIN_CONTROL_H

#define NUM_FINS 4
#define NODE_VERSION "1.8x"

#include <pthread.h>
#include <stdio.h>
#include <sys/select.h>
#include <string>

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <diagnostic_tools/health_check.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>
#include <fin_control/EnableReportAngles.h>
#include <fin_control/ReportAngle.h>
#include <fin_control/SetAngle.h>
#include <fin_control/SetAngles.h>
#include <health_monitor/ReportFault.h>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/DynamixelInfo.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

namespace qna
{
namespace robot
{
class FinControl
{
 public:
  explicit FinControl(ros::NodeHandle& nodeHandle);
  virtual ~FinControl();
  ros::Timer reportAngleTimer;
  void reportAngleSendTimeout(const ros::TimerEvent& timer);
  void reportAngles();
  void reportAngle(uint8_t id);

 private:
  boost::shared_ptr<boost::thread> m_thread;

  bool fincontrolEnabled;
  bool servosON;
  bool reportAnglesEnabled;
  bool currentLoggingEnabled;

  void workerFunc();
  void Start();
  void Stop();
  float radiansToDegrees(float radians);
  float degreesToRadians(float degrees);
  void handleSetAngle(const fin_control::SetAngle::ConstPtr& msg);
  void handleSetAngles(const fin_control::SetAngles::ConstPtr& msg);
  void handleEnableReportAngles(const fin_control::EnableReportAngles::ConstPtr& msg);

  double maxCtrlFinAngle;
  double ctrlFinOffset;
  double ctrlFinScaleFactor;

  double reportAngleRate;
  double minReportAngleRate;
  double maxReportAngleRate;

  ros::NodeHandle& nodeHandle;
  ros::Subscriber subscriber_setAngle;
  ros::Subscriber subscriber_setAngles;
  ros::Subscriber subscriber_enableReportAngles;
  ros::Publisher publisher_reportAngle;

  DynamixelWorkbench myWorkBench;

  uint8_t ids[10];
  uint8_t numOfIDs;

  boost::mutex m_mutex;

  diagnostic_tools::HealthCheck<double> finAngleCheck;
  diagnostic_updater::Updater diagnosticsUpdater;
};

}  // namespace robot
}  // namespace qna

#endif  // FIN_CONTROL_H
