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
 * fin_control.cpp
 *
 */

#include "fin_control/fin_control.h"

#include <string>

#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>


namespace qna
{
namespace robot
{

FinControl::FinControl(ros::NodeHandle &nodeHandle)
    : nodeHandle(nodeHandle), diagnosticsUpdater(nodeHandle)
{
  diagnosticsUpdater.setHardwareID("fin_control");

  fincontrolEnabled = false;
  std::string serial_dev = "/dev/ttyUSB0";
  int serial_baud = 57600;
  ctrlFinOffset = 0;
  ctrlFinScaleFactor = 1.0;
  servosON = false;
  const char *log;
  currentLoggingEnabled = false;
  reportAngleRate = 25.0;
  minReportAngleRate = reportAngleRate / 2.0;
  maxReportAngleRate = reportAngleRate * 2.0;

  ros::NodeHandle nh;

  ROS_INFO("Starting fin_control node Version: [%s]", NODE_VERSION);
  nh.setParam("/version_numbers/fin_control", NODE_VERSION);

  if (nh.getParam("/fin_control/port", serial_dev) == 0)
    ROS_INFO("Parmeter Not found defaullting Serial Port: [%s]", serial_dev.c_str());

  nh.getParam("/fin_control/baud", serial_baud);

  ROS_INFO("Serial Port: [%s]", serial_dev.c_str());
  ROS_INFO("baud: [%d]", serial_baud);

  maxCtrlFinAngle = degreesToRadians(10);
  nh.getParam("/fin_control/max_ctrl_fin_angle", maxCtrlFinAngle);
  ROS_INFO("max cf angle: [%f] rad", maxCtrlFinAngle);

  nh.getParam("/fin_control/ctrl_fin_offset", ctrlFinOffset);
  ROS_INFO("cf offset: [%f] rad", ctrlFinOffset);

  nh.getParam("/fin_control/ctrl_fin_scale_factor", ctrlFinScaleFactor);
  ROS_INFO("cf scale: [%f]", ctrlFinScaleFactor);

  nh.getParam("/fin_control/current_logging_enabled", currentLoggingEnabled);
  if (currentLoggingEnabled) ROS_INFO("FIN CURRENT LOGGING ENABLED");

  nh.getParam("/fin_control/report_angle_rate", reportAngleRate);
  nh.getParam("/fin_control/min_report_angle_rate", minReportAngleRate);
  nh.getParam("/fin_control/max_report_angle_rate", maxReportAngleRate);

  ROS_INFO("fin control constructor enter");

  subscriber_setAngle =
      nodeHandle.subscribe("/fin_control/set_angle", 10, &FinControl::handleSetAngle, this);
  subscriber_setAngles =
      nodeHandle.subscribe("/fin_control/set_angles", 10, &FinControl::handleSetAngles, this);

  publisher_reportAngle =
      diagnostic_tools::create_publisher<fin_control::ReportAngle>(
          nodeHandle, "/fin_control/report_angle", 1);

  diagnostic_tools::PeriodicMessageStatusParams report_angle_rate_check_params;
  report_angle_rate_check_params.min_acceptable_period(minReportAngleRate);
  report_angle_rate_check_params.max_acceptable_period(maxReportAngleRate);
  report_angle_rate_check_params.abnormal_diagnostic(diagnostic_tools::Diagnostic::WARN);
  report_angle_rate_check_params.stale_diagnostic({  // NOLINT(whitespace/braces)
      diagnostic_tools::Diagnostic::STALE, health_monitor::ReportFault::FIN_DATA_STALE
  });  // NOLINT(whitespace/braces)
  diagnosticsUpdater.add(
      publisher_reportAngle.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check", report_angle_rate_check_params));

  timer_reportAngle = nodeHandle.createTimer(
      ros::Duration(1. / reportAngleRate), &FinControl::reportAngleSendTimeout, this);

  finAngleCheck = diagnostic_tools::create_health_check<double>(
      "Fin angle within range",
      [this](double angle) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        if (std::abs(angle) > maxCtrlFinAngle)
        {
          using health_monitor::ReportFault;
          return Diagnostic(Diagnostic::WARN, ReportFault::FIN_DATA_THRESHOLD_REACHED)
              .description("Fin angle beyond limits: |%f| rad > %f rad", angle, maxCtrlFinAngle);
        }
        return Diagnostic::OK;
      });  // NOLINT

  diagnosticsUpdater.add(finAngleCheck);

  if (myWorkBench.begin(serial_dev.c_str(), serial_baud, &log))
  {
    ROS_INFO("Dynamixel Workbench intialized");
  }
  else
  {
    ROS_ERROR("Dynamixel Workbench failed init %s", log);
    return;
  }

  numOfIDs = 0;

  if (myWorkBench.scan(ids, &numOfIDs, 1, 4, &log))
  {
    ROS_INFO("num of ids found [%d]", numOfIDs);
    if (numOfIDs < NUM_FINS)
      ROS_WARN("Fin servos found: [%d] does not match expected number of fins [%d]", numOfIDs,
               NUM_FINS);
  }
  else
  {
    ROS_ERROR("Servo scan failed! %s", log);
    return;
  }
  // power on all servos
  for (int x = 0; x < numOfIDs; x++)
  {
    if (!myWorkBench.torqueOn(ids[x], &log))
    {
      ROS_ERROR("Could not set torque on fin sevro [%d] %s", ids[x], log);
      return;
    }
    else
    {
      servosON = true;
    }
  }

  if (!myWorkBench.initBulkWrite(&log))
  {
    ROS_ERROR("Could not init bulk write %s", log);
    return;
  }

  ros::spin();
}

FinControl::~FinControl()
{
  const char *log;

  if (servosON)
  {
    ROS_INFO("Turning off fin servos");
    // power off all servos
    for (int x = 0; x < numOfIDs; x++)
    {
      if (!myWorkBench.torqueOff(ids[x], &log))
        ROS_ERROR("Could not turn off torque on fin sevro [%d] %s", ids[x], log);
    }
  }
}

void FinControl::reportAngles()
{
  const char *log;
  fin_control::ReportAngle message;
  bool servos_angles_info_complete = true;
  float radianAngleFromMyWorkBench = 0.;

  message.header.stamp = ros::Time::now();

  for (int id = 1; id <= numOfIDs; ++id)
  {
    // get from dynamixel
    if (myWorkBench.getRadian(id, &radianAngleFromMyWorkBench, &log))
    {
      message.angles_in_radians.push_back(
          static_cast<float>(
        round(((radianAngleFromMyWorkBench / ctrlFinScaleFactor) - ctrlFinOffset) *
              100.0) /
        100.0));
      finAngleCheck.test(message.angles_in_radians.back());
    }
    else
    {
      servos_angles_info_complete = false;
      ROS_ERROR("Could not get servo angle for ID %d %s", id, log);
    }
  }
  if (servos_angles_info_complete)
  {
    publisher_reportAngle.publish(message);
  }
}

float FinControl::degreesToRadians(float degrees) { return ((degrees / 180.0) * M_PI); }

void FinControl::handleSetAngles(const fin_control::SetAngles::ConstPtr &msg)
{
  const char *log;
  float angle_plus_offset;

  // check for max angle the fins can mechanically handle
  for (uint8_t i = 0; i < 4; i++)
  {
    if (fabs(msg->fin_angle_in_radians[i]) > maxCtrlFinAngle)
    {
      ROS_ERROR_STREAM("The Angle " << i << "to be set is out of range - value: "
                                    << msg->fin_angle_in_radians[i]);
      return;
    }
  }

  boost::mutex::scoped_lock lock(m_mutex);

  if (!myWorkBench.initBulkWrite(&log))
  {
    ROS_ERROR("Could not init bulk write %s", log);
    return;
  }

  // check for max angle the fins can mechanically handle
  for (uint8_t i = 0; i < 4; i++)
  {
    angle_plus_offset = ctrlFinScaleFactor * (msg->fin_angle_in_radians[i] + ctrlFinOffset);

    if (!(myWorkBench.addBulkWriteParam(i + 1, "Goal_Position",
                                        myWorkBench.convertRadian2Value(i + 1, angle_plus_offset),
                                        &log)))
    {
      ROS_ERROR_STREAM("Could not add bulk write param " << i + 1 << " " << log);
      return;
    }

    if (!myWorkBench.bulkWrite(&log))
    {
      ROS_ERROR("Could not bulk write %s", log);
    }

    if (currentLoggingEnabled)
    {
      int32_t cdata1, cdata2, cdata3, cdata4;
      cdata1 = cdata2 = cdata3 = cdata4 = 0;

      if (!myWorkBench.itemRead(1, "Present_Current", &cdata1, &log))
      {
        ROS_ERROR("Could Read F1 Current %s", log);
      }
      else if (!myWorkBench.itemRead(2, "Present_Current", &cdata2, &log))
      {
        ROS_ERROR("Could Read F2 Current %s", log);
      }
      else if (!myWorkBench.itemRead(3, "Present_Current", &cdata3, &log))
      {
        ROS_ERROR("Could Read F3 Current %s", log);
      }
      else if (!myWorkBench.itemRead(4, "Present_Current", &cdata4, &log))
      {
        ROS_ERROR("Could Read F4 Current %s", log);
      }
      else
      {
        ROS_INFO("Current for fin 1-4 %f,%f,%f,%f",
                 myWorkBench.convertValue2Current((int16_t)cdata1),
                 myWorkBench.convertValue2Current((int16_t)cdata2),
                 myWorkBench.convertValue2Current((int16_t)cdata3),
                 myWorkBench.convertValue2Current((int16_t)cdata4));
      }
    }
  }
}
  void FinControl::handleSetAngle(const fin_control::SetAngle::ConstPtr &msg)
  {
    // check for max angle the fins can mechanically handle
    if (fabs(msg->angle_in_radians) > maxCtrlFinAngle)
    {
      ROS_ERROR("Angle set is out of range: %f rad", msg->angle_in_radians);
      return;
    }

    float angle_plus_offet = ctrlFinScaleFactor * (msg->angle_in_radians + ctrlFinOffset);

    // Call dynamixel service
    boost::mutex::scoped_lock lock(m_mutex);

    const char *log;
    if (!(myWorkBench.goalPosition(msg->ID, angle_plus_offet, &log)))
      ROS_ERROR("Could not set servo angle for ID %d %s", msg->ID, log);
  }

  void FinControl::reportAngleSendTimeout(const ros::TimerEvent &ev)
  {
    reportAngles();
    diagnosticsUpdater.update();
  }

}  // namespace robot
}  // namespace qna
