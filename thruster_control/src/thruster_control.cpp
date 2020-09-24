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
 * thruster_control.cpp
 *
 */

#include "thruster_control.h"

#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>

namespace qna {
namespace robot {

const double RPM_TO_RADSEC = 3.1415 / 30.0;
const double RADSEC_TO_RPM = 30.0 / 3.1415;

ThrusterControl::ThrusterControl(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle), diagnosticsUpdater(nodeHandle) {
  diagnosticsUpdater.setHardwareID("thruster");

  reportRPMRate = 0.1;
  reportMotorTemperatureRate = 0.5;
  setRPMTimeout = 0.5;  // 0.5 seconds, 500 milliseconds
  currentLoggingEnabled = false;
  std::string canNodeId1 = "";  // TODO what should the default be??
  std::string canNodeId2 = "";
  motorTemperatureThreshold = 50;
  maxAllowedMotorRPM = 0;

  nodeHandle.getParam("/thruster_control_node/report_rpm_rate", reportRPMRate);
  nodeHandle.getParam("/thruster_control_node/report_motor_temperature_rate",
                      reportMotorTemperatureRate);
  nodeHandle.getParam("/thruster_control_node/set_rpm_timeout_seconds", setRPMTimeout);
  nodeHandle.getParam("/thruster_control_node/current_logging_enabled", currentLoggingEnabled);
  nodeHandle.getParam("/thruster_control_node/can_node_id_1", canNodeId1);
  nodeHandle.getParam("/thruster_control_node/can_node_id_2", canNodeId2);
  nodeHandle.getParam("/thruster_control_node/max_allowed_motor_rpm", maxAllowedMotorRPM);
  nodeHandle.getParam("/thruster_control_node/motor_temperature_threshold",
                      motorTemperatureThreshold);

  ROS_INFO("report_rpm_rate set to %lf", reportRPMRate);
  ROS_INFO("report_motor_temperatuere_rate set to %lf", reportMotorTemperatureRate);
  ROS_INFO("set_rpm_timeout_seconds set to %lf", setRPMTimeout);
  ROS_INFO("current_logging_enabled set to %s", currentLoggingEnabled ? "true" : "false");
  ROS_INFO("CAN Node 1 Id = %s", canNodeId1);
  ROS_INFO("CAN Node 2 Id = %s", canNodeId2);

  subscriber_setRPM =
      nodeHandle.subscribe("/thruster_control/set_rpm", 1, &ThrusterControl::handle_SetRPM, this);

  canIntf.SetMotorTimeoutSeconds(setRPMTimeout);
  canIntf.SetEnableCANLogging(currentLoggingEnabled);
  canIntf.AddCanNodeIdToList(canNodeId1);
  canIntf.AddCanNodeIdToList(canNodeId2);

  publisher_reportRPM = diagnostic_tools::create_publisher<thruster_control::ReportRPM>(
      nodeHandle, "/thruster_control/report_rpm", 1);

  diagnosticsUpdater.add(publisher_reportRPM.add_check<diagnostic_tools::PeriodicMessageStatus>(
      "rate check", diagnostic_tools::PeriodicMessageStatusParams{}
                        .min_acceptable_period(reportRPMRate / 2)
                        .max_acceptable_period(reportRPMRate / 2)
                        .abnormal_diagnostic({diagnostic_tools::Diagnostic::WARN,
                                              health_monitor::ReportFault::THRUSTER_RPM_STALE})));

  publisher_reportMotorTemp =
      diagnostic_tools::create_publisher<thruster_control::ReportMotorTemperature>(
          nodeHandle, "/thruster_control/report_motor_temperature", 1);

  diagnosticsUpdater.add(
      publisher_reportMotorTemp.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check", diagnostic_tools::PeriodicMessageStatusParams{}
                            .min_acceptable_period(reportMotorTemperatureRate / 2)
                            .max_acceptable_period(reportMotorTemperatureRate * 2)
                            .abnormal_diagnostic({diagnostic_tools::Diagnostic::ERROR,
                                                  health_monitor::ReportFault::THRUSTER_TEMP_STALE})));

  diagnosticsUpdater.add(
      publisher_reportMotorTemp.add_check<diagnostic_tools::MessageStagnationCheck>(
          "stagnation check", [](const thruster_control::ReportMotorTemperature& a,
                                 const thruster_control::ReportMotorTemperature& b) {
            return a.motor_temp == b.motor_temp;
          },
          diagnostic_tools::MessageStagnationCheckParams{}
            .stagnation_diagnostic({diagnostic_tools::Diagnostic::WARN,
                                    health_monitor::ReportFault::THRUSTER_TEMP_STAGNATED})));

  canIntf.SetMotorTimeoutSeconds(setRPMTimeout);
  canIntf.SetEnableCANLogging(currentLoggingEnabled);

  motorRPMCheck = diagnostic_tools::create_health_check<double>(
      "Motor RPM check", [this](double rpms) -> diagnostic_tools::Diagnostic {
        using diagnostic_tools::Diagnostic;
        if (std::abs(rpms) < maxAllowedMotorRPM) {
          return Diagnostic{Diagnostic::WARN,
                            health_monitor::ReportFault::THRUSTER_RPM_THRESHOLD_REACHED}
              .description("Absolute RPMs above threshold: |%f| > %f", rpms, maxAllowedMotorRPM);
        }
        return Diagnostic::OK;
      });
  diagnosticsUpdater.add(motorRPMCheck);

  motorTemperatureCheck = diagnostic_tools::create_health_check<double>(
      "Motor temperature check", [this](double temperature) -> diagnostic_tools::Diagnostic {
        using diagnostic_tools::Diagnostic;
        if (temperature > motorTemperatureThreshold) {
          return Diagnostic{Diagnostic::ERROR}.description(
              "Temperature above threshold: %f degC > %f degC",
              temperature, motorTemperatureThreshold);
        }
        return Diagnostic::OK;
      });
  diagnosticsUpdater.add(motorTemperatureCheck);

  reportRPMTimer = nodeHandle.createTimer(ros::Duration(reportRPMRate),
                                          &ThrusterControl::reportRPMSendTimeout, this);
  reportMotorTempTimer = nodeHandle.createTimer(ros::Duration(reportMotorTemperatureRate),
                                                &ThrusterControl::reportMotorTempSendTimeout, this);

  canIntf.Init();
}

ThrusterControl::~ThrusterControl() {}

void ThrusterControl::reportRPMSendTimeout(const ros::TimerEvent& timer) {
  thruster_control::ReportRPM message;

  message.header.stamp = ros::Time::now();
  message.rpms = convertRadSecToRPM(canIntf.velocity_feedback_radsec.Get());
  motorRPMCheck.test(message.rpms);

  publisher_reportRPM.publish(message);
  diagnosticsUpdater.update();
}

void ThrusterControl::reportMotorTempSendTimeout(const ros::TimerEvent& timer) {
  thruster_control::ReportMotorTemperature message;

  message.header.stamp = ros::Time::now();
  message.motor_temp = canIntf.motor_tempC.Get();
  motorTemperatureCheck.test(message.motor_temp);

  publisher_reportMotorTemp.publish(message);
  diagnosticsUpdater.update();
}

void ThrusterControl::handle_SetRPM(const thruster_control::SetRPM::ConstPtr& msg) {
  double motor_radsec = convertRPMToRadSec(msg->commanded_rpms);
  canIntf.velocity_radsec.Set(motor_radsec);
  canIntf.last_set_rpm_time.Set(ros::Time::now().toSec());
}

double ThrusterControl::convertRPMToRadSec(double rpms) { return (rpms * RPM_TO_RADSEC); }

double ThrusterControl::convertRadSecToRPM(double radsec) { return (radsec * RADSEC_TO_RPM); }

}  // namespace robot
}  // namespace qna
