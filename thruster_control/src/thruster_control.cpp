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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com

/*
 * thruster_control.cpp
 *
 */

#include "thruster_control/thruster_control.h"

#include <string>

namespace qna
{
namespace robot
{
const double RPM_TO_RADSEC = 3.1415 / 30.0;
const double RADSEC_TO_RPM = 30.0 / 3.1415;

ThrusterControl::ThrusterControl(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle), diagnosticsUpdater(nodeHandle)
{
  using health_monitor::ReportFault;

  diagnosticsUpdater.setHardwareID("thruster");

  reportRPMRate = 0.1;
  reportMotorTemperatureRate = 0.5;
  minReportRPMRate = reportRPMRate / 2.0;
  maxReportRPMRate = reportRPMRate * 2.0;
  minReportMotorTemperatureRate = reportMotorTemperatureRate / 2.0;
  maxReportMotorTemperatureRate = reportMotorTemperatureRate * 2.0;
  reportBatteryHealthRate = 1.0;
  minReportBatteryHealthRate = reportBatteryHealthRate / 2.0;
  maxReportBatteryHealthRate = reportBatteryHealthRate * 2.0;
  setRPMTimeout = 0.5;  // 0.5 seconds, 500 milliseconds
  currentLoggingEnabled = false;
  std::string canNodeId1 = "";  // TODO(QNA) what should the default be??
  std::string canNodeId2 = "";
  motorTemperatureThreshold = 50;
  maxAllowedMotorRPM = 0;
  maxBatteryTotalCurrent = 0.0;
  double motorTemperatureSteadyBand = 0.0;
  double batteryCurrentSteadyBand = 0.0;
  double minBatteryCellVoltage = 30;  // in V

  nodeHandle.getParam("/thruster_control_node/report_rpm_rate", reportRPMRate);
  nodeHandle.getParam("/thruster_control_node/min_report_rpm_rate", minReportRPMRate);
  nodeHandle.getParam("/thruster_control_node/max_report_rpm_rate", maxReportRPMRate);
  nodeHandle.getParam("/thruster_control_node/report_motor_temperature_rate",
                      reportMotorTemperatureRate);
  nodeHandle.getParam("/thruster_control_node/min_report_motor_temperature_rate",
                      minReportMotorTemperatureRate);
  nodeHandle.getParam("/thruster_control_node/max_report_motor_temperature_rate",
                      maxReportMotorTemperatureRate);
  nodeHandle.getParam("/thruster_control_node/report_battery_health_rate",
                      reportBatteryHealthRate);
  nodeHandle.getParam("/thruster_control_node/min_report_battery_health_rate",
                      minReportBatteryHealthRate);
  nodeHandle.getParam("/thruster_control_node/max_report_battery_health_rate",
                      maxReportBatteryHealthRate);
  nodeHandle.getParam("/thruster_control_node/battery_current_steady_band",
                      batteryCurrentSteadyBand);
  nodeHandle.getParam("/thruster_control_node/set_rpm_timeout_seconds", setRPMTimeout);
  nodeHandle.getParam("/thruster_control_node/current_logging_enabled", currentLoggingEnabled);
  nodeHandle.getParam("/thruster_control_node/can_node_id_1", canNodeId1);
  nodeHandle.getParam("/thruster_control_node/can_node_id_2", canNodeId2);

  nodeHandle.getParam("/thruster_control_node/max_allowed_motor_rpm", maxAllowedMotorRPM);
  if (maxAllowedMotorRPM > thruster_control::SetRPM::MAX_RPM)
  {
    ROS_WARN_STREAM(
      "Maximum allowed motor RPMs cannot be larger than "
      << thruster_control::SetRPM::MAX_RPM);
    maxAllowedMotorRPM = thruster_control::SetRPM::MAX_RPM;
  }
  ROS_INFO_STREAM("Maximum allowed motor RPMs = " << maxAllowedMotorRPM);

  nodeHandle.getParam("/thruster_control_node/motor_temperature_threshold",
                      motorTemperatureThreshold);
  nodeHandle.getParam("/thruster_control_node/motor_temperature_steady_band",
                      motorTemperatureSteadyBand);
  nodeHandle.getParam("/thruster_control_node/max_battery_total_current", maxBatteryTotalCurrent);
  nodeHandle.getParam("/thruster_control_node/min_battery_cell_voltage", minBatteryCellVoltage);

  ROS_DEBUG_STREAM("report_rpm_rate set to " << reportRPMRate);
  ROS_DEBUG_STREAM("report_motor_temperatuere_rate set to  " << reportMotorTemperatureRate);
  ROS_DEBUG_STREAM("report_battery_health_rate set to  " << reportBatteryHealthRate);
  ROS_DEBUG_STREAM("set_rpm_timeout_seconds set to  " << setRPMTimeout);
  ROS_DEBUG_STREAM("current_logging_enabled set to  " << currentLoggingEnabled ? "true" : "false");
  ROS_DEBUG_STREAM("CAN Node 1 Id =  " << canNodeId1);
  ROS_DEBUG_STREAM("CAN Node 2 Id =  " << canNodeId2);
  ROS_DEBUG_STREAM("Maximum Battery Total Current = " << maxBatteryTotalCurrent);
  ROS_INFO("Minimum Battery Cell Voltage:[%lf]", minBatteryCellVoltage);

  subscriber_setRPM =
      nodeHandle.subscribe("/thruster_control/set_rpm", 1, &ThrusterControl::handle_SetRPM, this);

  canIntf.SetMotorTimeoutSeconds(setRPMTimeout);
  canIntf.SetEnableCANLogging(currentLoggingEnabled);
  canIntf.AddCanNodeIdToList(canNodeId1);
  canIntf.AddCanNodeIdToList(canNodeId2);

  publisher_reportBatteryHealth = diagnostic_tools::create_publisher<sensor_msgs::BatteryState>(
      nodeHandle, "/thruster_control/report_battery_health", 1);

  diagnostic_tools::PeriodicMessageStatusParams paramsReportsBatteryHealthCheckPeriod{};
  paramsReportsBatteryHealthCheckPeriod.min_acceptable_period(minReportBatteryHealthRate);
  paramsReportsBatteryHealthCheckPeriod.max_acceptable_period(maxReportBatteryHealthRate);
  diagnostic_tools::Diagnostic diagnosticBatteryHealthInfoStale(diagnostic_tools::Diagnostic::WARN,
                                                                ReportFault::BATTERY_INFO_STALE);
  paramsReportsBatteryHealthCheckPeriod.abnormal_diagnostic(diagnosticBatteryHealthInfoStale);
  diagnosticsUpdater.add(
      publisher_reportBatteryHealth.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check", paramsReportsBatteryHealthCheckPeriod));

  batteryCurrentCheck = diagnostic_tools::create_health_check<double>(
      "Battery current check", [this](double batteryCurrent) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        if (maxBatteryTotalCurrent < batteryCurrent)
        {
          return Diagnostic(Diagnostic::WARN)
              .code(ReportFault::BATTERY_CURRENT_THRESHOLD_REACHED)
              .description("Total current - NOT OK (%f mA < %f mA)",
                           maxBatteryTotalCurrent, batteryCurrent);
        }
        return Diagnostic::OK;
      }
);
  diagnosticsUpdater.add(batteryCurrentCheck);

  batteryVoltageCheck = diagnostic_tools::create_health_check<double>(
      "Battery Voltage check",
      [minBatteryCellVoltage](double batteryVoltage) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        if (batteryVoltage < minBatteryCellVoltage)
        {
          return Diagnostic(Diagnostic::WARN, ReportFault::BATTERY_VOLTAGE_THRESHOLD_REACHED)
              .description("Battery Votlage - NOT OK (%f V < %f V)", batteryVoltage,
                           minBatteryCellVoltage);
        }
        return Diagnostic::OK;
      }
);
  diagnosticsUpdater.add(batteryVoltageCheck);

  publisher_reportRPM = diagnostic_tools::create_publisher<thruster_control::ReportRPM>(
      nodeHandle, "/thruster_control/report_rpm", 1);
  diagnostic_tools::PeriodicMessageStatusParams paramsReportsRPMCheckPeriod{};
  paramsReportsRPMCheckPeriod.min_acceptable_period(minReportRPMRate);
  paramsReportsRPMCheckPeriod.max_acceptable_period(maxReportRPMRate);
  diagnostic_tools::Diagnostic diagnosticRPMInfoStale(diagnostic_tools::Diagnostic::WARN,
                                                      ReportFault::THRUSTER_RPM_STALE);
  paramsReportsRPMCheckPeriod.abnormal_diagnostic(diagnosticRPMInfoStale);
  diagnosticsUpdater.add(publisher_reportRPM.add_check<diagnostic_tools::PeriodicMessageStatus>(
      "rate check", paramsReportsRPMCheckPeriod));

  publisher_reportMotorTemp =
      diagnostic_tools::create_publisher<thruster_control::ReportMotorTemperature>(
          nodeHandle, "/thruster_control/report_motor_temperature", 1);
  diagnostic_tools::PeriodicMessageStatusParams paramsReportsTemperatureCheckPeriod{};
  paramsReportsTemperatureCheckPeriod.min_acceptable_period(minReportMotorTemperatureRate);
  paramsReportsTemperatureCheckPeriod.max_acceptable_period(maxReportMotorTemperatureRate);
  diagnostic_tools::Diagnostic diagnosticTemperatureInfoStale(diagnostic_tools::Diagnostic::WARN,
                                                              ReportFault::THRUSTER_TEMP_STALE);
  paramsReportsTemperatureCheckPeriod.abnormal_diagnostic(diagnosticTemperatureInfoStale);
  diagnosticsUpdater.add(
      publisher_reportMotorTemp.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check", paramsReportsTemperatureCheckPeriod));

  diagnostic_tools::MessageStagnationCheckParams paramsTemperatureMessageStagnationcheck;
  diagnostic_tools::Diagnostic diagnosticTemperatureInfoStagnate(
      diagnostic_tools::Diagnostic::WARN, ReportFault::THRUSTER_TEMP_STAGNATED);
  paramsTemperatureMessageStagnationcheck.stagnation_diagnostic(diagnosticTemperatureInfoStagnate);

  diagnosticsUpdater.add(
      publisher_reportMotorTemp.add_check<diagnostic_tools::MessageStagnationCheck>(
          "stagnation check",
          [motorTemperatureSteadyBand](const thruster_control::ReportMotorTemperature& a,
                                       const thruster_control::ReportMotorTemperature& b)
          {
            return std::fabs(a.motor_temp - b.motor_temp) < motorTemperatureSteadyBand;
          }
          , paramsTemperatureMessageStagnationcheck));

  canIntf.SetMotorTimeoutSeconds(setRPMTimeout);
  canIntf.SetEnableCANLogging(currentLoggingEnabled);

  motorRPMCheck = diagnostic_tools::create_health_check<double>(
      "Motor RPM check", [this](double rpms) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        if (std::abs(rpms) > maxAllowedMotorRPM)
        {
          return Diagnostic(Diagnostic::WARN, ReportFault::THRUSTER_RPM_THRESHOLD_REACHED)
              .description("Absolute RPMs above threshold: |%f| > %f", rpms, maxAllowedMotorRPM);
        }
        return Diagnostic::OK;
      }
);
  diagnosticsUpdater.add(motorRPMCheck);

  motorTemperatureCheck = diagnostic_tools::create_health_check<double>(
      "Motor temperature check", [this](double temperature) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        if (temperature > motorTemperatureThreshold)
        {
          return Diagnostic(Diagnostic::ERROR, ReportFault::THRUSTER_TEMP_THRESHOLD_REACHED)
              .description("Temperature above threshold: %f degC > %f degC", temperature,
                           motorTemperatureThreshold);
        }
        return Diagnostic::OK;
      }
);
  diagnosticsUpdater.add(motorTemperatureCheck);

  reportRPMTimer = nodeHandle.createTimer(ros::Duration(reportRPMRate),
                                          &ThrusterControl::reportRPMSendTimeout, this);
  reportMotorTempTimer = nodeHandle.createTimer(ros::Duration(reportMotorTemperatureRate),
                                                &ThrusterControl::reportMotorTempSendTimeout, this);
  reportBatteryHealthTimer =
      nodeHandle.createTimer(ros::Duration(reportBatteryHealthRate),
                             &ThrusterControl::reportBatteryHealthSendTimeout, this);
  canIntf.Init();
}

ThrusterControl::~ThrusterControl() {}

void ThrusterControl::reportRPMSendTimeout(const ros::TimerEvent& timer)
{
  thruster_control::ReportRPM message;

  message.header.stamp = ros::Time::now();
  message.rpms = convertRadSecToRPM(canIntf.velocity_feedback_radsec.Get());
  motorRPMCheck.test(message.rpms);

  publisher_reportRPM.publish(message);
  diagnosticsUpdater.update();
}

void ThrusterControl::reportMotorTempSendTimeout(const ros::TimerEvent& timer)
{
  thruster_control::ReportMotorTemperature message;

  message.header.stamp = ros::Time::now();
  message.motor_temp = canIntf.motor_tempC.Get();
  motorTemperatureCheck.test(message.motor_temp);

  publisher_reportMotorTemp.publish(message);
  diagnosticsUpdater.update();
}

void ThrusterControl::reportBatteryHealthSendTimeout(const ros::TimerEvent& timer)
{
  sensor_msgs::BatteryState message;

  message.header.stamp = ros::Time::now();
  message.voltage = canIntf.battery_voltage.Get()/1000.0;
  //  message.temperature = canIntf.motor_tempC.Get();
  message.current = canIntf.motor_current.Get()/1000.0;
  message.power_supply_status = 0;
  message.power_supply_health = 0;
  message.power_supply_technology = 0;
  message.present = 1;
  message.location = "A";
  message.serial_number = "1";
  batteryCurrentCheck.test(message.current);
  batteryVoltageCheck.test(message.voltage);
  publisher_reportBatteryHealth.publish(message);
  diagnosticsUpdater.update();
}

void ThrusterControl::handle_SetRPM(const thruster_control::SetRPM::ConstPtr& msg)
{
  double motor_rpms = msg->commanded_rpms;
  if (std::abs(motor_rpms) > maxAllowedMotorRPM)
  {
    ROS_DEBUG_STREAM(
      "Motor RPMs exceed maximum allowed: " <<
      "|" << motor_rpms << "|" << " > " << maxAllowedMotorRPM);
    motor_rpms = std::copysign(maxAllowedMotorRPM, motor_rpms);
  }
  canIntf.velocity_radsec.Set(convertRPMToRadSec(motor_rpms));
  canIntf.last_set_rpm_time.Set(ros::Time::now().toSec());
}

double ThrusterControl::convertRPMToRadSec(double rpms) { return (rpms * RPM_TO_RADSEC); }

double ThrusterControl::convertRadSecToRPM(double radsec) { return (radsec * RADSEC_TO_RPM); }

}  // namespace robot
}  // namespace qna
