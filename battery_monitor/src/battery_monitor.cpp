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
 * battery_monitor.cpp
 *
 */

#include "battery_monitor/battery_monitor.h"

namespace qna
{
namespace robot
{
BatteryMonitor::BatteryMonitor(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle), diagnosticsUpdater(nodeHandle)
{
  using health_monitor::ReportFault;

  diagnosticsUpdater.setHardwareID("battery");

  double minBatteryInfoCheckPeriod = 0.5;
  nodeHandle.getParam("/battery_monitor/min_battery_info_check_period", minBatteryInfoCheckPeriod);
  ROS_INFO("Minimun Battery Info Check Period:[%lf]", minBatteryInfoCheckPeriod);

  double maxBatteryInfoCheckPeriod = 2.0;
  nodeHandle.getParam("/battery_monitor/max_battery_info_check_period", maxBatteryInfoCheckPeriod);
  ROS_INFO("Maximun Battery Info Check Period:[%lf]", maxBatteryInfoCheckPeriod);

  double minBatteryLeakCheckPeriod = 0.5;
  nodeHandle.getParam("/battery_monitor/min_battery_leak_check_period", minBatteryLeakCheckPeriod);
  ROS_INFO("Minimun Battery Leak Check Period:[%lf]", minBatteryLeakCheckPeriod);

  double maxBatteryLeakCheckPeriod = 2.0;
  nodeHandle.getParam("/battery_monitor/max_battery_check_period", maxBatteryLeakCheckPeriod);
  ROS_INFO("Maximun Battery Check Period:[%lf]", maxBatteryLeakCheckPeriod);

  double batteryCheckPeriod = (maxBatteryInfoCheckPeriod + minBatteryInfoCheckPeriod) / 2.0;
  nodeHandle.getParam("/battery_monitor/battery_check_period", batteryCheckPeriod);
  ROS_INFO("Battery Check Period:[%lf]", batteryCheckPeriod);

  double batteryTotalCurrentSteadyBand = 0.0;
  nodeHandle.getParam("/battery_monitor/battery_total_current_steady_band",
                      batteryTotalCurrentSteadyBand);
  ROS_INFO("Epsilon value for stagnation compare values:[%lf]", batteryTotalCurrentSteadyBand);

  double minBatteryTotalCurrent = 5000;  // in mA
  nodeHandle.getParam("/battery_monitor/min_battery_total_current", minBatteryTotalCurrent);
  ROS_INFO("Minimum Battery Total Current:[%lf]", minBatteryTotalCurrent);

  double minBatteryCellVoltage = 30;  // in V
  nodeHandle.getParam("/battery_monitor/min_battery_cell_voltage", minBatteryCellVoltage);
  ROS_INFO("Minimum Battery Cell Voltage:[%lf]", minBatteryCellVoltage);

  double maxBatteryTemperature = 50;  // in Celsius degrees
  nodeHandle.getParam("/battery_monitor/max_battery_temperature", maxBatteryTemperature);
  ROS_INFO("Maximum Battery Temperature:[%lf]", maxBatteryTemperature);

  publisher_reportBatteryInfo =
      diagnostic_tools::create_publisher<battery_monitor::ReportBatteryInfo>(
          nodeHandle, "/battery_monitor/report_battery_info", 1);

  diagnostic_tools::PeriodicMessageStatusParams paramsBatteryCheckPeriod{};
  paramsBatteryCheckPeriod.min_acceptable_period(minBatteryInfoCheckPeriod);
  paramsBatteryCheckPeriod.max_acceptable_period(maxBatteryInfoCheckPeriod);
  diagnostic_tools::Diagnostic diagnosticBateryInfoStale
  {
    diagnostic_tools::Diagnostic::WARN, ReportFault::BATTERY_INFO_STALE
  };
  paramsBatteryCheckPeriod.abnormal_diagnostic(diagnosticBateryInfoStale);
  diagnosticsUpdater.add(
      publisher_reportBatteryInfo.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check", paramsBatteryCheckPeriod));


  diagnostic_tools::MessageStagnationCheckParams paramsBatteryReportInfoStagnate{};
  paramsBatteryReportInfoStagnate.stagnation_diagnostic(diagnostic_tools::Diagnostic
  {
    diagnostic_tools::Diagnostic::WARN, ReportFault::BATTERY_INFO_STAGNATED
  }
);
  diagnosticsUpdater.add(
      publisher_reportBatteryInfo.add_check<diagnostic_tools::MessageStagnationCheck>(
          "stagnation check",
          [batteryTotalCurrentSteadyBand](const battery_monitor::ReportBatteryInfo &a,
                                          const battery_monitor::ReportBatteryInfo &b)
          {
            return (std::fabs(a.batteryPacksTotalCurrent - b.batteryPacksTotalCurrent) <
                    batteryTotalCurrentSteadyBand);
          }
          , paramsBatteryReportInfoStagnate));

  batteryCurrentCheck = diagnostic_tools::create_health_check<BatteryInfo>(
      "Battery Current check",
      [minBatteryTotalCurrent](const BatteryInfo &info) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (info.batteryPacksTotalCurrent < minBatteryTotalCurrent)
        {
          diagnostic.status(Diagnostic::WARN)
              .description("Total current - NOT OK (%f mA < %f mA)", info.batteryPacksTotalCurrent,
                           minBatteryTotalCurrent)
              .code(ReportFault::BATTERY_CURRENT_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Packs total current - OK (%f mA)", info.batteryPacksTotalCurrent);
        }
        return diagnostic;
      }
);
  diagnosticsUpdater.add(batteryCurrentCheck);

  batteryVoltageCheck = diagnostic_tools::create_health_check<BatteryInfo>(
      "Battery Voltage check",
      [minBatteryCellVoltage](const BatteryInfo &info) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (info.batteryPackACell1 < minBatteryCellVoltage)
        {
          diagnostic.status(Diagnostic::WARN)
              .description("Pack A cell 1 voltage - NOT OK (%f V < %f V)", info.batteryPackACell1,
                           minBatteryCellVoltage)
              .code(ReportFault::BATTERY_VOLTAGE_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Pack A cell 1 voltage - OK (%f V)", info.batteryPackACell1);
        }
        return diagnostic;
      }
);
  diagnosticsUpdater.add(batteryVoltageCheck);

  batteryTemperatureCheck = diagnostic_tools::create_health_check<BatteryInfo>(
      "Battery Voltage check",
      [maxBatteryTemperature](const BatteryInfo &info) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (info.systemThermocouple1 > maxBatteryTemperature)
        {
          diagnostic.status(Diagnostic::WARN)
              .description("Packs temperature - NOT OK (%f degC > %f degC)",
                           info.systemThermocouple1, maxBatteryTemperature)
              .code(ReportFault::BATTERY_TEMPERATURE_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Packs temperature - OK (%f degC)", info.systemThermocouple1);
        }
        return diagnostic;
      }
);

  diagnosticsUpdater.add(batteryTemperatureCheck);

  publisher_reportLeakDetected =
      diagnostic_tools::create_publisher<battery_monitor::ReportLeakDetected>(
          nodeHandle, "/battery_monitor/report_leak_detected", 1);

  diagnostic_tools::PeriodicMessageStatusParams paramsBatteryLeakCheckPeriod{};
  paramsBatteryLeakCheckPeriod.min_acceptable_period(minBatteryLeakCheckPeriod);
  paramsBatteryLeakCheckPeriod.max_acceptable_period(maxBatteryLeakCheckPeriod);
  diagnostic_tools::Diagnostic diagnosticBateryInfoLeakStale
  {
    diagnostic_tools::Diagnostic::WARN, ReportFault::BATTERY_INFO_STALE
  };
  paramsBatteryLeakCheckPeriod.abnormal_diagnostic(diagnosticBateryInfoLeakStale);

  diagnosticsUpdater.add(
      publisher_reportLeakDetected.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check", paramsBatteryLeakCheckPeriod));

  batteryLeakCheck = diagnostic_tools::create_health_check<bool>(
      "Battery leak check", [](bool leak_detected) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        if (leak_detected)
        {
          return
          {
            Diagnostic::WARN, "Battery leak detected", ReportFault::BATTERY_LEAK_DETECTED};
        }
        return {Diagnostic::OK, "No leaks detected"};
      }
);
  diagnosticsUpdater.add(batteryLeakCheck);

  canIntf.Init();

  batteryCheckTimer = nodeHandle.createTimer(ros::Duration(batteryCheckPeriod),
                                             &BatteryMonitor::batteryCheckTimeout, this);
}

BatteryMonitor::~BatteryMonitor() {}

void BatteryMonitor::batteryCheckTimeout(const ros::TimerEvent &timer)
{
  checkBatteryInfo();
  checkBatteryLeaks();

  diagnosticsUpdater.update();
}

void BatteryMonitor::checkBatteryLeaks()
{
  bool leak_detected = canIntf.GetLeakDetected();
  batteryLeakCheck.test(leak_detected);

  battery_monitor::ReportLeakDetected message;
  message.header.stamp = ros::Time::now();
  message.leakDetected = leak_detected;

  publisher_reportLeakDetected.publish(message);
}

void BatteryMonitor::checkBatteryInfo()
{
  BatteryInfo bi = canIntf.GetBatteryInfo();
  batteryCurrentCheck.test(bi);
  batteryVoltageCheck.test(bi);
  batteryTemperatureCheck.test(bi);

  battery_monitor::ReportBatteryInfo message;

  message.header.stamp = ros::Time::now();

  message.batteryPackA.present = bi.batteryPackAConnected;
  message.batteryPackB.present = bi.batteryPackBConnected;

  message.batteryPacksTotalCurrent = bi.batteryPacksTotalCurrent;
  message.batteryPackA.cell_voltage[0] = bi.batteryPackACell1;
  message.batteryPackA.cell_voltage[1] = bi.batteryPackACell2;
  message.batteryPackA.cell_voltage[2] = bi.batteryPackACell3;
  message.batteryPackA.cell_voltage[3] = bi.batteryPackACell4;
  message.batteryPackA.cell_voltage[4] = bi.batteryPackACell5;
  message.batteryPackA.cell_voltage[5] = bi.batteryPackACell6;
  message.batteryPackA.cell_voltage[6] = bi.batteryPackACell7;
  message.batteryPackA.cell_voltage[7] = bi.batteryPackACell8;
  message.batteryPackA.cell_voltage[8] = bi.batteryPackACell9;
  message.batteryPackA.cell_voltage[9] = bi.batteryPackACell10;

  message.batteryPackB.cell_voltage[0] = bi.batteryPackBCell1;
  message.batteryPackB.cell_voltage[1] = bi.batteryPackBCell2;
  message.batteryPackB.cell_voltage[2] = bi.batteryPackBCell3;
  message.batteryPackB.cell_voltage[3] = bi.batteryPackBCell4;
  message.batteryPackB.cell_voltage[4] = bi.batteryPackBCell5;
  message.batteryPackB.cell_voltage[5] = bi.batteryPackBCell6;
  message.batteryPackB.cell_voltage[6] = bi.batteryPackBCell7;
  message.batteryPackB.cell_voltage[7] = bi.batteryPackBCell8;
  message.batteryPackB.cell_voltage[8] = bi.batteryPackBCell9;
  message.batteryPackB.cell_voltage[9] = bi.batteryPackBCell10;

  message.batteryCell1Thermocouple = bi.batteryCell1Thermocouple;
  message.batteryCell2Thermocouple = bi.batteryCell2Thermocouple;
  message.batteryCell3Thermocouple = bi.batteryCell3Thermocouple;
  message.batteryCell4Thermocouple = bi.batteryCell4Thermocouple;
  message.batteryCell5Thermocouple = bi.batteryCell5Thermocouple;
  message.batteryCell6Thermocouple = bi.batteryCell6Thermocouple;
  message.batteryCell7Thermocouple = bi.batteryCell7Thermocouple;
  message.batteryCell8Thermocouple = bi.batteryCell8Thermocouple;

  message.systemThermocouple1 = bi.systemThermocouple1;

  publisher_reportBatteryInfo.publish(message);
}

}  // namespace robot
}  // namespace qna
