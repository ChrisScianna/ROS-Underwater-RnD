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

#include "battery_monitor.h"

#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>

namespace qna {
namespace robot {

BatteryMonitor::BatteryMonitor(ros::NodeHandle &nodeHandle)
    : nodeHandle(nodeHandle), diagnosticsUpdater(nodeHandle) {
  diagnosticsUpdater.setHardwareID("battery");

  double batteryCheckPeriod = 1.0;
  nodeHandle.getParam("~/battery_check_period", batteryCheckPeriod);
  ROS_INFO("Battery Check Period:[%lf]", batteryCheckPeriod);

  double minBatteryTotalCurrent = 5000;  // in mA
  nodeHandle.getParam("~/min_battery_total_current", minBatteryTotalCurrent);
  ROS_INFO("Minimum Battery Total Current:[%lf]", minBatteryTotalCurrent);

  double minBatteryCellVoltage = 30;  // in V
  nodeHandle.getParam("~/min_battery_cell_voltage", minBatteryCellVoltage);
  ROS_INFO("Minimum Battery Cell Voltage:[%lf]", minBatteryCellVoltage);

  double maxBatteryTemperature = 50;  // in Celsius degrees
  nodeHandle.getParam("~/max_battery_temperature", maxBatteryTemperature);
  ROS_INFO("Maximum Battery Temperature:[%lf]", maxBatteryTemperature);

  publisher_reportBatteryInfo =
      diagnostic_tools::create_publisher<battery_monitor::ReportBatteryInfo>(
          nodeHandle, "/battery_monitor/report_battery_info", 1);

  diagnosticsUpdater.add(
      publisher_reportBatteryInfo.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check",
          diagnostic_tools::PeriodicMessageStatusParams{}
              .min_acceptable_period(batteryCheckPeriod / 2)
              .max_acceptable_period(batteryCheckPeriod * 2)
              .abnormal_diagnostic({diagnostic_tools::Diagnostic::WARN,
                                    health_monitor::ReportFault::BATTERY_INFO_STALE})));

  diagnosticsUpdater.add(
      publisher_reportBatteryInfo.add_check<diagnostic_tools::MessageStagnationCheck>(
          "stagnation check",
          [](const battery_monitor::ReportBatteryInfo &a,
             const battery_monitor::ReportBatteryInfo &b) {
            return a.batteryPacksTotalCurrent == b.batteryPacksTotalCurrent;
          },
          diagnostic_tools::MessageStagnationCheckParams{}.stagnation_diagnostic(
              {diagnostic_tools::Diagnostic::WARN,
               health_monitor::ReportFault::BATTERY_INFO_STAGNATED})));

  batteryCurrentCheck = diagnostic_tools::create_health_check<BatteryInfo>(
      "Battery Current check", [=](const BatteryInfo &info) -> diagnostic_tools::Diagnostic {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (info.batteryPacksTotalCurrent < minBatteryTotalCurrent) {
          diagnostic.status(Diagnostic::WARN)
              .description("Total current - NOT OK (%f mA < %f mA)", info.batteryPacksTotalCurrent,
                           minBatteryTotalCurrent)
              .code(health_monitor::ReportFault::BATTERY_CURRENT_THRESHOLD_REACHED);
        } else {
          diagnostic.description("Packs total current - OK (%f mA)", info.batteryPacksTotalCurrent);
        }
        return diagnostic;
      });
  diagnosticsUpdater.add(batteryCurrentCheck);

  batteryVoltageCheck = diagnostic_tools::create_health_check<BatteryInfo>(
      "Battery Voltage check", [=](const BatteryInfo &info) -> diagnostic_tools::Diagnostic {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (info.batteryPackACell1 < minBatteryCellVoltage) {
          diagnostic.status(Diagnostic::WARN)
              .description("Pack A cell 1 voltage - NOT OK (%f V < %f V)", info.batteryPackACell1,
                           minBatteryCellVoltage)
              .code(health_monitor::ReportFault::BATTERY_VOLTAGE_THRESHOLD_REACHED);
        } else {
          diagnostic.description("Pack A cell 1 voltage - OK (%f V)", info.batteryPackACell1);
        }
        return diagnostic;
      });
  diagnosticsUpdater.add(batteryVoltageCheck);

  batteryTemperatureCheck = diagnostic_tools::create_health_check<BatteryInfo>(
      "Battery Voltage check", [=](const BatteryInfo &info) -> diagnostic_tools::Diagnostic {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (info.systemThermocouple1 > maxBatteryTemperature) {
          diagnostic.status(Diagnostic::WARN)
              .description("Packs temperature - NOT OK (%f degC > %f degC)",
                           info.systemThermocouple1, maxBatteryTemperature)
              .code(health_monitor::ReportFault::BATTERY_TEMPERATURE_THRESHOLD_REACHED);
        } else {
          diagnostic.description("Packs temperature - OK (%f degC)", info.systemThermocouple1);
        }
        return diagnostic;
      });

  diagnosticsUpdater.add(batteryTemperatureCheck);

  publisher_reportLeakDetected =
      diagnostic_tools::create_publisher<battery_monitor::ReportLeakDetected>(
          nodeHandle, "/battery_monitor/report_leak_detected", 1);

  diagnosticsUpdater.add(
      publisher_reportLeakDetected.add_check<diagnostic_tools::PeriodicMessageStatus>(
          "rate check",
          diagnostic_tools::PeriodicMessageStatusParams{}
              .min_acceptable_period(batteryCheckPeriod / 2)
              .max_acceptable_period(batteryCheckPeriod * 2)
              .abnormal_diagnostic({diagnostic_tools::Diagnostic::WARN,
                                    health_monitor::ReportFault::BATTERY_INFO_STALE})));

  batteryLeakCheck = diagnostic_tools::create_health_check<bool>(
      "Battery leak check", [=](bool leak_detected) -> diagnostic_tools::Diagnostic {
        using diagnostic_tools::Diagnostic;
        if (leak_detected) {
          return {Diagnostic::WARN, "Battery leak detected",
                  health_monitor::ReportFault::LEAK_DETECTED};
        }
        return {Diagnostic::OK, "No leaks detected"};
      });
  diagnosticsUpdater.add(batteryLeakCheck);

  canIntf.Init();

  batteryCheckTimer = nodeHandle.createTimer(ros::Duration(batteryCheckPeriod),
                                             &BatteryMonitor::batteryCheckTimeout, this);
}

BatteryMonitor::~BatteryMonitor() {}

void BatteryMonitor::batteryCheckTimeout(const ros::TimerEvent &timer) {
  checkBatteryInfo();
  checkBatteryLeaks();

  diagnosticsUpdater.update();
}

void BatteryMonitor::checkBatteryLeaks() {
  bool leak_detected = canIntf.GetLeakDetected();
  batteryLeakCheck.test(leak_detected);

  battery_monitor::ReportLeakDetected message;
  message.header.stamp = ros::Time::now();
  message.leakDetected = leak_detected;

  publisher_reportLeakDetected.publish(message);
}

void BatteryMonitor::checkBatteryInfo() {
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
