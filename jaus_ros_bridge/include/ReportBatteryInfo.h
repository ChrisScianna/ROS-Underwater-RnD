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

// Original version: Jie Sun <Jie.Sun@us.QinetiQ.com>

#ifndef REPORTBATTERYINFOMATION_H
#define REPORTBATTERYINFOMATION_H

#include "JausMessageOut.h"

#include <battery_monitor/ReportBatteryInfo.h>
#include <battery_monitor/ReportLeakDetected.h>

struct BatteryInfoData {
  bool batteryPackAConnected;
  bool batteryPackBConnected;
  float batteryPacksTotalCurrent;

  float batteryPackCell1;
  float batteryPackCell2;
  float batteryPackCell3;
  float batteryPackCell4;
  float batteryPackCell5;
  float batteryPackCell6;
  float batteryPackCell7;
  float batteryPackCell8;
  float batteryPackCell9;
  float batteryPackCell10;

  // Not used for now
  // float batteryPackBCell1;
  // float batteryPackBCell2;
  // float batteryPackBCell3;
  // float batteryPackBCell4;
  // float batteryPackBCell5;
  // float batteryPackBCell6;
  // float batteryPackBCell7;
  // float batteryPackBCell8;
  // float batteryPackBCell9;
  // float batteryPackBCell10;

  unsigned short batteryCell1Thermocouple;
  unsigned short batteryCell2Thermocouple;
  unsigned short batteryCell3Thermocouple;
  unsigned short batteryCell4Thermocouple;
  unsigned short batteryCell5Thermocouple;
  unsigned short batteryCell6Thermocouple;
  unsigned short batteryCell7Thermocouple;
  unsigned short batteryCell8Thermocouple;

  unsigned short systemThermocouple1;
};

class ReportBatteryInfo : public JausMessageOut {
 private:
  uint8_t* Int16ToByteArray(int16_t inVal);
  ros::Subscriber _subscriber_reportLeakDetected;
  ros::Subscriber _subscriber_reportBatteryInfo;
  bool _leakDetected;

  string _batterypack;

  BatteryInfoData _batteryInfo;

  bool HasNewBatteryInfoData(const battery_monitor::ReportBatteryInfo::ConstPtr& msg);

 public:
  // ReportBatteryInfo();
  void init(ros::NodeHandle* nodeHandle, udpserver* udp);

  void SetBatteryPack(string batterypack) { _batterypack = batterypack; }

  void handleReportLeakDetected(const battery_monitor::ReportLeakDetected::ConstPtr& msg);
  void handleReportBatteryInfo(const battery_monitor::ReportBatteryInfo::ConstPtr& msg);

  virtual DataInfo GetPackedMessage(void* data){};
  virtual void Reset();
  DataInfo GetPackedMessageForLeak(bool leak);
  DataInfo GetPackedMessageForBatteryInfo(void* data);
};

#endif  // REPORTBATTERYINFOMATION_H
