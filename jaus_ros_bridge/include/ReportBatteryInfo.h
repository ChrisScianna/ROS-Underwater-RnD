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

// Original version: Jie Sun <Jie.Sun@us.QinetiQ.com>

#ifndef REPORTBATTERYINFOMATION_H
#define REPORTBATTERYINFOMATION_H

#include "JausMessageOut.h"

#include <health_monitor/ReportFault.h>
#include <thruster_control/ReportMotorTemperature.h>
#include <sensor_msgs/BatteryState.h>

struct BatteryInfoData {
    float batteryPacksTotalCurrent;
    float batteryVoltage;
    unsigned short systemThermocouple1;
};

class ReportBatteryInfo : public JausMessageOut {
 private:
  uint8_t* Int16ToByteArray(int16_t inVal);
  ros::Subscriber _subscriber_reportFaultID;
  ros::Subscriber _subscriber_reportBatteryInfo;
  ros::Subscriber _subscriber_reportTemperature;

  uint64_t _fault_id;
  //bool _leakDetected;

  string _batterypack;

  BatteryInfoData _batteryInfo;

  bool HasNewBatteryInfoData(const sensor_msgs::BatteryState::ConstPtr& msg);
  bool HasNewTemperatureData(const thruster_control::ReportMotorTemperature::ConstPtr& msg);

  bool TimeToSend1(clock_t beginTime, clock_t endTime) {
    int difference = (((int)endTime) - ((int)beginTime));
    return difference >= _sendInterval;
  }

 public:
  // ReportBatteryInfo();
  void init(ros::NodeHandle* nodeHandle, udpserver* udp);

  void SetBatteryPack(string batterypack) { _batterypack = batterypack; }

  void handleReportFaultID(const health_monitor::ReportFault::ConstPtr& msg);
  void handleReportTemperature(const thruster_control::ReportMotorTemperature::ConstPtr& msg);
  void handleReportBatteryInfo(const sensor_msgs::BatteryState::ConstPtr& msg);

  virtual DataInfo GetPackedMessage(void* data){};
  virtual void Reset();
  DataInfo GetPackedMessageForFaultID(uint64_t faults);
  DataInfo GetPackedMessageForBatteryInfo(void* data);
};

#endif  // REPORTBATTERYINFOMATION_H
