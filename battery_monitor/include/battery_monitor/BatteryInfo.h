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
 * BatteryInfo.h
 */

#ifndef BATTERY_MONITOR_BATTERYINFO_H
#define BATTERY_MONITOR_BATTERYINFO_H

class BatteryInfo
{
 public:
  BatteryInfo()
  {
    batteryPackAConnected = 0;
    batteryPackBConnected = 0;
    batteryPacksTotalCurrent = 0.0;

    batteryPackACell1 = 0.0;
    batteryPackACell2 = 0.0;
    batteryPackACell3 = 0.0;
    batteryPackACell4 = 0.0;
    batteryPackACell5 = 0.0;
    batteryPackACell6 = 0.0;
    batteryPackACell7 = 0.0;
    batteryPackACell8 = 0.0;
    batteryPackACell9 = 0.0;
    batteryPackACell10 = 0.0;

    batteryPackBCell1 = 0.0;
    batteryPackBCell2 = 0.0;
    batteryPackBCell3 = 0.0;
    batteryPackBCell4 = 0.0;
    batteryPackBCell5 = 0.0;
    batteryPackBCell6 = 0.0;
    batteryPackBCell7 = 0.0;
    batteryPackBCell8 = 0.0;
    batteryPackBCell9 = 0.0;
    batteryPackBCell10 = 0.0;

    batteryCell1Thermocouple = 0;
    batteryCell2Thermocouple = 0;
    batteryCell3Thermocouple = 0;
    batteryCell4Thermocouple = 0;
    batteryCell5Thermocouple = 0;
    batteryCell6Thermocouple = 0;
    batteryCell7Thermocouple = 0;
    batteryCell8Thermocouple = 0;

    systemThermocouple1 = 0;
  }

  ~BatteryInfo() { }

  void operator=(const BatteryInfo &bi)
  {
    batteryPackAConnected = bi.batteryPackAConnected;
    batteryPackBConnected = bi.batteryPackBConnected;
    batteryPacksTotalCurrent = bi.batteryPacksTotalCurrent;

    batteryPackACell1 = bi.batteryPackACell1;
    batteryPackACell2 = bi.batteryPackACell2;
    batteryPackACell3 = bi.batteryPackACell3;
    batteryPackACell4 = bi.batteryPackACell4;
    batteryPackACell5 = bi.batteryPackACell5;
    batteryPackACell6 = bi.batteryPackACell6;
    batteryPackACell7 = bi.batteryPackACell7;
    batteryPackACell8 = bi.batteryPackACell8;
    batteryPackACell9 = bi.batteryPackACell9;
    batteryPackACell10 = bi.batteryPackACell10;

    batteryPackBCell1 = bi.batteryPackBCell1;
    batteryPackBCell2 = bi.batteryPackBCell2;
    batteryPackBCell3 = bi.batteryPackBCell3;
    batteryPackBCell4 = bi.batteryPackBCell4;
    batteryPackBCell5 = bi.batteryPackBCell5;
    batteryPackBCell6 = bi.batteryPackBCell6;
    batteryPackBCell7 = bi.batteryPackBCell7;
    batteryPackBCell8 = bi.batteryPackBCell8;
    batteryPackBCell9 = bi.batteryPackBCell9;
    batteryPackBCell10 = bi.batteryPackBCell10;

    batteryCell1Thermocouple = bi.batteryCell1Thermocouple;
    batteryCell2Thermocouple = bi.batteryCell2Thermocouple;
    batteryCell3Thermocouple = bi.batteryCell3Thermocouple;
    batteryCell4Thermocouple = bi.batteryCell4Thermocouple;
    batteryCell5Thermocouple = bi.batteryCell5Thermocouple;
    batteryCell6Thermocouple = bi.batteryCell6Thermocouple;
    batteryCell7Thermocouple = bi.batteryCell7Thermocouple;
    batteryCell8Thermocouple = bi.batteryCell8Thermocouple;

    systemThermocouple1 = bi.systemThermocouple1;
  }

  bool batteryPackAConnected;
  bool batteryPackBConnected;
  double batteryPacksTotalCurrent;
  double batteryPackACell1;
  double batteryPackACell2;
  double batteryPackACell3;
  double batteryPackACell4;
  double batteryPackACell5;
  double batteryPackACell6;
  double batteryPackACell7;
  double batteryPackACell8;
  double batteryPackACell9;
  double batteryPackACell10;

  double batteryPackBCell1;
  double batteryPackBCell2;
  double batteryPackBCell3;
  double batteryPackBCell4;
  double batteryPackBCell5;
  double batteryPackBCell6;
  double batteryPackBCell7;
  double batteryPackBCell8;
  double batteryPackBCell9;
  double batteryPackBCell10;

  unsigned char batteryCell1Thermocouple;
  unsigned char batteryCell2Thermocouple;
  unsigned char batteryCell3Thermocouple;
  unsigned char batteryCell4Thermocouple;
  unsigned char batteryCell5Thermocouple;
  unsigned char batteryCell6Thermocouple;
  unsigned char batteryCell7Thermocouple;
  unsigned char batteryCell8Thermocouple;

  unsigned char systemThermocouple1;
};

#endif  // BATTERY_MONITOR_BATTERYINFO_H
