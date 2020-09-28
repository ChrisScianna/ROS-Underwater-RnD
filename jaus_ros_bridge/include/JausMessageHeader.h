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

/*
 * JausMessageHeader.h
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */

#ifndef JAUSMESSAGEHEADER_H_
#define JAUSMESSAGEHEADER_H_
#include <ros/ros.h>

enum JausCommandID {
  JAUS_COMMAND_Invalid = 0,
  JAUS_COMMAND_PowerPlantControl = 0x0A06,
  // JAUS_COMMAND_QueryPowerPlantSpecifications = 0x2A05,
  // JAUS_COMMAND_QueryPowerPlantStatus = 0x2A06,
  // JAUS_COMMAND_ReportPowerPlantSpecifications = 0x4A05,
  JAUS_COMMAND_ReportPowerPlantStatus = 0x4A06,

  // Fin control
  JAUS_COMMAND_SetFinDeflection = 0x0A36,
  JAUS_COMMAND_ReportFinDeflection = 0x4A36,

  // INS data report
  JAUS_COMMAND_ReportINSData = 0x5A01,

  // Altimeter report
  JAUS_COMMAND_ReportAltimeter = 0x5A06,

  // Altimeter report
  JAUS_COMMAND_ReportPoseEstimetorData = 0x5A10,

  // Center of gravity
  // JAUS_COMMAND_ReportBatteryPosition = 0x6A01, //TODO - need to be decided.
  JAUS_COMMAND_ReportFaultID = 0x6A02,  // TODO - need to be decided.
  JAUS_COMMAND_ReportBatteryInfo = 0x6A08,          // TODO - need to be decided.
  // JAUS_COMMAND_SetBatteryPosition = 0x6A04,   //TODO - need to be decided.

  // SSS - side scan sonar
  JAUS_COMMAND_EnableSSS = 0x8A01,

  // Pressure sensor data
  JAUS_COMMAND_ReportPressureSensorData = 0x8A11,

  //    // Mission state data
  //    JAUS_COMMAND_ReportMissionState = 0x8011,
  //    // Mission upload data
  //    JAUS_COMMAND_ReportUploadState = 0x8111,

  //    // Mission commands
  //    JAUS_COMMAND_ExcecuteMission = 0x8121,
  //    JAUS_COMMAND_AbortMission = 0x8131,
  //    JAUS_COMMAND_MissionPath = 0x8141,

  // Mission execute
  JAUS_COMMAND_ExecuteMission = 0x8100,
  JAUS_COMMAND_ReportExecuteMissionState = 0x8110,

  // Mission upload
  JAUS_COMMAND_LoadMission = 0x8200,
  JAUS_COMMAND_ReportLoadMissionState = 0x8210,

  JAUS_COMMAND_AbortMission = 0x8300,

  // Mission IDs
  JAUS_COMMAND_QueryMissions = 0x8400,
  JAUS_COMMAND_ReportMissions = 0x8410,
  // JAUS_COMMAND_MissionData = 0x8420,
  JAUS_COMMAND_RemoveMissions = 0x8420,

  // Payload command
  JAUS_COMMAND_PayloadCMD = 0x3000,

};

// extern bool debug_mode;

class JausMessageHeader {
 private:
  short _data_size;
  JausCommandID _commandID;
  short _headerSize;
  char* _headerData;

 public:
  JausMessageHeader(JausCommandID id, short datasize);
  JausMessageHeader(char* messageData);
  virtual ~JausMessageHeader();
  short GetHeadersize();
  short GetDatasize();
  JausCommandID GetId();
  char* GetHeaderData();
};

#endif /* JAUSMESSAGEHEADER_H_ */
