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
 *   * Neither the name of the QinetiQ nor the names of its
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
 * JausDataManager.h
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */

#ifndef JAUSDATAMANAGER_H_
#define JAUSDATAMANAGER_H_

#include <ros/ros.h>
#include "JausDataManager.h"
#include "JausMessageHeader.h"
#include "JausPresenceVector.h"
#include "PowerPlantControl.h"
#include "ReportBatteryInfo.h"
#include "ReportFinDeflection.h"
#include "ReportINSData.h"
#include "ReportMissionInfo.h"
#include "ReportPoseEstimatorData.h"
#include "ReportPowerPlantStatus.h"
#include "ReportPressureSensorData.h"
#include "SetFinDeflection.h"
#include "SetMissionCommands.h"
#include "SetPayloadCommands.h"

#include "udpserver.h"

#define CONNECT_COMMAND "connect"
#define DISCONNECT_COMMAND "disconnect"
#define ACTIVATE_MAUNAL_CONTROL "ActivateManualControl"
#define DEACTIVATE_MAUNAL_CONTROL "DeactivateManualControl"

#define ENABLE_LOGGING "EnableLogging"
#define DISABLE_LOGGING "DisableLogging"

#define MISSION_START "StartMission"
#define MISSION_ABORT "AbortMission"

#define CLEAR_FAULT "ClearFault"

enum ROS_DISP_TYPE { ROS_DEBUG = 0, ROS_INFO = 1, ROS_WARN = 2, ROS_ERROR = 3, ROS_FATAL = 4 };

extern bool debug_mode;

extern void Ros_Show(ROS_DISP_TYPE type);

class JausDataManager {
 public:
  JausDataManager(ros::NodeHandle* nodeHandle, udpserver* udp);
  virtual ~JausDataManager();
  void SetBatteryPack(string batterypack);

  void ProcessReceivedData(char* buffer);
  static float degreesToRadians(int8_t degrees) { return ((degrees / 180.0) * M_PI); };

  static float radiansToDegrees(float radians) { return (radians * (180.0 / M_PI)); };

  void handleReportRPM(const thruster_control::ReportRPM::ConstPtr& msg);

 private:
  PowerPlantControl _thrusterControl;
  SetFinDeflection _finControl;
  SetMissionCommands _missionCommands;
  SetPayloadCommands _payloadCommands;
  ReportPowerPlantStatus _reportThrusterState;
  ReportFinDeflection _reportFin1State;
  ReportFinDeflection _reportFin2State;
  ReportFinDeflection _reportFin3State;
  ReportFinDeflection _reportFin4State;
  ReportINSData _reportINS;
  ReportPressureSensorData _reportPressureSensor;
  ReportBatteryInfo _reportBatteryInfo;
  ReportPoseEstimatorData _reportPoseData;
  ReportMissionInfo _reportMissionInfo;

  ros::NodeHandle& _nodeHandle;
  udpserver* _udp;
  ros::Publisher _publisher_ActivateManualControl;
  ros::Publisher _publisher_EnableLogging;
  ros::Publisher _publisher_StartMission;
  ros::Publisher _publisher_AbortMission;
  ros::Publisher _publisher_ClearFault;

  bool _activateManualControlEnabled;

  ros::Timer ActivateManualControl_timer;

  void ActivateManualControlTimeout(const ros::TimerEvent& timer);
  void ResetAll();

  // bool _needTimerUpdate;

  ros::Publisher _publisher_setRPM;
  ros::Subscriber _subscriber_reportRPM;
  int16_t _currentRpm;
};

#endif /* JAUSDATAMANAGER_H_ */
