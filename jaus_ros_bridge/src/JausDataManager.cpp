
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
 * JausDataManager.cpp
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */

#include "JausDataManager.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include <jaus_ros_bridge/EnableLogging.h>
#include "JausMessageHeader.h"

#include <thruster_control/SetRPM.h>
#include "PowerPlantControl.h"
#include "ReportPowerPlantStatus.h"

#include <mission_control/AbortMission.h>
#include <mission_control/ExecuteMission.h>

#include <health_monitor/ClearFault.h>


bool debug_mode = false;

JausDataManager::JausDataManager(ros::NodeHandle* nodeHandle, udpserver* udp)
    : _nodeHandle(*nodeHandle) {
  _thrusterControl.init(nodeHandle);
  _finControl.init(nodeHandle);
  _missionCommands.init(nodeHandle);
  _payloadCommands.init(nodeHandle);
  _reportFin1State.init(nodeHandle, 1, udp);
  _reportFin2State.init(nodeHandle, 2, udp);
  _reportFin3State.init(nodeHandle, 3, udp);
  _reportFin4State.init(nodeHandle, 4, udp);
  _reportThrusterState.init(nodeHandle, udp);
  _reportINS.init(nodeHandle, udp);
  _reportPressureSensor.init(nodeHandle, udp);
  _reportBatteryInfo.init(nodeHandle, udp);
  _reportPoseData.init(nodeHandle, udp);
  _reportMissionInfo.init(nodeHandle, udp);
  _reportMissionInfo.init(nodeHandle, udp);

  _publisher_EnableLogging = _nodeHandle.advertise<jaus_ros_bridge::EnableLogging>(
      "/jaus_ros_bridge/enable_logging", 1, true);

  _publisher_ClearFault = _nodeHandle.advertise<health_monitor::ClearFault>(
              "/health_monitor/clear_fault", 1, true);

  _udp = udp;

  _subscriber_reportRPM = _nodeHandle.subscribe("/thruster_control/report_rpm", 1,
                                                &JausDataManager::handleReportRPM, this);

  double OCUCommunicationTimeOut;
  _nodeHandle.param("OCU_timeout", OCUCommunicationTimeOut, 3.0);
  _timer_checkOCUCommunication = _nodeHandle.createTimer(ros::Duration(OCUCommunicationTimeOut),
                                                &JausDataManager::checkOCUConnection, this);

  _currentRpm = 0;
}

JausDataManager::~JausDataManager() {}

void JausDataManager::checkOCUConnection(const ros::TimerEvent& ev)
{
  if(_manualControl)
  {
    _finControl.PublishFinAngles(true);
    _thrusterControl.PublishRPM(true);
    _manualControl = false;
  }
  else
  {
    _finControl.PublishFinAngles(false);
    _thrusterControl.PublishRPM(false);
  }
}

void JausDataManager::ResetAll() {
  _reportFin1State.Reset();
  _reportFin2State.Reset();
  _reportFin3State.Reset();
  _reportFin4State.Reset();
  _reportThrusterState.Reset();
  _reportINS.Reset();
  _reportPressureSensor.Reset();
  _reportBatteryInfo.Reset();
  _reportPoseData.Reset();
  _reportMissionInfo.Reset();
}

void JausDataManager::SetBatteryPack(string batterypack) {
  _reportBatteryInfo.SetBatteryPack(batterypack);
}

void JausDataManager::ProcessReceivedData(char* buffer)
{
  if (strcmp(buffer, CONNECT_COMMAND) == 0) {
    _reportFin1State.Refresh();
    _reportFin2State.Refresh();
    _reportFin3State.Refresh();
    _reportFin4State.Refresh();
    ROS_INFO(CONNECT_COMMAND);
    ResetAll();
  }
  else if (strcmp(buffer, DISCONNECT_COMMAND) == 0) {
    ROS_INFO(DISCONNECT_COMMAND);
  }
  else if (strcmp(buffer, ACTIVATE_MAUNAL_CONTROL) == 0) {
      _missionCommands.StopMissions();
      ROS_INFO(ACTIVATE_MAUNAL_CONTROL);
      _manualControl = true;
  }
  else if (strcmp(buffer, DEACTIVATE_MAUNAL_CONTROL) == 0) {
      ROS_INFO(DEACTIVATE_MAUNAL_CONTROL);
      _manualControl = false;
    }
  else if (strcmp(buffer, ENABLE_LOGGING) == 0) {
    jaus_ros_bridge::EnableLogging msg;
    msg.enable_logging = true;
    _publisher_EnableLogging.publish(msg);
    ROS_INFO(ENABLE_LOGGING);
  }
  else if (strcmp(buffer, DISABLE_LOGGING) == 0) {
      jaus_ros_bridge::EnableLogging msg;
      msg.enable_logging = false;
      _publisher_EnableLogging.publish(msg);
      ROS_INFO(DISABLE_LOGGING);
  }
  else if (strcmp(buffer, CLEAR_FAULT) == 0) {
    health_monitor::ClearFault msg;
    msg.fault_id = health_monitor::ClearFault::ALL_FAULTS; // 0
    _publisher_ClearFault.publish(msg);
    ROS_INFO(CLEAR_FAULT);
  }
  else {
    JausCommandID commandID = (JausCommandID) * ((long*)buffer);
    // thruster control
    if (commandID == JAUS_COMMAND_PowerPlantControl) {
      _thrusterControl.ProcessData(buffer);
    } else if (commandID == JAUS_COMMAND_SetFinDeflection) {
      _finControl.ProcessData(buffer);
    } else if (commandID == JAUS_COMMAND_ExecuteMission || commandID == JAUS_COMMAND_AbortMission ||
               commandID == JAUS_COMMAND_LoadMission || commandID == JAUS_COMMAND_QueryMissions ||
               commandID == JAUS_COMMAND_RemoveMissions) {
      //  TODO  (aschapiro) OCU Should send a DEACTIVATE command to shutdown the timers.
      _manualControl = false;
      _missionCommands.ProcessData(buffer, commandID);
    } else if (commandID == JAUS_COMMAND_PayloadCMD) {
      _payloadCommands.ProcessData(buffer, commandID);
    }
  }
}

void JausDataManager::handleReportRPM(const thruster_control::ReportRPM::ConstPtr& msg) {
  _currentRpm = (int)msg->rpms;
}
