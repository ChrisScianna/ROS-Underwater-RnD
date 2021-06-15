
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

#include <jaus_ros_bridge/ActivateManualControl.h>
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

  _publisher_ActivateManualControl = _nodeHandle.advertise<jaus_ros_bridge::ActivateManualControl>(
      "/jaus_ros_bridge/activate_manual_control", 1, true);
  _publisher_EnableLogging = _nodeHandle.advertise<jaus_ros_bridge::EnableLogging>(
      "/jaus_ros_bridge/enable_logging", 1, true);

  _publisher_ClearFault = _nodeHandle.advertise<health_monitor::ClearFault>(
              "/health_monitor/clear_fault", 1, true);

  _udp = udp;

  _subscriber_reportRPM = _nodeHandle.subscribe("/thruster_control/report_rpm", 1,
                                                &JausDataManager::handleReportRPM, this);
  _publisher_setRPM =
      _nodeHandle.advertise<thruster_control::SetRPM>("input/jaus_ros_bridge/set_rpm", 1, true);

  _currentRpm = 0;
  // time out every 2 sec
  ActivateManualControl_timer = _nodeHandle.createTimer(
      ros::Duration(2), &JausDataManager::ActivateManualControlTimeout, this);
  _activateManualControlEnabled = false;
  _deactivateFromOCU = true;
  //_needTimerUpdate = false;
}

JausDataManager::~JausDataManager() {}

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

void JausDataManager::ActivateManualControlTimeout(const ros::TimerEvent& timer) {
  if (_activateManualControlEnabled) {
    _activateManualControlEnabled = false;

  } else {
    jaus_ros_bridge::ActivateManualControl msg;
    msg.activate_manual_control = false;
    _publisher_ActivateManualControl.publish(msg);
    _activateManualControlEnabled = false;
    //_needTimerUpdate = false;
    // Tell thruster to stop if running
    _thrusterControl.StopThruster();
    //_udp->TimeoutDisconnect();
  }
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
    jaus_ros_bridge::ActivateManualControl msg;
    msg.activate_manual_control = false;
    _publisher_ActivateManualControl.publish(msg);
    ROS_INFO(DISCONNECT_COMMAND);
  }
  else if (strcmp(buffer, ACTIVATE_MAUNAL_CONTROL) == 0) {
    jaus_ros_bridge::ActivateManualControl msg;
    msg.activate_manual_control = true;
    _publisher_ActivateManualControl.publish(msg);

    if(!_activateManualControlEnabled&&_deactivateFromOCU){
      ROS_INFO(ACTIVATE_MAUNAL_CONTROL);
      _deactivateFromOCU = false;
    }
    
    _activateManualControlEnabled = true;
    //_needTimerUpdate = true;
  }
  else if (strcmp(buffer, DEACTIVATE_MAUNAL_CONTROL) == 0) {
    jaus_ros_bridge::ActivateManualControl msg;
    msg.activate_manual_control = false;
    _publisher_ActivateManualControl.publish(msg);
    if(_activateManualControlEnabled)
      ROS_INFO(DEACTIVATE_MAUNAL_CONTROL);
    _activateManualControlEnabled = false;
    _deactivateFromOCU = true;
    //_needTimerUpdate = false;

    ResetAll();
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
      _missionCommands.ProcessData(buffer, commandID);
    } else if (commandID == JAUS_COMMAND_PayloadCMD) {
      _payloadCommands.ProcessData(buffer, commandID);
    }
  }
}

void JausDataManager::handleReportRPM(const thruster_control::ReportRPM::ConstPtr& msg) {
  _currentRpm = (int)msg->rpms;
}
