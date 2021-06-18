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
 * PowerPlantControl.cpp
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */

#include "PowerPlantControl.h"
#include <thruster_control/SetRPM.h>
#include "JausDataManager.h"
#include "JausMessageHeader.h"

void PowerPlantControl::init(ros::NodeHandle* nodeHandle) {
  _nodeHandle = nodeHandle;
  _publisher_setRPM =
      _nodeHandle->advertise<thruster_control::SetRPM>("input/jaus_ros_bridge/set_rpm", 1, true);

  _isSetToZero = true;

  // Info
  // ROS_INFO("Specified max RPM is: +-%d", thruster_control::SetRPM::MAX_RPM);

  double thrusterRPMPublishingRate;
  _nodeHandle->param("thruster_rpm_publishing_rate", thrusterRPMPublishingRate, 15.0);
  _commandRPMTimer = _nodeHandle->createTimer(ros::Duration(1.0 / thrusterRPMPublishingRate),
                                                   &PowerPlantControl::commandRPM, this);

}

void PowerPlantControl::ProcessData(char* message) {
  if (message == nullptr) return;

  // build new header every time a new message is received
  if (_header != nullptr) delete _header;
  _header = new JausMessageHeader(message);  // 6 bytes
  int index = 6;                             // read data starts from index 6
  _powerPlantId = (int8_t)message[index++];  // i byte
  _powerCmd = (int8_t)message[index++];      // 1 byte

  // create new presence vector every time a new message is received
  if (_PresenceVector != nullptr) delete _PresenceVector;
  int16_t pv = buildPresenceVector(message, index);
  // printf("Presence vector is: %d \n", pv);

  _engineId = (int8_t)message[index++];  // 1 byte

  if (_PresenceVector->IsBitSet((int)RPMBit)) {
    char tmp[4];
    tmp[0] = message[index++];
    tmp[1] = message[index++];
    tmp[2] = message[index++];
    tmp[3] = message[index++];
    _rpm = *((int*)tmp);

    // Make sure the max RPM do not exceed the max value
    if (_rpm > thruster_control::SetRPM::MAX_RPM) {
      _rpm = thruster_control::SetRPM::MAX_RPM;
      // if(debug_mode)
      ROS_ERROR("RPM exceeded Max. Set at %d", thruster_control::SetRPM::MAX_RPM);
    } else if (_rpm < (0 - thruster_control::SetRPM::MAX_RPM)) {
      _rpm = 0 - thruster_control::SetRPM::MAX_RPM;
      // if(debug_mode)
      ROS_ERROR("RPM exceeded -Max. Set at %d", (0 - thruster_control::SetRPM::MAX_RPM));
    }
    _isSetToZero = false;
    if (_rpm == 0) {
      _isSetToZero = true;
    }
  }
}

int8_t PowerPlantControl::GetPowerPlantID() { return _powerPlantId; }

int8_t PowerPlantControl::GetPowerCMD() { return _powerCmd; }

int8_t PowerPlantControl::GetEnginId() { return _engineId; }

int PowerPlantControl::GetRpm() {
  if (_PresenceVector->IsBitSet((int)RPMBit)) return _rpm;
  return 0;
}

void PowerPlantControl::StopThruster() {
  if (_isSetToZero) return;
  _rpm = 0;
  ROS_INFO("Set thruster rpm to 0 at connection lost!!!");
  _isSetToZero = true;
}

void PowerPlantControl::PublishRPM(const bool enable)
{
  if (enable)
    _commandRPMTimer.start();
  else
    _commandRPMTimer.stop();
}

void PowerPlantControl::commandRPM(const ros::TimerEvent& ev)
{
  thruster_control::SetRPM msg;
  msg.commanded_rpms = _rpm;  //_maxRPM * _rpm/100;
  _publisher_setRPM.publish(msg);
}
