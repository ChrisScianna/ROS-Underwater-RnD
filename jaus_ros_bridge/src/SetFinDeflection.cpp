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
 * SetFinDeflection.cpp
 *
 *  Created on: Jun 11, 2019
 *      Author: jsun
 */

#include "SetFinDeflection.h"

#include "JausDataManager.h"

void SetFinDeflection::init(ros::NodeHandle* nodeHandle)
{
  _nodeHandle = nodeHandle;
  _publisher_setAngles =
      _nodeHandle->advertise<fin_control::SetAngles>("input/jaus_ros_bridge/set_angles", 1, true);

  double finAnglesPublishingRate;
  _nodeHandle->param("fin_angles_publishing_rate", finAnglesPublishingRate, 15.0);
  _commandFinAnglesTimer = _nodeHandle->createTimer(ros::Duration(1.0 / finAnglesPublishingRate),
                                                   &SetFinDeflection::commandFinAngles, this);
}

void SetFinDeflection::ProcessData(char* message)
{
  if (message == nullptr) return;

  // build new header every time a new message is received
  if (_header != nullptr) delete _header;
  _header = new JausMessageHeader(message);  // 6 bytes
  int index = 6;                             // read data starts from index 6
  if (_PresenceVector != nullptr) delete _PresenceVector;
  int16_t pv = buildPresenceVector(message, index);

  _finId = (int8_t)message[index++];            // 1 byte
  _deflectionAngle = (int8_t)message[index++];  // 1 byte

  //  The OCU sends _finID  from 1 to 4
  if(_finId < 5)
    _finAngles[_finId-1] = JausDataManager::degreesToRadians(_deflectionAngle);
  else
    ROS_ERROR_STREAM("fin ID out of range - finID: " << _finId);

  if (_PresenceVector->IsBitSet((int)0))
  {
    _deflectionRateCmd = (int8_t)message[index++];  // 1 byte
  }
}
int8_t SetFinDeflection::GetFinID() { return _finId; }

int8_t SetFinDeflection::GetDeflectionAngle() { return _deflectionAngle; }

int8_t SetFinDeflection::GetDeflectionRateCmd()
{
  if (_PresenceVector->IsBitSet(0)) return _deflectionRateCmd;

  return 0;
}

void SetFinDeflection::commandFinAngles(const ros::TimerEvent& ev)
{
  fin_control::SetAngles msg;
    for (int i = 0; i < 4; i++)
    {
      msg.fin_angle_in_radians[i] = _finAngles[i];
    }
    _publisher_setAngles.publish(msg);
}

void SetFinDeflection::PublishFinAngles(const bool enable)
{
  if (enable)
    _commandFinAnglesTimer.start();
  else
    _commandFinAnglesTimer.stop();
}
