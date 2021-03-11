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

/*
 * SetFinDeflection.cpp
 *
 *  Created on: Jun 11, 2019
 *      Author: jsun
 */

#include "SetFinDeflection.h"
#include <fin_control/SetAngle.h>
#include "JausDataManager.h"

void SetFinDeflection::init(ros::NodeHandle* nodeHandle) {
  _nodeHandle = nodeHandle;
  _publisher_setAngle =
      _nodeHandle->advertise<fin_control::SetAngle>("/fin_control/set_angle", 1, true);
}

void SetFinDeflection::ProcessData(char* message) {
  if (message == nullptr) return;

  // build new header every time a new message is received
  if (_header != nullptr) delete _header;
  _header = new JausMessageHeader(message);  // 6 bytes
  int index = 6;                             // read data starts from index 6
  if (_PresenceVector != nullptr) delete _PresenceVector;
  int16_t pv = buildPresenceVector(message, index);
  // printf("Presence vector is: %d \n", pv);

  _finId = (int8_t)message[index++];  // 1 byte
  printf("FinID is: %d \n", _finId);

  _deflectionAngle = (int8_t)message[index++];  // 1 byte
  printf("_deflectionAngle in degree is: %d \n", _deflectionAngle);

  fin_control::SetAngle msg;
  msg.ID = _finId;
  msg.angle_in_radians = JausDataManager::degreesToRadians(_deflectionAngle);
  // int count = 0;
  // while(count<5)
  //{
  _publisher_setAngle.publish(msg);
  //    count++;

  //    sleep(0.05);
  //    printf("sleeping for 50ms. count= %d \n", count);
  //}

  if (_PresenceVector->IsBitSet((int)0)) {
    _deflectionRateCmd = (int8_t)message[index++];  // 1 byte
  }
}

int8_t SetFinDeflection::GetFinID() { return _finId; }

int8_t SetFinDeflection::GetDeflectionAngle() { return _deflectionAngle; }

int8_t SetFinDeflection::GetDeflectionRateCmd() {
  if (_PresenceVector->IsBitSet(0)) return _deflectionRateCmd;

  return 0;
}
