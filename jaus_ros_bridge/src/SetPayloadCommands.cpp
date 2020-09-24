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

#include "SetPayloadCommands.h"
#include "JausDataManager.h"

#include <payload_manager/PayloadCommand.h>

void SetPayloadCommands::init(ros::NodeHandle* nodeHandle) {
  _nodeHandle = nodeHandle;
  _publisher_payload_command =
      _nodeHandle->advertise<payload_manager::PayloadCommand>("/payload_manager/command", 1, true);
}

void SetPayloadCommands::ProcessData(char* message, JausCommandID cmdID) {
  if (message == nullptr) return;

  if (_header != nullptr) delete _header;
  _header = new JausMessageHeader(message);  // 6 bytes
  int index = 6;                             // read data starts from index 6
  if (_PresenceVector != nullptr) delete _PresenceVector;
  int16_t pv = buildPresenceVector(message, index);  // 2 butes

  short size = _header->GetDatasize();
  //             ROS_ERROR("size =  %d", size);

  char cmd[size];
  strcpy(cmd, &message[index]);

  payload_manager::PayloadCommand msg;
  msg.command = string(cmd);

  ROS_INFO("Payload message %s published.", msg.command.c_str());
  _publisher_payload_command.publish(msg);
}
