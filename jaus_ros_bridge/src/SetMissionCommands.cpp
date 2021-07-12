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

#include "SetMissionCommands.h"
#include "JausDataManager.h"

#include <mission_control/AbortMission.h>
#include <mission_control/ExecuteMission.h>
#include <mission_control/LoadMission.h>
#include <mission_control/QueryMissions.h>
#include <mission_control/RemoveMissions.h>
#include <std_msgs/Empty.h>

void SetMissionCommands::init(ros::NodeHandle* nodeHandle) {
  _nodeHandle = nodeHandle;
  // All Set command handlings should be moved to here!
  _publisher_Execute_mission =
      _nodeHandle->advertise<mission_control::ExecuteMission>("/mngr/execute_mission", 1, true);
  _publisher_Stop_mission =
      _nodeHandle->advertise<std_msgs::Empty>("/mngr/stop_missions", 1, true);
  _publisher_Abort_mission =
      _nodeHandle->advertise<mission_control::AbortMission>("/mngr/abort_mission", 1, true);
  _publisher_Load_mission =
      _nodeHandle->advertise<mission_control::LoadMission>("/mngr/load_mission", 1, true);
  _publisher_Query_mission =
      _nodeHandle->advertise<mission_control::QueryMissions>("/mngr/query_missions", 1, true);
  _publisher_Remove_mission =
      _nodeHandle->advertise<mission_control::RemoveMissions>("/mngr/remove_missions", 1, true);
}

void SetMissionCommands::ProcessData(char* message, JausCommandID cmdID) {
  if (message == nullptr) return;

  if (_header != nullptr) delete _header;
  _header = new JausMessageHeader(message);  // 6 bytes
  int index = 6;                             // read data starts from index 6
  if (_PresenceVector != nullptr) delete _PresenceVector;
  int16_t pv = buildPresenceVector(message, index);  // 2 butes

  switch (cmdID) {
    case JAUS_COMMAND_ExecuteMission: {
      mission_control::ExecuteMission msg;
      msg.mission_id = (int8_t)message[index++];
      _publisher_Execute_mission.publish(msg);
      if (debug_mode) ROS_INFO("Starting mission... Mission ID is %d ", msg.mission_id);
      break;
    }
    case JAUS_COMMAND_AbortMission: {
      mission_control::AbortMission msg;
      msg.mission_id = (int8_t)message[index++];
      _publisher_Abort_mission.publish(msg);
      if (debug_mode) ROS_INFO("Aborting mission... Mission ID is %d ", msg.mission_id);
      break;
    }
    case JAUS_COMMAND_LoadMission: {
      mission_control::LoadMission msg;
      short size = _header->GetDatasize();
      //             ROS_ERROR("size =  %d", size);

      char path[size];
      strcpy(path, &message[index]);

      msg.mission_file_full_path = string(path);
      if (debug_mode) ROS_INFO("Load mission file path is: %s", msg.mission_file_full_path.c_str());
      _publisher_Load_mission.publish(msg);
      break;
    }
    case JAUS_COMMAND_QueryMissions: {
      mission_control::QueryMissions msg;
      _publisher_Query_mission.publish(msg);
      if (debug_mode) ROS_INFO("Query mission... ");
      break;
    }
    case JAUS_COMMAND_RemoveMissions: {
      mission_control::RemoveMissions msg;
      _publisher_Remove_mission.publish(msg);
      if (debug_mode) ROS_INFO("Remove mission... ");
      break;
    }
  }
}

void SetMissionCommands::StopMissions()
{
  _publisher_Stop_mission.publish(std_msgs::Empty());
}
