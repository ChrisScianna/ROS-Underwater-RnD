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

#include "ReportMissionInfo.h"
#include "JausDataManager.h"

void ReportMissionInfo::init(ros::NodeHandle* nodeHandle, udpserver* udp) {
  _nodeHandle = nodeHandle;
  _udpserver = udp;
  _subscriber_reportMissionState =
      nodeHandle->subscribe("/mngr/report_mission_execute_state", 1,
                            &ReportMissionInfo::handleReportMissionStateData, this);
  _subscriber_reportUploadState = nodeHandle->subscribe(
      "/mngr/report_mission_load_state", 1, &ReportMissionInfo::handleReportUploadData, this);
  _subscriber_reportMissions = nodeHandle->subscribe(
      "/mngr/report_missions", 1, &ReportMissionInfo::handleReportMissions, this);
  _myID = "ReportMissionInfo";
  Reset();
}

void ReportMissionInfo::handleReportMissionStateData(
    const mission_manager::ReportExecuteMissionState::ConstPtr& msg) {
  if (!_udpserver->IsConnected()) return;

  MissionState missionState;

  missionState.timestamp_seconds = (int)msg->header.stamp.toSec();  // in seconds

  missionState.missionID = (ushort)msg->mission_id;
  missionState.execute_mission_state = (ushort)msg->execute_mission_state;

  missionState.current_behavior_id = (ushort)msg->current_behavior_id;
  missionState.current_behavior_name = msg->current_behavior_name;

  DataInfo info = GetPackedMessageMissionState(&missionState);
  _udpserver->RequestSendingMessage(_myID + "handleReportMissionStateData", info._data, info._size);
  delete[] info._data;
}

void ReportMissionInfo::handleReportUploadData(
    const mission_manager::ReportLoadMissionState::ConstPtr& msg) {
  if (!_udpserver->IsConnected()) return;

  UploadState uploadState;

  // uploadState.timestamp = (float) msg->stamp.toSec() *1000; // convert to mili seconds
  uploadState.missionID = (ushort)msg->mission_id;
  uploadState.uploadState = (ushort)msg->load_state;

  if (debug_mode)
    ROS_INFO("handleReportUploadData() is called! missionID is: %d, uploadState is %d",
             uploadState.missionID, uploadState.uploadState);

  DataInfo info = GetPackedMessageUploadState(&uploadState);
  string strID = std::to_string(uploadState.missionID);
  _udpserver->RequestSendingMessage(_myID + "handleReportUploadData" + strID, info._data,
                                    info._size);
  delete[] info._data;
}

void ReportMissionInfo::handleReportMissions(const mission_manager::ReportMissions& msg) {
  // ROS_INFO("handleReportMissions()... msg.missions.size() = %ld", msg.missions.size());
  for (int i = 0; i < msg.missions.size(); ++i) {
    const mission_manager::MissionData& mdata = msg.missions[i];
    if (debug_mode)
      ROS_INFO("Inside handleReportMissions() - Mission Id:[%d] Description:[%s]", mdata.mission_id,
               mdata.mission_description.c_str());

    Mission mission;

    mission.missionID = mdata.mission_id;
    mission.description = mdata.mission_description;

    DataInfo info = GetPackedMessageMissions(&mission);
    string strID = std::to_string(mission.missionID);
    _udpserver->RequestSendingMessage(_myID + "handleReportMissions" + strID, info._data,
                                      info._size);
    delete[] info._data;
  }
}

DataInfo ReportMissionInfo::GetPackedMessageMissions(void* data) {
  Mission* mission = reinterpret_cast<Mission*>(data);
  // TODO
  // here 10: 2 for presence vector + 2 for mission ID and 2 for mission state and 4 for stamp
  int datalength = 2 + 2 + mission->description.length();
  JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportMissions, datalength);

  char* newData = new char[header->GetHeadersize() + header->GetDatasize()];
  // start from header
  int index = 0;
  char* headerData = header->GetHeaderData();
  for (index = 0; index < header->GetHeadersize(); index++) {
    newData[index] = headerData[index];
  }
  // jsun - PV's ToByteArray() is not working, so just pad presence vector to 0 for now
  newData[index++] = 0;
  newData[index++] = 0;

  // finally the data
  char* temp;
  temp = reinterpret_cast<char*>(&mission->missionID);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  // index++;
  for (int i = 0; i < mission->description.length(); i++) {
    newData[index + i] = mission->description[i];
  }

  DataInfo info;

  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}

DataInfo ReportMissionInfo::GetPackedMessageMissionState(void* data) {
  MissionState* missionStateData = reinterpret_cast<MissionState*>(data);

  // here datalength: 2 for presence vector + 2 for mission ID + 2 for mission state + 4 for time
  // stamp
  // + 2 for current_behavior_id + length of current_behavior_name
  int datalength = 2 + 2 + 2 + 4 + 2 + missionStateData->current_behavior_name.length();

  JausMessageHeader* header =
      new JausMessageHeader(JAUS_COMMAND_ReportExecuteMissionState, datalength);

  char* newData = new char[header->GetHeadersize() + header->GetDatasize()];
  // start from header
  int index = 0;
  char* headerData = header->GetHeaderData();

  for (index = 0; index < header->GetHeadersize(); index++) {
    newData[index] = headerData[index];
  }

  // jsun - PV's ToByteArray() is not working, so just pad presence vector to 0 for now
  newData[index++] = 0;
  newData[index++] = 0;

  // finally the data

  char* temp;
  temp = reinterpret_cast<char*>(&missionStateData->missionID);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char*>(&missionStateData->execute_mission_state);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char*>(&missionStateData->timestamp_seconds);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[3];
  newData[index++] = temp[4];

  temp = reinterpret_cast<char*>(&missionStateData->current_behavior_id);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  for (int i = 0; i < missionStateData->current_behavior_name.length(); i++) {
    newData[index + i] = missionStateData->current_behavior_name[i];
  }

  // ROS_INFO("inside ReportMissionInfo::GetPackedMessageMissionState() - current_behavior_id=%ld,
  // current_behavior_name=%s", missionStateData->current_behavior_name,
  // missionStateData->current_behavior_name.c_str());

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}

DataInfo ReportMissionInfo::GetPackedMessageUploadState(void* data) {
  UploadState* loadStateData = reinterpret_cast<UploadState*>(data);
  // ROS_INFO("inside ReportMissionInfo::GetPackedMessageUploadState()");

  // here 6: 2 for presence vector + 2 for mission ID and 2 for upload state
  JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportLoadMissionState, 6);

  char* newData = new char[header->GetHeadersize() + header->GetDatasize()];
  // start from header
  int index = 0;
  char* headerData = header->GetHeaderData();

  for (index = 0; index < header->GetHeadersize(); index++) {
    newData[index] = headerData[index];
  }

  // jsun - PV's ToByteArray() is not working, so just pad presence vector to 0 for now
  newData[index++] = 0;
  newData[index++] = 0;

  // finally the data

  char* temp;
  temp = reinterpret_cast<char*>(&loadStateData->missionID);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char*>(&loadStateData->uploadState);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}
