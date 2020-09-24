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

#include "ReportPoseEstimatorData.h"
#include "JausDataManager.h"

void ReportPoseEstimatorData::init(ros::NodeHandle* nodeHandle, udpserver* udp) {
  _nodeHandle = nodeHandle;
  _udpserver = udp;
  _subscriber_reportPoseData = nodeHandle->subscribe(
      "/pose/corrected_data", 1, &ReportPoseEstimatorData::handleReportPoseEstimatorData, this);
  _myID = "ReportPoseEstimatorData";
  Reset();
}

bool ReportPoseEstimatorData::HasNewData(const pose_estimator::CorrectedData::ConstPtr& msg) {
  // if(m_poseData.altitude != (float)msg->altitude)
  //    return true;
  if (m_poseData.depth != (float)msg->depth) return true;
  if (m_poseData.speed != (float)msg->speed) return true;
  if (m_poseData.roll != (float)msg->rpy_ang.x) return true;
  if (m_poseData.pitch != (float)msg->rpy_ang.y) return true;
  if (m_poseData.course != (float)msg->rpy_ang.z) return true;

  return false;
}

void ReportPoseEstimatorData::Reset() {
  m_poseData.depth = 0;
  // m_poseData.altitude = 0;
  m_poseData.speed = 0;
  m_poseData.roll = 0;
  m_poseData.pitch = 0;
  m_poseData.course = 0;
}

void ReportPoseEstimatorData::handleReportPoseEstimatorData(
    const pose_estimator::CorrectedData::ConstPtr& msg) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime, clock())) return;

  if (!HasNewData(msg)) return;

  // m_poseData.altitude = (float)msg->altitude;
  m_poseData.depth = (float)msg->depth;
  m_poseData.speed = (float)msg->speed;
  m_poseData.roll = (float)msg->rpy_ang.x;
  m_poseData.pitch = (float)msg->rpy_ang.y;
  m_poseData.course = (float)msg->rpy_ang.z;

  if (debug_mode) ROS_INFO("handleReportPoseEstimatorData() is called with new update!");

  DataInfo info = GetPackedMessage(&m_poseData);
  _udpserver->RequestSendingMessage(_myID + "handleReportPoseEstimatorData", info._data,
                                    info._size);
  delete[] info._data;

  _beginTime = clock();
}

DataInfo ReportPoseEstimatorData::GetPackedMessage(void* data) {
  PoseEstimatorData* poseData = reinterpret_cast<PoseEstimatorData*>(data);
  // ROS_INFO("inside ReportPoseEstimatorData::GetPackedMessage() - depth is %f, altitude is %f,
  // speed is %f", poseData->depth, poseData->altitude, poseData->speed);

  // here 4: 2 for presence vector + 5 fields x 4 bytes = 22 bytes
  JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportPoseEstimetorData, 22);

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

  //    ROS_WARN("inside ReportPoseEstimatorData::() - poseData->depth is %f", poseData->depth);
  //    ROS_WARN("inside ReportPoseEstimatorData::() - poseData->speed is %f", poseData->speed);
  //    ROS_WARN("inside ReportPoseEstimatorData::() - poseData->roll is %f", poseData->roll);
  //    ROS_WARN("inside ReportPoseEstimatorData::() - poseData->pitch is %f", poseData->pitch);
  //    ROS_WARN("inside ReportPoseEstimatorData::() - poseData->course is %f", poseData->course);

  // finally the data

  char* temp;
  temp = reinterpret_cast<char*>(&poseData->depth);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char*>(&poseData->speed);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char*>(&poseData->roll);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char*>(&poseData->pitch);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char*>(&poseData->course);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}
