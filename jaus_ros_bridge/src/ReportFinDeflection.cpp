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
 * ReportFinDeflection.cpp
 *
 *  Created on: Jun 13, 2019
 *      Author: jsun
 */

#include "ReportFinDeflection.h"
#include "JausDataManager.h"

void ReportFinDeflection::init(ros::NodeHandle* nodeHandle, int8_t finId, udpserver* udp) {
  _nodeHandle = nodeHandle;
  _udpserver = udp;
  _finId = finId;
  _deflectionAngle = -100;
  _subscriber_reportAngle = nodeHandle->subscribe("/fin_control/report_angle", 1,
                                                  &ReportFinDeflection::handleReportAngle, this);
  _myID = "ReportFinDeflection" + std::to_string(finId);
  Reset();
}

DataInfo ReportFinDeflection::GetPackedMessage(void* data) {
  // FinData* finData = reinterpret_cast<FinData*>(data);
  // build new header every time a new message is received
  JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportFinDeflection, 4);
  // here 4:  2 for presence vector + 1 for finID + 1 for angle

  short dataIndex = 0;
  char* newData = new char[header->GetHeadersize() + header->GetDatasize()];

  // start from header
  int index = 0;
  char* headerData = header->GetHeaderData();

  for (index = 0; index < header->GetHeadersize(); index++) {
    newData[index] = headerData[index];
  }

  // jsun - ToByteArray() is not working, so just pad to 0 for now
  newData[index++] = 0;
  newData[index++] = 0;

  // finally the data - finID and fin angle
  newData[index++] = _finId;

  newData[index] = _deflectionAngle;
  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}

void ReportFinDeflection::handleReportAngle(const fin_control::ReportAngle::ConstPtr& msg) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime, clock())) return;

  if (_finId != msg->ID) return;  // do not handle this msg if it's a different finID

  float f_angle = JausDataManager::radiansToDegrees(msg->angle_in_radians);
  if (f_angle < 0)
    f_angle -= .5;
  else
    f_angle += .5;

  int8_t angle = (int8_t)f_angle;

  if (_deflectionAngle != angle) {
    if (debug_mode)
      ROS_INFO("ReportFinDeflection::handleReportAngle() is called! angle is %d", angle);

    _deflectionAngle = angle;
    // if(!_udpserver->InSendProcess())
    //{
    DataInfo info = GetPackedMessage(nullptr);
    _udpserver->RequestSendingMessage(_myID + "handleReportAngle", info._data, info._size);
    delete[] info._data;
    //}
    // else {
    //    ROS_INFO("!!!udpserver is busy sending message, can not send at this time!!!");
    //}
  }
  _beginTime = clock();
}
