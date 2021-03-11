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
 * ReportPowerPlantStatus.cpp
 *
 *  Created on: Jun 18, 2019
 *      Author: jsun
 */

#include "ReportPowerPlantStatus.h"
#include "JausDataManager.h"

pthread_mutex_t mu_update_powerplant_data;

void ReportPowerPlantStatus::init(ros::NodeHandle* nodeHandle, udpserver* udp) {
  _nodeHandle = nodeHandle;
  _udpserver = udp;
  _subscriber_reportRPM = nodeHandle->subscribe("/thruster_control/report_rpm", 1,
                                                &ReportPowerPlantStatus::handleReportRPM, this);
  _subscriber_reportMotorTemp =
      nodeHandle->subscribe("/thruster_control/report_motor_temp", 1,
                            &ReportPowerPlantStatus::handleReportMotorTemp, this);
  _myID = "ReportPowerPlantStatus";
  Reset();
}

void ReportPowerPlantStatus::Reset() {
  _engineData._engineId = 0;
  _engineData._engineState = 0;
  _engineData._rpm = 30000;
  _engineData._engineTemp = 30000;
}

DataInfo ReportPowerPlantStatus::GetPackedMessage(void* data) {
  EngineData* enginedata = reinterpret_cast<EngineData*>(data);
  pthread_mutex_lock(&mu_update_powerplant_data);
  // build new header every time a new message is received
  JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportPowerPlantStatus, 8);
  // here 8:  2 for presence vector + 1 for engineID + 1 for state + 2 for RPM + 2 for engineTemp

  char* newData = new char[header->GetHeadersize() + header->GetDatasize()];

  // start from header
  int index = 0;
  char* headerData = header->GetHeaderData();  // header size should be 6
  for (index = 0; index < header->GetHeadersize(); index++) {
    newData[index] = headerData[index];
  }

  index = 6;
  //    //followed by presence vector
  JausPresenceVector* _PresenceVector = new JausPresenceVector(0);  // initialized with 0 for now;

  char* pvData = _PresenceVector->ToByteArray();  // vp size should be 2
  // newData[index++] = pvData[0];
  // newData[index++] = pvData[1];
  delete _PresenceVector;
  // jsun - ToByteArray() is not working, so just pad to 0 for now
  newData[index++] = 0;
  newData[index++] = 0;

  // index = 8;
  // finally the data - enginID, state, and fin rpm
  newData[index++] = enginedata->_engineId;

  newData[index++] = enginedata->_engineState;  // index = 9

  char* temp;
  // This takes 2 bytes
  temp = reinterpret_cast<char*>(&enginedata->_rpm);
  // Or
  // temp = (char*)&enginedata->_rpm;
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  // This takes 2 bytes
  uint8_t bytes[sizeof(int16_t)];
  *(int16_t*)(bytes) = enginedata->_engineTemp;
  // uint8_t* bytes = Int16ToByteArray(_engineTemp);
  newData[index++] = bytes[0];
  newData[index++] = bytes[1];

  pthread_mutex_unlock(&mu_update_powerplant_data);

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}

uint8_t* ReportPowerPlantStatus::Int16ToByteArray(int16_t inVal) {
  //    uint8_t* bytes = new uint8_t(sizeof(int16_t));
  //    *(int16_t*)(*bytes) = inVal;
  //    return bytes;

  uint8_t* b = new uint8_t(sizeof(int16_t));

  (int16_t&)b = inVal;
  return b;

  //        std::string s = std::to_string(inVal);
  //        char const * st = s.c_str();

  //        char * val = new char(sizeof(st));

  //        for(int i=0; i < (int)sizeof(s.c_str()); i++)
  //        {
  //                val[i] = st[i];
  //        }

  //        return val;
}

void ReportPowerPlantStatus::handleReportRPM(const thruster_control::ReportRPM::ConstPtr& msg) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime, clock())) return;

  if (_engineData._rpm != (int)msg->rpms) {
    if (debug_mode) ROS_INFO("handleReportRPM() is called with new RPM to report.");

    _engineData._rpm = (int)msg->rpms;
    DataInfo info = GetPackedMessage(&_engineData);

    _udpserver->RequestSendingMessage(_myID + "handleReportRPM", info._data, info._size);
    delete[] info._data;
  }
  _beginTime = clock();
}

void ReportPowerPlantStatus::handleReportMotorTemp(
    const thruster_control::ReportMotorTemperature::ConstPtr& msg) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime1, clock())) return;

  // ROS_INFO("inside ReportPowerPlantStatus::handleReportMotorTemp - received msg: %f",
  // msg->motor_temp);
  if (_engineData._engineTemp != (int)msg->motor_temp) {
    if (debug_mode) ROS_INFO("handleReportMotorTemp() is called with new temperature to report.");

    _engineData._engineTemp = (int)msg->motor_temp;
    if (!_udpserver->InSendProcess()) {
      DataInfo info = GetPackedMessage(&_engineData);
      _udpserver->RequestSendingMessage(_myID + "handleReportMotorTemp", info._data, info._size);
      // ROS_INFO("New temperature %d is reported.", _engineData._engineTemp);
      delete[] info._data;
    } else {
      // ROS_INFO("!!!udpserver is busy sending message, can not send at this time!!!");
    }
  }

  _beginTime1 = clock();
}
